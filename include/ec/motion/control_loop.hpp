#pragma once
/// @file
/// The control thread (ADR 0014, docs/motion.md 5.2 and 6.1): the only user of
/// the command connection. Every period, at an absolute deadline, it reads the
/// newest state and the applications' requests, lets the executor step or
/// close the activity, sends the commands that produced, and publishes its
/// status. Applications talk to it through requests and the handles below;
/// none of their code runs on its thread.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "ec/command/client.hpp"
#include "ec/command/error.hpp"
#include "ec/command/json.hpp"
#include "ec/command/request.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/latest.hpp"
#include "ec/motion/activity.hpp"
#include "ec/motion/conditions.hpp"
#include "ec/motion/executor.hpp"

namespace ec::motion {

/// The loop's settings (docs/motion.md 7.2).
struct ControlOptions {
    std::chrono::milliseconds period{8};  ///< the cycle, and a stream's sampling period
    /// Per command: a reply that takes longer fails it. Generous on purpose: the
    /// cycle's timing is watched separately (max_late_cycles).
    std::chrono::milliseconds reply_timeout{50};
    /// While an activity runs, this many consecutive cycles starting more than
    /// half a period late are a fault (Kind::Overrun).
    int max_late_cycles = 3;
    Thresholds thresholds{};
    Executor::Options executor{};
    std::chrono::milliseconds reconnect_every{1'000};   ///< while the command connection is down
    std::chrono::milliseconds shutdown_timeout{3'000};  ///< destruction stops an open activity within this
};

/// What the loop publishes at the end of every cycle.
struct Status {
    Mode mode = Mode::Idle;
    std::optional<Error> fault{};
    std::string activity{};   ///< the open activity's name; empty while none is open
    Conditions conditions{};  ///< what held in the cycle
    bool connected = false;   ///< the command connection is up
    std::uint64_t cycles = 0;
    std::uint64_t late_cycles = 0;  ///< cycles that started more than half a period late
    std::chrono::nanoseconds worst_lateness{};
};

namespace detail {
struct Requests;
struct PendingCall;
}  // namespace detail

/// The handle of a started activity. Copies share it. Dropping every handle
/// leaves the activity running (like dropping a std::future); cancel() ends it.
class Job {
public:
    Job() = default;

    /// Waits for the activity to end: nothing if it finished, else why not.
    [[nodiscard]] std::expected<void, Error> wait() const;
    /// Waits up to `timeout`: nullopt if it is still running.
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout) const;
    [[nodiscard]] bool done() const;
    /// Ends the activity gracefully at the next cycle (Error::Kind::Cancelled).
    void cancel() const;
    [[nodiscard]] Ticket ticket() const noexcept { return ticket_; }
    [[nodiscard]] bool valid() const noexcept { return result_.valid(); }

private:
    friend class ControlLoop;
    Job(Ticket ticket, std::shared_future<std::expected<void, Error>> result,
        std::shared_ptr<detail::Requests> requests)
        : ticket_(ticket), result_(std::move(result)), requests_(std::move(requests)) {}

    Ticket ticket_ = 0;
    std::shared_future<std::expected<void, Error>> result_;
    std::shared_ptr<detail::Requests> requests_;
};

/// See the file comment. Starts its thread on construction; neither copyable
/// nor movable. Every member function may be called from any thread.
class ControlLoop {
public:
    /// Connects the command port again after a loss; called on the loop's thread.
    using Reconnect = std::function<std::expected<command::Client, command::Error>()>;

    /// `state` is the newest monitoring sample, filled by a feedback::Receiver.
    ControlLoop(command::Client client, Reconnect reconnect, std::shared_ptr<const Latest<feedback::Sample>> state,
                const ControlOptions& options);

    /// Stops an open activity (as stop() does, waiting up to
    /// ControlOptions::shutdown_timeout for it to close), then the thread.
    /// Jobs still open end with Kind::Disconnected.
    ~ControlLoop();

    ControlLoop(const ControlLoop&) = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;
    ControlLoop(ControlLoop&&) = delete;
    ControlLoop& operator=(ControlLoop&&) = delete;

    /// Starts `activity` at the next cycle. The job ends with Kind::Refused if it
    /// cannot start (another one is open, a fault is latched, a start condition is missing).
    [[nodiscard]] Job run(AnyActivity activity);

    /// Ends whatever runs as fast as its family allows, at the next cycle; when
    /// nothing runs, sends a plain stop to the controller.
    void stop() noexcept;

    /// Clears a latched fault (at the next cycle, waited for). Refused while an
    /// activity is still closing.
    [[nodiscard]] std::expected<void, Error> acknowledge();

    /// Sends one call as soon as the control thread is free, between its cycles
    /// (a cycle that is due goes first), and waits for its unwrapped result: a
    /// sequence of calls costs a round trip each, not a cycle each. Queries and
    /// computations always; settings, actions and sessions only while no
    /// activity is open; motion commands never (they belong to activities):
    /// those are Kind::Refused. A failed call is Kind::Command, or
    /// Kind::Disconnected.
    [[nodiscard]] std::expected<command::Json, Error> call(command::Request request, command::Kind kind);

    /// The newest status.
    [[nodiscard]] Status status() const;
    /// The status of the first cycle after the one numbered `cycles`, waiting up to `timeout`.
    [[nodiscard]] std::optional<Status> next_status(std::uint64_t cycles, std::chrono::nanoseconds timeout) const;

private:
    void run(const std::stop_token& stop);
    void wait_until(Clock::time_point deadline);  // answers calls while it waits
    void serve_requests(const Cycle& cycle);
    void send(const Outbox& out, std::vector<Reply>& replies);
    void serve_calls();
    void finish_jobs();
    void publish(const Cycle& cycle);
    void fail_everything(const std::string& why);
    [[nodiscard]] Cycle read_inputs(Clock::time_point now) const;

    command::Client client_;
    Reconnect reconnect_;
    std::shared_ptr<const Latest<feedback::Sample>> state_;
    ControlOptions options_;
    std::shared_ptr<detail::Requests> requests_;
    Executor executor_;  // the loop thread's own: no lock
    std::vector<Reply> replies_;
    std::vector<std::pair<Ticket, std::promise<std::expected<void, Error>>>> open_jobs_;
    std::vector<detail::PendingCall> calls_;  // one-off calls waiting for the send step
    Latest<Status> status_;
    Status current_;
    bool disconnect_reported_ = false;
    Clock::time_point last_reconnect_{};
    int consecutive_late_ = 0;
    std::jthread thread_;  // last: starts once everything above exists
};

}  // namespace ec::motion
