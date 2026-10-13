#pragma once
/// @file
/// The core of supervision: runs at most one activity, checks the conditions it
/// needs every cycle, always closes it, and latches faults (docs/motion.md 5.6).
/// Pure: no I/O, no threads, no clocks; the control thread drives it once per
/// cycle, and tests drive it with scripted cycles.

#include <chrono>
#include <cstdint>
#include <expected>
#include <optional>
#include <string_view>
#include <vector>

#include "ec/motion/activity.hpp"
#include "ec/motion/error.hpp"

namespace ec::motion {

/// Identifies one started activity, so its result can be matched to its job.
using Ticket = std::uint64_t;

/// What the executor remembers; everything else is recomputed from the newest state.
enum class Mode {
    Idle,     ///< nothing runs; an activity may start
    Running,  ///< an activity steps
    Closing,  ///< an activity is being closed: done, cancelled, stopped or faulted
    Fault,    ///< a fault is latched; nothing starts until acknowledge()
};

[[nodiscard]] std::string_view to_string(Mode m) noexcept;

/// An activity that has ended, and how.
struct Finished {
    Ticket ticket = 0;
    std::expected<void, Error> result;
};

/// Runs one activity at a time; see the file comment. Not thread-safe: the
/// control thread owns it.
class Executor {
public:
    struct Options {
        /// How long close() may take before the activity is dropped with a fault
        /// (after a last-resort stop). Must cover a stream's lookahead.
        std::chrono::nanoseconds close_timeout{std::chrono::seconds{2}};
    };

    Executor() = default;
    explicit Executor(const Options& options) : options_(options) {}

    /// Starts `activity` under `ticket`, to be stepped in this cycle (call before
    /// cycle()). Refused, with nothing latched, while another activity is open,
    /// while a fault is latched, or if a start condition is missing in `now`.
    [[nodiscard]] std::expected<void, Error> start(Ticket ticket, AnyActivity activity, const Cycle& now);

    /// Ends the activity of `ticket` gracefully at the next cycle; ignored for
    /// any other ticket.
    void cancel(Ticket ticket) noexcept;

    /// Ends whatever runs, urgently, at the next cycle. When nothing runs, the
    /// next cycle sends a plain stop to the controller instead.
    void stop() noexcept;

    /// A fault the control thread found itself (a late loop, a lost connection):
    /// at the next cycle it ends the open activity and latches, or just latches.
    void fail(Error error);

    /// Clears a latched fault. Refused while an activity is still open.
    [[nodiscard]] std::expected<void, Error> acknowledge();

    /// One cycle: applies the requests above, then steps or closes the open
    /// activity, writing its commands into `out`.
    void cycle(const Cycle& c, Outbox& out);

    /// The activities that ended since the last call, oldest first.
    [[nodiscard]] std::vector<Finished> take_finished();

    [[nodiscard]] Mode mode() const noexcept { return mode_; }
    [[nodiscard]] const std::optional<Error>& fault() const noexcept { return fault_; }
    /// The open activity's ticket and name, while one is open.
    [[nodiscard]] std::optional<Ticket> current() const noexcept;
    [[nodiscard]] std::string_view current_name() const noexcept;

private:
    void begin_close(End why, std::expected<void, Error> result, Clock::time_point now);
    void finish(std::expected<void, Error> result);

    Options options_;
    Mode mode_ = Mode::Idle;
    std::optional<AnyActivity> activity_;
    Ticket ticket_ = 0;
    End why_ = End::Done;
    std::expected<void, Error> result_;  // what the open activity ends with once closed
    Clock::time_point close_started_{};
    bool stop_requested_ = false;
    std::optional<Ticket> cancel_requested_;
    std::optional<Error> failure_;  // from fail(), applied at the next cycle
    std::optional<Error> fault_;
    std::vector<Finished> finished_;
};

}  // namespace ec::motion
