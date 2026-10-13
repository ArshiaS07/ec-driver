#include "ec/motion/control_loop.hpp"

#include <algorithm>
#include <exception>
#include <format>
#include <utility>

#include "ec/command/movement.hpp"
#include "ec/feedback/frame.hpp"
#include "ec/feedback/state.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
// windows.h first
#include <timeapi.h>
#endif

namespace ec::motion {

namespace detail {

struct StartRequest {
    Ticket ticket;
    AnyActivity activity;
    std::promise<std::expected<void, Error>> done;
};
struct CancelRequest {
    Ticket ticket;
};
struct AcknowledgeRequest {
    std::promise<std::expected<void, Error>> done;
};
struct PendingCall {
    command::Request request;
    command::Kind kind;
    std::promise<std::expected<command::Json, Error>> done;
};
using Request = std::variant<StartRequest, CancelRequest, AcknowledgeRequest, PendingCall>;

/// What the applications' threads and the loop share.
struct Requests {
    std::mutex mutex;
    std::vector<Request> pending;  // guarded by mutex
    bool closed = false;           // guarded by mutex: the loop has ended
    std::atomic<bool> stop{false};
    std::atomic<Ticket> next_ticket{1};
};

}  // namespace detail

namespace {

using std::chrono::duration_cast;
using std::chrono::milliseconds;

// How long an application waits for the loop to answer a request it waits on.
constexpr auto kAnswerTimeout = std::chrono::seconds{5};

Error loop_ended(const std::string& why) { return Error{Error::Kind::Disconnected, why, {}}; }

// Windows wakes sleeping threads in steps of its system timer, 15.6 ms by
// default, which would make most cycles late: ask for 1 ms while the loop runs.
struct TimerResolution {
#if defined(_WIN32)
    TimerResolution() noexcept { timeBeginPeriod(1); }
    ~TimerResolution() { timeEndPeriod(1); }
    TimerResolution(const TimerResolution&) = delete;
    TimerResolution& operator=(const TimerResolution&) = delete;
#endif
};

}  // namespace

// ------------------------------------------------------------------------- Job

std::expected<void, Error> Job::wait() const {
    if (!result_.valid()) return std::unexpected(Error{Error::Kind::Refused, "not a job", {}});
    return result_.get();
}

std::optional<std::expected<void, Error>> Job::wait_for(std::chrono::nanoseconds timeout) const {
    if (!result_.valid()) return std::unexpected(Error{Error::Kind::Refused, "not a job", {}});
    if (result_.wait_for(timeout) != std::future_status::ready) return std::nullopt;
    return result_.get();
}

bool Job::done() const {
    return result_.valid() && result_.wait_for(std::chrono::seconds{0}) == std::future_status::ready;
}

void Job::cancel() const {
    if (!requests_) return;
    const std::lock_guard lock(requests_->mutex);
    if (!requests_->closed) requests_->pending.emplace_back(detail::CancelRequest{ticket_});
}

// ----------------------------------------------------------- the applications' side

ControlLoop::ControlLoop(command::Client client, Reconnect reconnect,
                         std::shared_ptr<const Latest<feedback::Sample>> state, const ControlOptions& options)
    : client_(std::move(client)),
      reconnect_(std::move(reconnect)),
      state_(std::move(state)),
      options_(options),
      requests_(std::make_shared<detail::Requests>()),
      executor_(options.executor),
      thread_([this](const std::stop_token& stop) { run(stop); }) {}

ControlLoop::~ControlLoop() {
    thread_.request_stop();
    if (thread_.joinable()) thread_.join();
}

Job ControlLoop::run(AnyActivity activity) {
    std::promise<std::expected<void, Error>> done;
    auto result = done.get_future().share();
    const Ticket ticket = requests_->next_ticket.fetch_add(1);
    {
        const std::lock_guard lock(requests_->mutex);
        if (requests_->closed)
            done.set_value(std::unexpected(loop_ended("the control loop has ended")));
        else
            requests_->pending.emplace_back(detail::StartRequest{ticket, std::move(activity), std::move(done)});
    }
    return Job(ticket, std::move(result), requests_);
}

void ControlLoop::stop() noexcept { requests_->stop.store(true); }

std::expected<void, Error> ControlLoop::acknowledge() {
    std::promise<std::expected<void, Error>> done;
    auto answer = done.get_future();
    {
        const std::lock_guard lock(requests_->mutex);
        if (requests_->closed) return std::unexpected(loop_ended("the control loop has ended"));
        requests_->pending.emplace_back(detail::AcknowledgeRequest{std::move(done)});
    }
    if (answer.wait_for(kAnswerTimeout) != std::future_status::ready)
        return std::unexpected(Error{Error::Kind::Timeout, "the control loop did not answer", {}});
    return answer.get();
}

std::expected<command::Json, Error> ControlLoop::call(command::Request request, command::Kind kind) {
    if (kind == command::Kind::Motion)
        return std::unexpected(Error{
            Error::Kind::Refused, std::format("{} is a motion command: run it in an activity", request.method), {}});
    std::promise<std::expected<command::Json, Error>> done;
    auto answer = done.get_future();
    {
        const std::lock_guard lock(requests_->mutex);
        if (requests_->closed) return std::unexpected(loop_ended("the control loop has ended"));
        requests_->pending.emplace_back(detail::PendingCall{std::move(request), kind, std::move(done)});
    }
    if (answer.wait_for(kAnswerTimeout) != std::future_status::ready)
        return std::unexpected(Error{Error::Kind::Timeout, "the control loop did not answer", {}});
    return answer.get();
}

Status ControlLoop::status() const {
    if (auto e = status_.get()) return e->value;
    return Status{};
}

std::optional<Status> ControlLoop::next_status(std::uint64_t cycles, std::chrono::nanoseconds timeout) const {
    if (auto e = status_.wait_newer(cycles, timeout)) return e->value;
    return std::nullopt;
}

// ------------------------------------------------------------------- the loop

void ControlLoop::run(const std::stop_token& stop) {
    [[maybe_unused]] const TimerResolution timer_resolution;
    const auto period = std::chrono::duration_cast<Clock::duration>(options_.period);
    auto deadline = Clock::now();
    std::optional<Clock::time_point> shutdown_by;
    Outbox out;
    std::vector<Reply> next_replies;
    try {
        for (;;) {
            deadline += period;
            std::this_thread::sleep_until(deadline);
            const auto now = Clock::now();

            // Timing: count late starts; far behind, give up catching up.
            const auto lateness = now - deadline;
            ++current_.cycles;
            if (lateness > period / 2) {
                ++current_.late_cycles;
                ++consecutive_late_;
                current_.worst_lateness = std::max<std::chrono::nanoseconds>(current_.worst_lateness, lateness);
            } else {
                consecutive_late_ = 0;
            }
            if (lateness > 10 * period) deadline = now;
            if (consecutive_late_ >= options_.max_late_cycles && executor_.current()) {
                executor_.fail(Error{Error::Kind::Overrun,
                                     std::format("{} cycles in a row started late, the last by {} ms",
                                                 consecutive_late_, duration_cast<milliseconds>(lateness).count()),
                                     {}});
                consecutive_late_ = 0;
            }

            // Shutting down: stop what runs first, within a time limit.
            if (stop.stop_requested()) {
                if (!shutdown_by) {
                    shutdown_by = now + options_.shutdown_timeout;
                    executor_.stop();
                }
                if (!executor_.current() || now > *shutdown_by) break;
            }

            // The connection.
            if (!client_.is_connected()) {
                if (!disconnect_reported_) {
                    executor_.fail(Error{Error::Kind::Disconnected, "the command connection was lost", {}});
                    disconnect_reported_ = true;
                }
                if (reconnect_ && now - last_reconnect_ >= options_.reconnect_every) {
                    last_reconnect_ = now;
                    if (auto client = reconnect_()) {
                        client_ = std::move(*client);
                        disconnect_reported_ = false;
                    }
                }
            }

            const Cycle cycle = read_inputs(now);
            if (requests_->stop.exchange(false)) executor_.stop();
            serve_requests(cycle);
            out.clear();
            executor_.cycle(cycle, out);
            next_replies.clear();
            send(out, next_replies);
            serve_calls();
            std::swap(replies_, next_replies);  // the cycle above no longer looks at them
            publish(cycle);  // before the jobs end: a thread woken by its job sees the status that ended it
            finish_jobs();
        }
    } catch (const std::exception& e) {
        const auto stopped = client_.call(command::to_request(command::Stop{}), options_.reply_timeout);
        static_cast<void>(stopped);  // the last resort: nothing more can be done if it fails
        current_.fault = Error{Error::Kind::Internal, e.what(), {}};
        current_.mode = Mode::Fault;
        status_.publish(current_);
        fail_everything(std::string("the control loop failed: ") + e.what());
        return;
    }
    fail_everything("the control loop has ended");
}

Cycle ControlLoop::read_inputs(Clock::time_point now) const {
    Cycle c;
    c.now = now;
    c.period = options_.period;
    c.replies = replies_;
    c.state_age = std::chrono::hours{1};  // no state: not fresh
    if (const auto newest = state_->get()) {
        if (const auto frame = feedback::decode(newest->value.raw)) {
            c.state = feedback::to_si(*frame);
            c.state_seq = newest->seq;
            // Measured now, not at the cycle's start: a frame that arrived since then
            // would otherwise look as if it came from the future, which is not fresh.
            c.state_age = Clock::now() - newest->value.arrival;
        }
    }
    c.conditions = observe(c.state, c.state_age, options_.thresholds);
    return c;
}

void ControlLoop::serve_requests(const Cycle& cycle) {
    std::vector<detail::Request> pending;
    {
        const std::lock_guard lock(requests_->mutex);
        pending.swap(requests_->pending);
    }
    for (auto& request : pending) {
        if (auto* start = std::get_if<detail::StartRequest>(&request)) {
            if (auto started = executor_.start(start->ticket, std::move(start->activity), cycle); !started)
                start->done.set_value(std::unexpected(std::move(started.error())));
            else
                open_jobs_.emplace_back(start->ticket, std::move(start->done));
        } else if (const auto* cancel = std::get_if<detail::CancelRequest>(&request)) {
            executor_.cancel(cancel->ticket);
        } else if (auto* ack = std::get_if<detail::AcknowledgeRequest>(&request)) {
            ack->done.set_value(executor_.acknowledge());
        } else if (auto* call = std::get_if<detail::PendingCall>(&request)) {
            calls_.push_back(std::move(*call));
        }
    }
}

void ControlLoop::send(const Outbox& out, std::vector<Reply>& replies) {
    for (const Outgoing& o : out.items()) {
        if (!o.expects_reply) {
            const auto sent = client_.send_only(o.request);
            static_cast<void>(sent);  // a failure closes the connection, which the next cycle reports
            continue;
        }
        replies.push_back(Reply{o.request.method, client_.call(o.request, options_.reply_timeout)});
    }
}

void ControlLoop::serve_calls() {
    for (auto& call : calls_) {
        const bool busy = executor_.current().has_value();
        if (busy && !command::is_read_only(call.kind)) {
            call.done.set_value(std::unexpected(Error{Error::Kind::Refused,
                                                      std::format("{} changes the controller: not while {} runs",
                                                                  call.request.method, executor_.current_name()),
                                                      {}}));
            continue;
        }
        auto result = client_.call(call.request, options_.reply_timeout);
        if (result)
            call.done.set_value(std::move(*result));
        else
            call.done.set_value(std::unexpected(command_failed(call.request.method, result.error())));
    }
    calls_.clear();
}

void ControlLoop::finish_jobs() {
    for (auto& finished : executor_.take_finished()) {
        const auto it = std::ranges::find(open_jobs_, finished.ticket,
                                          &std::pair<Ticket, std::promise<std::expected<void, Error>>>::first);
        if (it == open_jobs_.end()) continue;
        it->second.set_value(std::move(finished.result));
        open_jobs_.erase(it);
    }
}

void ControlLoop::publish(const Cycle& cycle) {
    current_.mode = executor_.mode();
    current_.fault = executor_.fault();
    current_.activity = std::string(executor_.current_name());
    current_.conditions = cycle.conditions;
    current_.connected = client_.is_connected();
    status_.publish(current_);
}

void ControlLoop::fail_everything(const std::string& why) {
    for (auto& [ticket, done] : open_jobs_) done.set_value(std::unexpected(loop_ended(why)));
    open_jobs_.clear();
    for (auto& call : calls_) call.done.set_value(std::unexpected(loop_ended(why)));
    calls_.clear();
    std::vector<detail::Request> pending;
    {
        const std::lock_guard lock(requests_->mutex);
        requests_->closed = true;
        pending.swap(requests_->pending);
    }
    for (auto& request : pending) {
        if (auto* start = std::get_if<detail::StartRequest>(&request))
            start->done.set_value(std::unexpected(loop_ended(why)));
        else if (auto* ack = std::get_if<detail::AcknowledgeRequest>(&request))
            ack->done.set_value(std::unexpected(loop_ended(why)));
        else if (auto* call = std::get_if<detail::PendingCall>(&request))
            call->done.set_value(std::unexpected(loop_ended(why)));
    }
}

}  // namespace ec::motion
