#include "ec/motion/executor.hpp"

#include <format>
#include <utility>

#include "ec/command/movement.hpp"

namespace ec::motion {

std::string_view to_string(Mode m) noexcept {
    switch (m) {
        case Mode::Idle: return "idle";
        case Mode::Running: return "running";
        case Mode::Closing: return "closing";
        case Mode::Fault: return "fault";
    }
    return "?";
}

std::expected<void, Error> Executor::start(Ticket ticket, AnyActivity activity, const Cycle& now) {
    if (mode_ == Mode::Fault)
        return std::unexpected(Error{
            Error::Kind::Refused,
            std::format("a fault is latched ({}); acknowledge it first", fault_ ? describe(*fault_) : std::string("?")),
            {}});
    if (mode_ != Mode::Idle)
        return std::unexpected(Error{Error::Kind::Refused, std::format("busy: {} is still open", current_name()), {}});
    const Conditions missing = activity.needs().to_start.missing_from(now.conditions);
    if (!missing.empty())
        return std::unexpected(Error{Error::Kind::Refused,
                                     std::format("{} cannot start without: {}", activity.name(), to_string(missing)),
                                     missing});
    activity_.emplace(std::move(activity));
    ticket_ = ticket;
    mode_ = Mode::Running;
    return {};
}

void Executor::cancel(Ticket ticket) noexcept {
    if (activity_ && ticket == ticket_) cancel_requested_ = ticket;
}

void Executor::stop() noexcept { stop_requested_ = true; }

void Executor::fail(Error error) {
    if (!failure_) failure_ = std::move(error);
}

std::expected<void, Error> Executor::acknowledge() {
    if (mode_ == Mode::Running || mode_ == Mode::Closing)
        return std::unexpected(Error{Error::Kind::Refused, std::format("{} is still open", current_name()), {}});
    fault_.reset();
    mode_ = Mode::Idle;
    return {};
}

void Executor::cycle(const Cycle& c, Outbox& out) {
    // Requests, most urgent first.
    if (failure_) {
        Error e = std::move(*failure_);
        failure_.reset();
        if (mode_ == Mode::Running) {
            begin_close(End::Faulted, std::unexpected(std::move(e)), c.now);
        } else if (mode_ == Mode::Closing) {
            why_ = End::Faulted;
            if (result_ || !latches(result_.error().kind)) result_ = std::unexpected(std::move(e));
        } else if (mode_ == Mode::Idle) {
            fault_ = std::move(e);
            mode_ = Mode::Fault;
        }
    }
    if (std::exchange(stop_requested_, false)) {
        const Error stopped{Error::Kind::Stopped, "stop requested", {}};
        if (mode_ == Mode::Running) {
            begin_close(End::Stopped, std::unexpected(stopped), c.now);
        } else if (mode_ == Mode::Closing) {
            if (why_ != End::Faulted) why_ = End::Stopped;
            if (result_ || !latches(result_.error().kind)) result_ = std::unexpected(stopped);
        } else {
            out.send(command::Stop{});
        }
    }
    if (const auto ticket = std::exchange(cancel_requested_, std::nullopt);
        ticket && mode_ == Mode::Running && *ticket == ticket_)
        begin_close(End::Cancelled, std::unexpected(Error{Error::Kind::Cancelled, "cancelled", {}}), c.now);

    // The open activity.
    if (mode_ == Mode::Running && activity_) {
        AnyActivity& a = *activity_;
        const Conditions missing = a.needs().while_running.missing_from(c.conditions);
        if (!missing.empty()) {
            begin_close(End::Faulted,
                        std::unexpected(Error{Error::Kind::ConditionLost,
                                              std::format("{} lost: {}", a.name(), to_string(missing)), missing}),
                        c.now);
        } else if (auto r = a.step(c, out); !r) {
            const End why = latches(r.error().kind) ? End::Faulted : End::Cancelled;
            begin_close(why, std::unexpected(std::move(r.error())), c.now);
        } else if (*r == Phase::Done) {
            begin_close(End::Done, {}, c.now);
        }
    }
    if (mode_ == Mode::Closing && activity_) {
        AnyActivity& a = *activity_;
        if (a.close(c, out, why_)) {
            finish(std::move(result_));
        } else if (c.now - close_started_ > options_.close_timeout) {
            out.send(command::Stop{});  // the last resort
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(options_.close_timeout).count();
            const std::string late = std::format("{} did not close within {} ms", a.name(), ms);
            if (!result_ && latches(result_.error().kind)) {
                result_.error().message += "; " + late;
                finish(std::move(result_));
            } else {
                finish(std::unexpected(Error{Error::Kind::Timeout, late, {}}));
            }
        }
    }
}

std::vector<Finished> Executor::take_finished() { return std::exchange(finished_, {}); }

std::optional<Ticket> Executor::current() const noexcept {
    if (activity_) return ticket_;
    return std::nullopt;
}

std::string_view Executor::current_name() const noexcept { return activity_ ? activity_->name() : std::string_view{}; }

bool Executor::current_needs_every_cycle() const { return activity_ && activity_->needs().every_cycle; }

void Executor::begin_close(End why, std::expected<void, Error> result, Clock::time_point now) {
    why_ = why;
    result_ = std::move(result);
    close_started_ = now;
    mode_ = Mode::Closing;
}

void Executor::finish(std::expected<void, Error> result) {
    if (!result && latches(result.error().kind)) {
        fault_ = result.error();
        mode_ = Mode::Fault;
    } else {
        mode_ = Mode::Idle;
    }
    finished_.push_back(Finished{ticket_, std::move(result)});
    activity_.reset();
}

}  // namespace ec::motion
