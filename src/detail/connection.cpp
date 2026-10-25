#include "detail/connection.hpp"

#include <algorithm>
#include <format>
#include <thread>

#include "ec/sdk/movement.hpp"
#include "ec/sdk/param.hpp"
#include "ec/status.hpp"

namespace ec::detail {

std::expected<sdk::Json, Error> Connection::call(const sdk::Request& request) {
    const std::lock_guard lock(calls);
    const auto sent = Clock::now();
    auto result = client.call(request);
    std::expected<sdk::Json, Error> reply = result ? std::expected<sdk::Json, Error>(std::move(*result))
                                                   : std::unexpected(from_call(request.method, result.error()));
    if (on_call) {
        try {
            on_call(CallRecord{.method = std::string(request.method),
                               .params = request.params,
                               .reply = reply,
                               .sent = sent,
                               .answered = Clock::now()});
        } catch (...) {  // NOLINT(bugprone-empty-catch): a log must not break the call it records
        }
    }
    return reply;
}

std::expected<sdk::Json, Error> Connection::call_checked(const sdk::Request& request, sdk::Kind kind) {
    if (!sdk::is_read_only(kind) && motion_open())
        return std::unexpected(Error{
            Error::Kind::Busy, std::format("{}: a motion is open: only queries and computations", request.method)});
    return call(request);
}

std::expected<std::uint64_t, Error> Connection::open(std::function<void()> stop) {
    const std::lock_guard lock(motions);
    if (open_motion != 0) return std::unexpected(Error{Error::Kind::Busy, "another motion is open"});
    open_motion = next_motion++;
    stop_open = std::move(stop);
    stop_requested.store(false);
    return open_motion;
}

void Connection::close(std::uint64_t id) noexcept {
    const std::lock_guard lock(motions);
    if (open_motion != id) return;
    open_motion = 0;
    stop_open = nullptr;
}

bool Connection::motion_open() const {
    const std::lock_guard lock(motions);
    return open_motion != 0;
}

void Connection::stop() noexcept {
    try {
        stop_requested.store(true);
        std::function<void()> stop_motion;
        {
            const std::lock_guard lock(motions);
            stop_motion = stop_open;
        }
        if (stop_motion) stop_motion();
        const auto stopped = send(sdk::Stop{});
        static_cast<void>(stopped);  // nothing more to do if it fails
    } catch (...) {                  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

namespace {

// What a wait needs to know, from one frame or one getRobotState.
struct Observed {
    RobotState state = RobotState::Stop;
    bool emergency_stop = false;
    bool collision = false;
    bool streamed = false;
};

}  // namespace

std::optional<std::expected<void, Error>> Connection::wait_stopped(Clock::time_point started, bool& ran,
                                                                   std::chrono::nanoseconds timeout,
                                                                   bool stop_ends_wait) {
    using namespace std::chrono_literals;
    using Result = std::expected<void, Error>;
    const auto fault = [](const char* what) { return Result(std::unexpected(Error{Error::Kind::Fault, what})); };
    const auto now = Clock::now();
    const auto deadline = timeout >= Clock::time_point::max() - now ? Clock::time_point::max() : now + timeout;
    for (;;) {
        if (stop_ends_wait && stop_requested.load()) return Result(std::unexpected(Error{Error::Kind::Stopped, ""}));
        Observed seen;
        std::uint64_t seq = 0;
        if (const auto s = monitor->latest(); s && s->age() < 100ms) {
            seen = {.state = s->motion.state,
                    .emergency_stop = s->safety.emergency_stop,
                    .collision = s->safety.collision,
                    .streamed = true};
            seq = s->seq;
        } else {
            const auto state = send(sdk::GetRobotState{});
            if (!state) return Result(std::unexpected(state.error()));
            seen.state = *state;
        }
        if (seen.emergency_stop) return fault("emergency stop");
        if (seen.collision) return fault("collision");
        switch (seen.state) {
            case RobotState::Running: ran = true; break;
            case RobotState::Stop:
                if (ran || Clock::now() - started > 1s) return Result{};
                break;
            case RobotState::EmergencyStop: return fault("emergency stop");
            case RobotState::Alarm: return fault("alarm");
            case RobotState::Collision: return fault("collision");
            default: break;  // paused: wait
        }
        const auto left = deadline - Clock::now();
        if (left <= Clock::duration::zero()) return std::nullopt;
        if (seen.streamed) {
            const auto next = monitor->next(seq, std::min<Clock::duration>(left, 50ms));  // the next frame
            static_cast<void>(next);  // read again above, as the newest
        } else {
            std::this_thread::sleep_for(10ms);
        }
    }
}

}  // namespace ec::detail
