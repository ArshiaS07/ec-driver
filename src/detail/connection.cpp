#include "detail/connection.hpp"

#include <thread>

#include "ec/command/param.hpp"
#include "ec/status.hpp"

namespace ec::detail {

std::expected<command::Json, Error> Connection::call(const command::Request& request) {
    const std::lock_guard lock(calls);
    auto result = client.call(request);
    if (!result) return std::unexpected(from_command(request.method, result.error()));
    return std::move(*result);
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

std::optional<std::expected<void, Error>> Connection::wait_stopped(Clock::time_point started, bool& ran,
                                                                   std::chrono::nanoseconds timeout) {
    using namespace std::chrono_literals;
    const auto deadline = Clock::now() + timeout;
    for (;;) {
        if (stop_requested.load()) return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Stopped, ""}));
        const auto state = send(command::GetRobotState{});
        if (!state) return std::expected<void, Error>(std::unexpected(state.error()));
        switch (*state) {
            case RobotState::Running: ran = true; break;
            case RobotState::Stop:
                if (ran || Clock::now() - started > 1s) return std::expected<void, Error>{};
                break;
            case RobotState::EmergencyStop:
                return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Fault, "emergency stop"}));
            case RobotState::Alarm:
                return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Fault, "alarm"}));
            case RobotState::Collision:
                return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Fault, "collision"}));
            default: break;  // paused: wait
        }
        if (Clock::now() >= deadline) return std::nullopt;
        std::this_thread::sleep_for(10ms);
    }
}

}  // namespace ec::detail
