#include "ec/motion/controller.hpp"

#include <format>

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/sdk/param.hpp"

namespace ec::motion {

std::expected<RobotState, Error> Controller::state() const { return detail::send(session_, sdk::GetRobotState{}); }

std::optional<std::expected<void, Error>> Controller::wait_until_stopped(std::chrono::nanoseconds timeout) const {
    const auto c = session_.lock();
    if (!c) return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Disconnected, "the robot is gone"}));
    bool at_rest_counts = true;  // nothing was just started: a stop is a stop
    return c->wait_stopped(Clock::now(), at_rest_counts, timeout, /*stop_ends_wait=*/false);
}

void Controller::stop() const noexcept {
    if (const auto c = session_.lock()) c->stop();
}

std::expected<double, Error> Controller::speed_scale() const {
    const auto pct = detail::send(session_, sdk::GetSpeed{});
    if (!pct) return std::unexpected(pct.error());
    return *pct / 100.0;
}

std::expected<void, Error> Controller::set_speed_scale(double fraction) const {
    if (!(fraction >= 0.0005 && fraction <= 1.0))
        return std::unexpected(
            Error{Error::Kind::Invalid, std::format("setSpeed: {} is outside [0.0005, 1] (0.05..100 %)", fraction)});
    return detail::done(detail::send(session_, sdk::SetSpeed{.value_pct = fraction * 100.0}));
}

std::expected<bool, Error> Controller::hand_guiding() const { return detail::send(session_, sdk::GetDragState{}); }

std::expected<void, Error> Controller::set_hand_guiding(bool on) const {
    return detail::done(detail::send(session_, sdk::DragTeachSwitch{.on = on}));
}

}  // namespace ec::motion
