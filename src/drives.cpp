#include "ec/drives.hpp"

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/sdk/backdrive.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/servo.hpp"

namespace ec {
using detail::done;
using detail::on;

std::expected<bool, Error> Drives::servo_on() const { return detail::send(session_, sdk::GetServoStatus{}); }

std::expected<void, Error> Drives::switch_servo(bool on) const {
    return done(detail::send(session_, sdk::SetServoStatus{.on = on}));
}

std::expected<bool, Error> Drives::synced() const { return detail::send(session_, sdk::GetMotorStatus{}); }

std::expected<void, Error> Drives::synchronise() const { return done(detail::send(session_, sdk::SyncMotorStatus{})); }

std::expected<bool, Error> Drives::precise() const {
    return on(detail::send(session_, sdk::GetServoPrecisePositionStatus{}));
}

std::expected<bool, Error> Drives::servo_alarm() const { return on(detail::send(session_, sdk::GetServoAlarmState{})); }

std::expected<void, Error> Drives::clear_alarm() const { return done(detail::send(session_, sdk::ClearAlarm{})); }

std::expected<void, Error> Drives::reset() const { return done(detail::send(session_, sdk::ResetRobotStatus{})); }

std::expected<std::vector<bool>, Error> Drives::brakes_released() const {
    const auto states = detail::send(session_, sdk::GetServoBrakeOffStatus{});
    if (!states) return std::unexpected(states.error());
    std::vector<bool> released;
    released.reserve(states->size());
    for (const int s : *states) released.push_back(s != 0);
    return released;
}

std::expected<bool, Error> Drives::backdrive() const { return on(detail::send(session_, sdk::GetBackdriveStatus{})); }

std::expected<void, Error> Drives::enter_backdrive() const {
    return done(detail::send(session_, sdk::EnterBackdrive{}));
}

std::expected<void, Error> Drives::exit_backdrive() const { return done(detail::send(session_, sdk::ExitBackdrive{})); }

}  // namespace ec
