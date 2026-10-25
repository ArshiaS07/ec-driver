#pragma once
/// @file
/// ec::Drives: the robot's drives, as the controller exposes them: servo
/// enable and brakes, the encoders' synchronisation and precision, alarms,
/// backdrive (SDK manual 2.2.1, 2.2.2.64-65, 2.2.11; docs/ec66.md §3.3-§3.4,
/// §5.2). Each operation is one call on the command connection (8055),
/// answered before it returns; the controller does the work and checks every
/// request itself (the actions need REMOTE mode). The servo and encoder flags
/// are also on the monitoring path, free to read: robot.monitor().latest()->drives.
///
///     auto drives = robot.drives();
///     if (!drives.servo_on().value()) drives.switch_servo(true);

#include <expected>
#include <memory>
#include <vector>

#include "ec/error.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A handle onto a robot's drives: cheap to copy, usable from any thread
/// (calls wait for each other). Once its Robot is gone, every operation fails
/// with Disconnected. While a motion is open the actions are Busy; the
/// readings still work.
class Drives {
public:
    /// getServoStatus [S 2.2.1.1]: the servos are on, the brakes released.
    [[nodiscard]] std::expected<bool, Error> servo_on() const;
    /// set_servo_status [S 2.2.1.2]. On releases the brakes (the drive then
    /// checks them with a small motion, inferred: docs/ec66.md §3.3); off
    /// engages them. The reply means accepted: the servos are on once
    /// servo_on() or the monitor says so (the manual waits 1 s).
    [[nodiscard]] std::expected<void, Error> switch_servo(bool on) const;

    /// getMotorStatus [S 2.2.1.5]: the encoders are synchronised.
    [[nodiscard]] std::expected<bool, Error> synced() const;
    /// syncMotorStatus [S 2.2.1.3]: needed after power-up and after an
    /// emergency stop, before any motion; the manual waits 0.5 s after it.
    [[nodiscard]] std::expected<void, Error> synchronise() const;
    /// get_servo_precise_position_status [S 2.2.2.64]: the encoders are
    /// precise (docs/ec66.md §3.4). Not on the monitoring path.
    [[nodiscard]] std::expected<bool, Error> precise() const;

    /// get_servo_alarm_state [S 2.2.2.65]: a servo drive is in alarm.
    [[nodiscard]] std::expected<bool, Error> servo_alarm() const;
    /// clearAlarm [S 2.2.1.4]; the manual waits 0.5 s after it.
    [[nodiscard]] std::expected<void, Error> clear_alarm() const;
    /// reset_robot_status [S 2.2.11.5]: only when stopped, or in an error
    /// other than an emergency stop.
    [[nodiscard]] std::expected<void, Error> reset() const;

    /// get_servo_brake_off_status [S 2.2.11.1]: for each joint, whether its
    /// brake is released, as many as the controller sends (unverified on the EA66).
    [[nodiscard]] std::expected<std::vector<bool>, Error> brakes_released() const;
    /// get_backdrive_status [S 2.2.11.2]: in backdrive mode, where each joint's
    /// brake is released while it is pushed by hand (docs/ec66.md §3.3).
    [[nodiscard]] std::expected<bool, Error> backdrive() const;
    /// enter_backdrive [S 2.2.11.3]: only from the reset state.
    [[nodiscard]] std::expected<void, Error> enter_backdrive() const;
    /// exit_backdrive [S 2.2.11.4].
    [[nodiscard]] std::expected<void, Error> exit_backdrive() const;

private:
    friend class Robot;
    explicit Drives(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec
