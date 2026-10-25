#pragma once
/// @file
/// ec::Joint: one of the arm's six joints, read on the command connection
/// (8055): a stateful mirror, not something to command (every motion is the
/// motion controller's; docs/ec66.md §3.2). Each reading is one call that
/// answers all six joints, of which this one is taken; the same values come
/// free on the monitoring path, as of their frame:
/// robot.monitor().latest()->joints[i - 1].
///
///     double t3 = robot.joint(3).temperature().value();   // asked now [degC]

#include <expected>
#include <memory>
#include <string>

#include "ec/error.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A handle onto one joint, 1..6 (another number makes every reading
/// Invalid): cheap to copy, usable from any thread. Once its Robot is gone,
/// every reading fails with Disconnected. The motor-side position
/// (get_motor_pos) is left out: the manual gives no unit (docs/ec66.md §8).
class Joint {
public:
    [[nodiscard]] int number() const noexcept { return number_; }  ///< 1..6

    /// get_joint_pos [S 2.2.2.3]: on the output side, after the reducer [rad].
    [[nodiscard]] std::expected<double, Error> position() const;
    /// get_joint_speed [S 2.2.2.57] [rad/s].
    [[nodiscard]] std::expected<double, Error> velocity() const;
    /// get_joint_torques [S 2.2.2.46]: the output torque [N m].
    [[nodiscard]] std::expected<double, Error> torque() const;
    /// get_motor_torque [S 2.2.2.14]: the fraction of the motor's rated torque [1].
    [[nodiscard]] std::expected<double, Error> torque_of_rated() const;
    /// get_joint_temp [S 2.2.2.72] [degC].
    [[nodiscard]] std::expected<double, Error> temperature() const;
    /// get_servo_brake_off_status [S 2.2.11.1]: this joint's brake is released
    /// (a Command error if the controller sends fewer entries; shape unverified).
    [[nodiscard]] std::expected<bool, Error> brake_released() const;
    /// getJointVersion [S 2.2.8.2]: the joint's servo drive version, as text
    /// (servo version 11 or later).
    [[nodiscard]] std::expected<std::string, Error> firmware() const;

private:
    friend class Robot;
    Joint(std::weak_ptr<detail::Connection> session, int number) noexcept
        : session_(std::move(session)), number_(number) {}
    std::weak_ptr<detail::Connection> session_;
    int number_ = 1;
};

}  // namespace ec
