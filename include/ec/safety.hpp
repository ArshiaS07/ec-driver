#pragma once
/// @file
/// ec::Safety: what the controller's safety system reports, and the collision
/// reset (SDK manual 2.2.2.21-54; docs/ec66.md §5.4, §5.8). Each operation is
/// one call on the command connection (8055). The emergency stop and the
/// collision flag are also on the monitoring path:
/// robot.monitor().latest()->safety. Changing the limits needs the remote
/// password (setSafetyParams) and stays a typed call; the safety functions
/// themselves are the controller's, not the driver's.

#include <expected>
#include <memory>

#include "ec/error.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A limit of the safety system, in normal and in reduced mode.
struct SafetyLimit {
    double normal = 0.0;
    double reduced = 0.0;
};

/// A handle onto a robot's safety system: cheap to copy, usable from any
/// thread. Once its Robot is gone, every operation fails with Disconnected.
class Safety {
public:
    /// get_estop_status [S 2.2.2.61]: an emergency stop is pressed. (The
    /// state's emergency-stop value is short-lived; this is not.)
    [[nodiscard]] std::expected<bool, Error> emergency_stop() const;
    /// getCollisionState [S 2.2.2.21]: a collision was detected.
    [[nodiscard]] std::expected<bool, Error> collision() const;
    /// resetCollisionState [S 2.2.2.35]; then clear the alarm (Drives::clear_alarm).
    /// Busy while a motion is open.
    [[nodiscard]] std::expected<void, Error> reset_collision() const;
    /// get_collision_enable_status [S 2.2.2.53]: collision detection is on.
    [[nodiscard]] std::expected<bool, Error> collision_detection() const;
    /// getCollisionSensitivity [S 2.2.2.54] as a fraction: lower triggers at a
    /// smaller force.
    [[nodiscard]] std::expected<double, Error> collision_sensitivity() const;

    /// getRobotSafetyParamsEnabled [S 2.2.2.41]: the limits below are enforced.
    [[nodiscard]] std::expected<bool, Error> limits_enabled() const;
    [[nodiscard]] std::expected<SafetyLimit, Error> power_limit() const;        ///< getRobotSafetyPower [W]
    [[nodiscard]] std::expected<SafetyLimit, Error> momentum_limit() const;     ///< getRobotSafetyMomentum [kg m/s]
    [[nodiscard]] std::expected<SafetyLimit, Error> tool_force_limit() const;   ///< getRobotSafetyToolForce [N]
    [[nodiscard]] std::expected<SafetyLimit, Error> elbow_force_limit() const;  ///< getRobotSafetyElbowForce [N]
    /// getRobotSpeedPercentage [S 2.2.2.48], as fractions.
    [[nodiscard]] std::expected<SafetyLimit, Error> speed_limit() const;

private:
    friend class Robot;
    explicit Safety(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec
