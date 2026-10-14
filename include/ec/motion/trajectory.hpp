#pragma once
/// @file
/// Joint trajectories: positions at increasing times, the limits they are
/// checked against, and their peak rates. Pure. The timestamp service plays
/// them (ec/motion/timestamped.hpp).

#include <cstddef>
#include <expected>
#include <string>
#include <vector>

#include "ec/units.hpp"

namespace ec::motion {

/// Joint positions at increasing times: a path with its timing, in SI.
struct JointTrajectory {
    std::vector<double> time{};      ///< [s]; the first is 0, then strictly increasing
    std::vector<Joints> position{};  ///< [rad]; one per time

    [[nodiscard]] std::size_t size() const noexcept { return time.size(); }
    [[nodiscard]] double duration() const noexcept { return time.empty() ? 0.0 : time.back(); }
};

/// The bounds a trajectory is checked against before it runs.
struct TrajectoryLimits {
    Joints max_velocity = kEc66MaxJointSpeed;  ///< [rad/s], between neighbouring points
    /// [rad/s^2], peak_acceleration(): the datasheet speeds reached in 0.5 s. The
    /// datasheet gives no accelerations; to tune on the robot (docs/motion.md 7.2).
    Joints max_acceleration = scale(kEc66MaxJointSpeed, 2.0);
    double joint_limit = kEc66JointLimit;  ///< [rad], on |q|
};

/// Why `t` cannot run, or nothing: it needs at least 3 points, as many times as
/// positions, times from 0 strictly increasing, finite positions within the
/// joint limit, no segment faster than the speed limit on any joint, and no
/// change of speed (from rest at the start, to rest at the end) above the
/// acceleration limit.
[[nodiscard]] std::expected<void, std::string> validate(const JointTrajectory& t, const TrajectoryLimits& limits = {});

/// Each joint's largest speed [rad/s] between neighbouring points of a valid
/// trajectory.
[[nodiscard]] Joints peak_velocity(const JointTrajectory& t) noexcept;

/// Each joint's largest acceleration [rad/s^2] of a valid trajectory, from the
/// change of speed between neighbouring segments, and from rest into the first
/// segment and out of the last (a trajectory is run from rest to rest).
[[nodiscard]] Joints peak_acceleration(const JointTrajectory& t) noexcept;

/// The position at time `s` [s] of a valid trajectory, interpolated linearly
/// between its points; the first or last point outside its times.
[[nodiscard]] Joints position_at(const JointTrajectory& t, double s) noexcept;

}  // namespace ec::motion
