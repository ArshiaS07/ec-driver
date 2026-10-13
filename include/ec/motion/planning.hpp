#pragma once
/// @file
/// Turning a Cartesian path into a timed joint trajectory (docs/motion.md 5.11):
/// the path is solved by inverse kinematics every few millimetres, the
/// solutions are interpolated in joint space, and the path is timed by a
/// rest-to-rest speed profile. Pure, apart from the inverse kinematics the
/// caller supplies (the controller's, through Robot::inverse_kinematics).

#include <cmath>
#include <cstddef>
#include <expected>
#include <format>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "ec/motion/error.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/path.hpp"
#include "ec/pose.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// Rest-to-rest timing along a distance: the speed rises for `ramp` seconds as
/// a half cosine (so the acceleration starts and ends at zero), stays at the
/// cruise speed, and falls the same way. A distance too short to reach the
/// speed asked for lowers the cruise speed.
class SpeedProfile {
public:
    /// The profile, or why the numbers do not make one: `distance` [m] >= 0,
    /// `speed` [m/s] > 0, `ramp` [s] >= 0, all finite.
    [[nodiscard]] static std::expected<SpeedProfile, std::string> make(double distance, double speed, double ramp);

    [[nodiscard]] double duration() const noexcept;                 ///< [s]
    [[nodiscard]] double distance(double t) const noexcept;         ///< [m] covered at time t [s], within [0, total()]
    [[nodiscard]] double speed(double t) const noexcept;            ///< [m/s] at time t; 0 outside [0, duration()]
    [[nodiscard]] double total() const noexcept { return total_; }  ///< [m]
    [[nodiscard]] double cruise_speed() const noexcept { return cruise_; }  ///< [m/s]
    [[nodiscard]] double ramp() const noexcept { return ramp_; }            ///< [s]
    /// [m/s^2]; infinite without a ramp.
    [[nodiscard]] double peak_acceleration() const noexcept;

private:
    SpeedProfile(double total, double cruise, double ramp) : total_(total), cruise_(cruise), ramp_(ramp) {}
    [[nodiscard]] double ramped(double t) const noexcept;  // distance covered t seconds into a ramp

    double total_;
    double cruise_;
    double ramp_;
};

/// Joint positions interpolated over an increasing parameter by cubic Hermite
/// segments, the slope at each knot taken from the parabola through it and its
/// neighbours (Bessel's method): positions and slopes are continuous, and
/// quadratics are reproduced exactly. Outside the knots the end positions hold.
class JointSpline {
public:
    /// The spline, or why not: at least 2 knots, strictly increasing and
    /// finite, with one finite position per knot.
    [[nodiscard]] static std::expected<JointSpline, std::string> make(std::vector<double> knots,
                                                                      std::vector<Joints> positions);

    [[nodiscard]] Joints position(double x) const noexcept;
    /// dq/dx; zero outside the knots.
    [[nodiscard]] Joints slope(double x) const noexcept;
    [[nodiscard]] const std::vector<double>& knots() const noexcept { return x_; }
    [[nodiscard]] const std::vector<Joints>& positions() const noexcept { return q_; }

private:
    JointSpline(std::vector<double> x, std::vector<Joints> q, std::vector<Joints> m)
        : x_(std::move(x)), q_(std::move(q)), m_(std::move(m)) {}
    [[nodiscard]] std::size_t segment(double x) const noexcept;  // k with x_k <= x < x_k+1, for x inside

    std::vector<double> x_;
    std::vector<Joints> q_;
    std::vector<Joints> m_;  // the slope at each knot
};

/// The inverse kinematics the planner calls: the joint positions [rad] that put
/// the TCP at `pose`, chosen near `reference`. Robot::inverse_kinematics is one.
using InverseKinematics = std::function<std::expected<Joints, Error>(const Pose& pose, const Joints& reference)>;

/// How finely a path is solved.
struct PlanOptions {
    double spacing = 0.003;  ///< [m] of path between solved points; both ends are always solved
    /// [rad]: neighbouring solutions further apart than this on any joint are
    /// refused: the arm would change configuration, or pass a singularity.
    double max_joint_step = deg_to_rad(10.0);
    std::size_t max_points = 5'000;  ///< a path that needs more points is refused
};

/// A path solved into joint positions: the spline's parameter is the distance
/// along the path [m], from 0 to its length.
struct JointPath {
    JointSpline joints;
    std::vector<Pose> poses{};  ///< the pose solved at each knot

    [[nodiscard]] double length() const noexcept { return joints.knots().back(); }
};

/// Solves `poses`, at distances `s` [m] along a path (increasing from 0), by
/// `ik`: the first near `start`, each next one near the one before (a solution
/// given within one turn is taken to the nearest turn). Fails with
/// Kind::Invalid if a pose has no solution or neighbouring solutions are more
/// than options.max_joint_step apart on a joint.
[[nodiscard]] std::expected<JointPath, Error> solve(std::vector<double> s, std::vector<Pose> poses, const Joints& start,
                                                    const InverseKinematics& ik, const PlanOptions& options = {});

/// Solves `path` at every options.spacing of its length and at its end (see
/// above). One inverse-kinematics call per point.
template <CartesianPath P>
[[nodiscard]] std::expected<JointPath, Error> solve(const P& path, const Joints& start, const InverseKinematics& ik,
                                                    const PlanOptions& options = {}) {
    const double total = length(path);
    if (!(std::isfinite(total) && total > 0.0))
        return std::unexpected(Error{Error::Kind::Invalid, std::format("the path's length is {} m", total), {}});
    if (!(options.spacing > 0.0))
        return std::unexpected(Error{Error::Kind::Invalid, "the spacing must be positive", {}});
    const double steps = std::ceil(total / options.spacing);
    if (steps + 1.0 > static_cast<double>(options.max_points))
        return std::unexpected(Error{Error::Kind::Invalid,
                                     std::format("the path needs {} points at {} mm spacing, more than {}", steps + 1.0,
                                                 options.spacing * 1000.0, options.max_points),
                                     {}});
    const auto n = static_cast<std::size_t>(steps);
    std::vector<double> s(n + 1);
    std::vector<Pose> poses(n + 1);
    for (std::size_t k = 0; k <= n; ++k) {
        s[k] = k == n ? total : total * static_cast<double>(k) / static_cast<double>(n);
        poses[k] = pose_at(path, s[k]);
    }
    return solve(std::move(s), std::move(poses), start, ik, options);
}

/// A solved path with its timing: positions and velocities at any time.
class TimedPath {
public:
    /// Times `path` with a SpeedProfile over its length; fails (Kind::Invalid)
    /// on a speed or ramp SpeedProfile::make() refuses.
    [[nodiscard]] static std::expected<TimedPath, Error> make(JointPath path, double speed, double ramp);

    [[nodiscard]] double duration() const noexcept { return profile_.duration(); }
    [[nodiscard]] Joints position(double t) const noexcept;  ///< [rad] at time t [s]
    [[nodiscard]] Joints velocity(double t) const noexcept;  ///< [rad/s] at time t [s]
    [[nodiscard]] const JointPath& path() const noexcept { return path_; }
    [[nodiscard]] const SpeedProfile& profile() const noexcept { return profile_; }

    /// Positions every `period` [s] from 0, ending exactly at the duration (the
    /// last interval between half a period and one and a half): a trajectory to
    /// play (PlayTrajectory) or follow (FollowTrajectory).
    [[nodiscard]] JointTrajectory sample(double period) const;

private:
    TimedPath(JointPath path, SpeedProfile profile) : path_(std::move(path)), profile_(profile) {}

    JointPath path_;
    SpeedProfile profile_;
};

}  // namespace ec::motion
