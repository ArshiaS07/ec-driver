#pragma once
/// @file
/// Planned paths in joint space (docs/motion.md 5.11, ADR 0017): timed TCP
/// poses are solved by inverse kinematics into joint positions, checked, and
/// sampled into a JointTrajectory, which the controller's timestamp service
/// plays (PlayTrajectory). Pure, apart from the inverse kinematics the caller
/// supplies (the controller's, through Robot::ik). Poses follow ec/pose.hpp.

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <expected>
#include <functional>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "ec/motion/error.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/path.hpp"
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

    [[nodiscard]] double duration() const noexcept;          ///< [s]
    [[nodiscard]] double distance(double t) const noexcept;  ///< [m] covered at time t [s], within [0, total()]
    [[nodiscard]] double speed(double t) const noexcept;     ///< [m/s] at time t; 0 outside [0, duration()]
    /// [s] when `distance` [m] is reached: the inverse of distance(t), clamped
    /// to [0, duration()].
    [[nodiscard]] double time_at(double distance) const noexcept;
    [[nodiscard]] double total() const noexcept { return total_; }          ///< [m]
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

/// A TCP pose at a time: one point of a planned path.
struct TimedPose {
    double time = 0.0;                                       ///< [s] from the path's start
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  ///< T_base_tcp
};

/// The poses of `path` timed by `profile` (made over the path's length): a pose
/// every `step` [m] of path, and more often where the path is slow, so that no
/// two are further apart in time than `step` takes at the cruise speed; and
/// one at the end. Dense enough for plan() to interpolate between them in time.
template <CartesianPath P>
[[nodiscard]] std::vector<TimedPose> sample(const P& path, const SpeedProfile& profile, double step) {
    const double end = profile.duration();
    const double longest = profile.cruise_speed() > 0.0 ? step / profile.cruise_speed() : end;
    std::vector<TimedPose> out{{.time = 0.0, .pose = pose_at(path, 0.0)}};
    for (double t = 0.0;;) {
        t = std::min(t + longest, profile.time_at(profile.distance(t) + step));
        if (!(t < end - 1e-9)) break;
        out.push_back({.time = t, .pose = pose_at(path, profile.distance(t))});
    }
    if (end > 0.0) out.push_back({.time = end, .pose = pose_at(path, profile.total())});
    return out;
}

/// The inverse kinematics plan() calls: the joint positions [rad] that put the
/// TCP at T_base_tcp, chosen near `near`. Robot::ik is one.
using InverseKinematics =
    std::function<std::expected<Joints, Error>(const Eigen::Isometry3d& T_base_tcp, const Joints& near)>;

/// How plan() solves and checks a path.
struct PlanOptions {
    /// [m]: neighbouring points further apart are joined by a straight line,
    /// solved every max_step along it.
    double max_step = 0.003;
    double max_turn = deg_to_rad(1.0);  ///< [rad]: the same for the orientation, turned evenly (slerp)
    /// [rad]: neighbouring solutions further apart than this on a joint are
    /// refused: the arm would change configuration, or pass close to a singularity.
    double max_joint_step = deg_to_rad(10.0);
    /// [rad]: how far J5 (the wrist) and J3 (the elbow) must stay from 0 and 180
    /// deg, where the EC series' arm is singular (its layout is Universal Robots').
    double singularity_margin = deg_to_rad(5.0);
    double period = 0.02;             ///< [s] between the trajectory's points
    TrajectoryLimits limits{};        ///< joint positions, speeds, accelerations
    std::size_t max_points = 10'000;  ///< solved points at most
};

/// Turns a timed TCP path into a joint trajectory. The path's times start at 0
/// and increase; its first and last points should be at rest (the trajectory
/// runs from rest to rest). Steps: points further apart than max_step or
/// max_turn are joined by straight lines and filled in; every point is solved by
/// `ik`, the first near `start`, each next one near the one before (a solution
/// given within one turn taken to the nearest turn); the solutions are
/// interpolated over time (JointSpline) and sampled every `period`, ending
/// exactly at the last time. Fails with Kind::Invalid, naming the point, if a
/// pose has no solution, if neighbouring solutions differ by more than
/// max_joint_step, if J5 or J3 comes within singularity_margin of 0 or 180 deg,
/// or if the trajectory breaks the limits; other kinds (Disconnected, ...) pass
/// through from `ik`. Every point costs one call of `ik`.
[[nodiscard]] std::expected<JointTrajectory, Error> plan(std::span<const TimedPose> path, const Joints& start,
                                                         const InverseKinematics& ik, const PlanOptions& options = {});

/// How close joint positions come to the EC series' wrist and elbow
/// singularities: the least distance of J5, and of J3, from 0 or 180 deg [rad].
struct SingularityMargins {
    double wrist = std::numbers::pi / 2.0;
    double elbow = std::numbers::pi / 2.0;
};
[[nodiscard]] SingularityMargins singularity_margins(std::span<const Joints> positions) noexcept;

}  // namespace ec::motion
