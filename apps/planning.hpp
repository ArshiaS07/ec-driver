#pragma once
// Planning for the sandbox (application code, not part of the driver): a motion
// of the TCP, poses stamped with their time, solved by the controller's inverse
// kinematics into a checked joint trajectory, which ec::motion::Timestamped
// runs (its reasoning: docs/archive/motion-2026-10-18.md §5.11). Temporary, until the application's own planning
// replaces it.
//
// plan() fills in the poses (3 mm, 1 deg apart), solves each near the one
// before, checks every solution (no jump, no singularity, within the limits),
// interpolates the joints over time (at rest at the ends and at pauses) and
// samples them every 20 ms; the trajectory is checked against the joints'
// speed and acceleration limits.

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <expected>
#include <format>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include "ec/error.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/kinematics.hpp"
#include "ec/sdk/param.hpp"
#include "ec/stamped.hpp"
#include "ec/trajectory.hpp"
#include "ec/units.hpp"

namespace planning {

using ec::Clock;
using ec::Error;
using ec::Joints;
using ec::kJointCount;

/// A motion of the TCP: poses T_base_tcp, each stamped with its time since the
/// start; the first at 0, then increasing. Between two poses the TCP goes in a
/// straight line at constant speed, its orientation turning evenly
/// (interpolate()). The same pose at two neighbouring stamps is a pause: the arm
/// rests there.
using PoseTrajectory = std::vector<ec::Stamped<Eigen::Isometry3d, Clock::duration>>;

/// How plan() solves and checks a motion.
struct PlanOptions {
    /// [m]: neighbouring poses further apart are joined by a straight line,
    /// solved every max_step along it.
    double max_step = 0.003;
    double max_turn = ec::deg_to_rad(1.0);  ///< [rad]: the same for the orientation, turned evenly (slerp)
    /// [rad]: neighbouring solutions further apart than this on a joint are
    /// refused: the arm would change configuration, or pass close to a singularity.
    double max_joint_step = ec::deg_to_rad(10.0);
    /// [rad]: how far J5 (the wrist) and J3 (the elbow) must stay from 0 and 180
    /// deg, where the EC series' arm is singular (its layout is Universal Robots').
    double singularity_margin = ec::deg_to_rad(5.0);
    double period = 0.02;                          ///< [s] between the trajectory's points
    Joints max_velocity = ec::kEc66MaxJointSpeed;  ///< [rad/s], between neighbouring points
    /// [rad/s^2]: the datasheet speeds reached in 0.5 s. The datasheet gives no
    /// accelerations; to tune on the robot.
    Joints max_acceleration = ec::scale(ec::kEc66MaxJointSpeed, 2.0);
    double joint_limit = ec::kEc66JointLimit;  ///< [rad], on |q|
    std::size_t max_points = 10'000;           ///< solved points at most
};

namespace detail {

inline double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

inline Clock::duration stamp(double s) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}

inline std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message)});
}

// The angle [rad] of the rotation that takes orientation `a` to `b`, in [0, pi].
inline double angle_between(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    const Eigen::Quaterniond q(a.transpose() * b);  // from the quaternion: exact for small angles too
    return 2.0 * std::atan2(q.vec().norm(), std::abs(q.w()));
}

// The pose a fraction `u` in [0, 1] of the way from `a` to `b`: the position on
// the straight line, the orientation turned evenly (slerp, the shorter way round).
inline Eigen::Isometry3d interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double u) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = a.translation() + u * (b.translation() - a.translation());
    pose.linear() = Eigen::Quaterniond(a.linear()).slerp(u, Eigen::Quaterniond(b.linear())).toRotationMatrix();
    return pose;
}

inline bool is_rotation(const Eigen::Matrix3d& r) {
    return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-6 &&
           r.determinant() > 0.0;
}

// Why `motion` is not one, or nothing.
inline std::expected<void, std::string> validate(const PoseTrajectory& motion) {
    if (motion.size() < 2) return std::unexpected(std::format("{} poses; a motion needs at least 2", motion.size()));
    if (motion.front().stamp != Clock::duration::zero())
        return std::unexpected(std::format("it starts at {} s, not 0", seconds(motion.front().stamp)));
    for (std::size_t k = 0; k < motion.size(); ++k) {
        const auto& p = motion[k];
        if (k > 0 && !(p.stamp > motion[k - 1].stamp))
            return std::unexpected(std::format("pose {}: its stamp, {} s, does not increase", k, seconds(p.stamp)));
        if (!p.value.translation().allFinite() || !is_rotation(p.value.linear()))
            return std::unexpected(std::format("pose {}: not a rigid transform (a finite position and a rotation)", k));
    }
    return {};
}

// A pose of the motion, or one filled in between two, at its time [s].
struct Point {
    double time = 0.0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

inline std::string where(const Point& p) {
    const Eigen::Vector3d r = p.pose.translation() * 1000.0;
    return std::format("at {:.3f} s ({:.1f}, {:.1f}, {:.1f} mm)", p.time, r.x(), r.y(), r.z());
}

// Whether two poses are the same: within a nanometre and a nanoradian.
inline bool same_pose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return (a.translation() - b.translation()).norm() <= 1e-9 && angle_between(a.linear(), b.linear()) <= 1e-9;
}

// The motion with points inserted, along straight lines and even turns, so that
// no two neighbours are further apart than max_step or max_turn.
inline std::expected<std::vector<Point>, Error> fill_in(const PoseTrajectory& motion, const PlanOptions& o) {
    std::vector<Point> points{{.time = seconds(motion.front().stamp), .pose = motion.front().value}};
    for (std::size_t k = 1; k < motion.size(); ++k) {
        const Point a = points.back();
        const Point b{.time = seconds(motion[k].stamp), .pose = motion[k].value};
        const double gap = (b.pose.translation() - a.pose.translation()).norm();
        const double turn = angle_between(a.pose.linear(), b.pose.linear());
        const double pieces = std::max({1.0, std::ceil(gap / o.max_step), std::ceil(turn / o.max_turn)});
        if (static_cast<double>(points.size()) + pieces > static_cast<double>(o.max_points))
            return invalid(std::format("the motion needs more than {} points {} mm and {} deg apart", o.max_points,
                                       o.max_step * 1000.0, ec::rad_to_deg(o.max_turn)));
        const auto n = static_cast<std::size_t>(pieces);
        for (std::size_t i = 1; i < n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(n);
            points.push_back({.time = a.time + (b.time - a.time) * u, .pose = interpolate(a.pose, b.pose, u)});
        }
        points.push_back(b);
    }
    return points;
}

// How far a joint angle is from the nearest multiple of pi [rad].
inline double from_singular(double q) { return std::abs(std::remainder(q, std::numbers::pi)); }

// Why the arm should not be at `q`, or nothing: beyond the joint limit, or J5
// or J3 too close to 0 or 180 deg (the wrist and elbow singularities).
inline std::expected<void, std::string> check_point(const Joints& q, const PlanOptions& o) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(q[j]) <= o.joint_limit))
            return std::unexpected(
                std::format("joint {} at {:.1f} deg: beyond the joint limit", j + 1, ec::rad_to_deg(q[j])));
    if (from_singular(q[4]) < o.singularity_margin)
        return std::unexpected(
            std::format("J5 at {:.1f} deg: within {:.1f} deg of the wrist singularity (J5 at 0 or 180 deg)",
                        ec::rad_to_deg(q[4]), ec::rad_to_deg(o.singularity_margin)));
    if (from_singular(q[2]) < o.singularity_margin)
        return std::unexpected(
            std::format("J3 at {:.1f} deg: within {:.1f} deg of the elbow singularity (the arm stretched or folded, "
                        "J3 at 0 or 180 deg)",
                        ec::rad_to_deg(q[2]), ec::rad_to_deg(o.singularity_margin)));
    return {};
}

// Why the arm should not go from `from` to `to` in one step, or nothing: a
// joint turning more than max_joint_step, or what check_point(to) finds.
inline std::expected<void, std::string> check_step(const Joints& from, const Joints& to, const PlanOptions& o) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(to[j] - from[j]) <= o.max_joint_step))
            return std::unexpected(std::format(
                "joint {} turns {:.1f} deg in one step: the arm would change configuration, or pass close to a "
                "singularity",
                j + 1, ec::rad_to_deg(std::abs(to[j] - from[j]))));
    return check_point(to, o);
}

// Joint positions interpolated over time by cubic Hermite segments, the slope
// at each inner knot from the parabola through it and its neighbours (Bessel):
// positions and slopes are continuous. At rest at the first and last knot, and
// at both knots of a pause (the same positions twice); next to a knot at rest
// the slopes are kept within three times their segment's secant, so that no
// segment overshoots into or out of rest (Fritsch and Carlson).
class JointSpline {
public:
    // At least 2 knots, strictly increasing, one position per knot.
    JointSpline(std::vector<double> knots, std::vector<Joints> positions)
        : x_(std::move(knots)), q_(std::move(positions)), m_(x_.size()) {
        const std::size_t n = x_.size();
        std::vector<Joints> secant(n - 1);
        for (std::size_t k = 0; k + 1 < n; ++k)
            for (std::size_t j = 0; j < kJointCount; ++j)
                secant[k][j] = (q_[k + 1][j] - q_[k][j]) / (x_[k + 1] - x_[k]);
        for (std::size_t k = 1; k + 1 < n; ++k) {
            const double h0 = x_[k] - x_[k - 1];
            const double h1 = x_[k + 1] - x_[k];
            for (std::size_t j = 0; j < kJointCount; ++j)
                m_[k][j] = (h1 * secant[k - 1][j] + h0 * secant[k][j]) / (h0 + h1);
        }
        std::vector<bool> rest(n, false);
        rest.front() = rest.back() = true;
        for (std::size_t k = 0; k + 1 < n; ++k)
            if (q_[k] == q_[k + 1]) rest[k] = rest[k + 1] = true;
        const auto limit = [](double& slope, double chord) {
            slope = chord >= 0.0 ? std::clamp(slope, 0.0, 3.0 * chord) : std::clamp(slope, 3.0 * chord, 0.0);
        };
        for (std::size_t k = 0; k < n; ++k) {
            if (!rest[k]) continue;
            m_[k] = Joints{};
            for (std::size_t j = 0; j < kJointCount; ++j) {
                if (k > 0 && !rest[k - 1]) limit(m_[k - 1][j], secant[k - 1][j]);
                if (k + 1 < n && !rest[k + 1]) limit(m_[k + 1][j], secant[k][j]);
            }
        }
    }

    [[nodiscard]] Joints position(double x) const {
        if (!(x > x_.front())) return q_.front();
        if (x >= x_.back()) return q_.back();
        const auto next = static_cast<std::size_t>(std::upper_bound(x_.begin(), x_.end(), x) - x_.begin());
        const std::size_t k = next - 1;
        const double h = x_[k + 1] - x_[k];
        const double t = (x - x_[k]) / h;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        const double h10 = t3 - 2.0 * t2 + t;
        const double h01 = -2.0 * t3 + 3.0 * t2;
        const double h11 = t3 - t2;
        Joints q{};
        for (std::size_t j = 0; j < kJointCount; ++j)
            q[j] = h00 * q_[k][j] + h10 * h * m_[k][j] + h01 * q_[k + 1][j] + h11 * h * m_[k + 1][j];
        return q;
    }

private:
    std::vector<double> x_;
    std::vector<Joints> q_;
    std::vector<Joints> m_;  // the slope at each knot
};

// Why the sampled trajectory `t` (at least 3 points, times increasing) should
// not run, or nothing: a position beyond joint_limit, a segment faster than
// max_velocity, or a change of speed (from rest at the start, to rest at the
// end) above max_acceleration.
inline std::expected<void, std::string> check_limits(const ec::JointTrajectory& t, const PlanOptions& o) {
    const std::size_t n = t.size();
    const auto dt = [&t](std::size_t from, std::size_t to) { return seconds(t[to].stamp - t[from].stamp); };
    const auto speed = [&t, &dt](std::size_t k, std::size_t j) {  // of segment k, from point k to k + 1
        return (t[k + 1].value[j] - t[k].value[j]) / dt(k, k + 1);
    };
    for (std::size_t k = 0; k < n; ++k)
        for (std::size_t j = 0; j < kJointCount; ++j) {
            if (!(std::abs(t[k].value[j]) <= o.joint_limit))
                return std::unexpected(
                    std::format("point {}: joint {} at {} rad is outside the limit", k, j + 1, t[k].value[j]));
            if (k > 0 && std::abs(speed(k - 1, j)) > o.max_velocity[j])
                return std::unexpected(std::format("point {}: joint {} would move at {:.3f} rad/s, above {:.3f}", k,
                                                   j + 1, std::abs(speed(k - 1, j)), o.max_velocity[j]));
        }
    for (std::size_t j = 0; j < kJointCount; ++j) {
        double peak = std::max(std::abs(speed(0, j)) / dt(0, 1), std::abs(speed(n - 2, j)) / dt(n - 2, n - 1));
        for (std::size_t k = 1; k + 1 < n; ++k)
            peak = std::max(peak, std::abs(speed(k, j) - speed(k - 1, j)) / (dt(k - 1, k + 1) / 2.0));
        if (peak > o.max_acceleration[j] * (1.0 + 1e-9))
            return std::unexpected(std::format("joint {} would accelerate at {:.3f} rad/s^2, above {:.3f}", j + 1, peak,
                                               o.max_acceleration[j]));
    }
    return {};
}

}  // namespace detail

/// Turns a motion of the TCP into a joint trajectory, starting from the joints
/// now (get_joint_pos), with the controller's inverse kinematics
/// (inverseKinematic, one call per point but a pause). The trajectory runs from
/// rest to rest and passes through every pose at its time without stopping,
/// except at a pause. Invalid, naming the pose, for an invalid motion, a failed
/// check or a trajectory too fast for the joints; a failed call keeps its kind
/// and gains the pose in its message.
inline std::expected<ec::JointTrajectory, Error> plan(ec::Robot& robot, const PoseTrajectory& motion,
                                                      const PlanOptions& o = {}) {
    using namespace detail;
    if (auto ok = validate(motion); !ok) return invalid("not a motion: " + ok.error());
    if (!(o.max_step > 0.0 && o.max_turn > 0.0 && o.period > 0.0))
        return invalid("max_step, max_turn and period must be positive");
    auto points = fill_in(motion, o);
    if (!points) return std::unexpected(points.error());
    const auto now = robot.call(ec::sdk::GetJointPos{});
    if (!now) return std::unexpected(now.error());

    // Solve every point, each near the one before, and check it.
    std::vector<double> times;
    std::vector<Joints> solutions;
    times.reserve(points->size());
    solutions.reserve(points->size());
    Joints reference = ec::deg_to_rad(*now);
    for (std::size_t k = 0; k < points->size(); ++k) {
        const Point& p = (*points)[k];
        if (k > 0 && same_pose(p.pose, (*points)[k - 1].pose)) {  // a pause: the same joints, held
            times.push_back(p.time);
            solutions.push_back(reference);
            continue;
        }
        auto q = robot.call(
            ec::sdk::InverseKinematic{.target = ec::to_mm_rad(p.pose), .reference = ec::rad_to_deg(reference)});
        if (!q) {  // no solution, or no answer: the message says which
            Error e = std::move(q.error());
            e.message = std::format("inverse kinematics failed for the pose {}: {}", where(p), e.message);
            return std::unexpected(std::move(e));
        }
        // Each joint taken by whole turns to the value nearest the one before.
        const Joints solution = ec::nearest_turn(ec::deg_to_rad(*q), reference, o.joint_limit);
        // The first point is checked where it is: the arm is moved there before
        // the trajectory runs (the manual's workflow, 2.2.9.10).
        if (auto ok = k == 0 ? check_point(solution, o) : check_step(reference, solution, o); !ok)
            return invalid(std::format("the pose {}: {}", where(p), ok.error()));
        reference = solution;
        times.push_back(p.time);
        solutions.push_back(solution);
    }

    // Interpolate over time, and sample: the last point exactly at the end.
    const double end = times.back();
    const JointSpline spline(std::move(times), std::move(solutions));
    const double period = std::min(o.period, end / 2.0);  // at least 3 points
    const auto steps = static_cast<std::size_t>(std::floor(end / period));
    ec::JointTrajectory trajectory;
    trajectory.reserve(steps + 2);
    for (std::size_t k = 0; k <= steps; ++k) {
        const double t = period * static_cast<double>(k);
        if (k > 0 && end - t < period / 2.0) break;  // the end comes next, not this close behind
        trajectory.push_back({stamp(t), spline.position(t)});
    }
    trajectory.push_back({motion.back().stamp, spline.position(end)});
    if (auto ok = check_limits(trajectory, o); !ok) return invalid(ok.error());
    return trajectory;
}

}  // namespace planning
