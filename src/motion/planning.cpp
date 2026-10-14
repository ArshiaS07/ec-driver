#include "ec/motion/planning.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <numbers>
#include <utility>

#include "ec/pose.hpp"

namespace ec::motion {

// ------------------------------------------------------------------- JointSpline

std::expected<JointSpline, std::string> JointSpline::make(std::vector<double> knots, std::vector<Joints> positions,
                                                          Ends ends) {
    const std::size_t n = knots.size();
    if (n < 2) return std::unexpected(std::format("{} knots; at least 2 are needed", n));
    if (positions.size() != n) return std::unexpected(std::format("{} knots but {} positions", n, positions.size()));
    for (std::size_t k = 0; k < n; ++k) {
        if (!std::isfinite(knots[k])) return std::unexpected(std::format("knot {} is not finite", k));
        if (k > 0 && !(knots[k] > knots[k - 1]))
            return std::unexpected(std::format("knot {} ({}) does not increase", k, knots[k]));
        for (double q : positions[k])
            if (!std::isfinite(q)) return std::unexpected(std::format("position {} is not finite", k));
    }
    // Secant slopes, then each knot's slope from the parabola through it and its neighbours.
    std::vector<Joints> secant(n - 1);
    for (std::size_t k = 0; k + 1 < n; ++k) {
        const double h = knots[k + 1] - knots[k];
        for (std::size_t j = 0; j < kJointCount; ++j) secant[k][j] = (positions[k + 1][j] - positions[k][j]) / h;
    }
    std::vector<Joints> slopes(n);
    if (n == 2) {
        slopes[0] = slopes[1] = secant[0];
    } else {
        for (std::size_t k = 1; k + 1 < n; ++k) {
            const double h0 = knots[k] - knots[k - 1];
            const double h1 = knots[k + 1] - knots[k];
            for (std::size_t j = 0; j < kJointCount; ++j)
                slopes[k][j] = (h1 * secant[k - 1][j] + h0 * secant[k][j]) / (h0 + h1);
        }
        const double a0 = knots[1] - knots[0];
        const double a1 = knots[2] - knots[1];
        const double b0 = knots[n - 2] - knots[n - 3];
        const double b1 = knots[n - 1] - knots[n - 2];
        for (std::size_t j = 0; j < kJointCount; ++j) {
            slopes[0][j] = secant[0][j] + (secant[0][j] - secant[1][j]) * a0 / (a0 + a1);
            slopes[n - 1][j] = secant[n - 2][j] + (secant[n - 2][j] - secant[n - 3][j]) * b1 / (b0 + b1);
        }
    }
    // At rest: the ends, if asked, and both knots of a pause (the same positions
    // twice). A segment into or out of rest overshoots unless the slope at its
    // other knot stays within three times its secant (Fritsch and Carlson): keep
    // it there, so that motion from rest starts forwards and comes to rest
    // without going past.
    std::vector<bool> rest(n, false);
    if (ends == Ends::Rest) rest.front() = rest.back() = true;
    for (std::size_t k = 0; k + 1 < n; ++k)
        if (positions[k] == positions[k + 1]) rest[k] = rest[k + 1] = true;
    const auto limit = [](double& slope, double chord) {
        slope = chord >= 0.0 ? std::clamp(slope, 0.0, 3.0 * chord) : std::clamp(slope, 3.0 * chord, 0.0);
    };
    for (std::size_t k = 0; k < n; ++k) {
        if (!rest[k]) continue;
        slopes[k] = Joints{};
        for (std::size_t j = 0; j < kJointCount; ++j) {
            if (k > 0 && !rest[k - 1]) limit(slopes[k - 1][j], secant[k - 1][j]);
            if (k + 1 < n && !rest[k + 1]) limit(slopes[k + 1][j], secant[k][j]);
        }
    }
    return JointSpline(std::move(knots), std::move(positions), std::move(slopes));
}

std::size_t JointSpline::segment(double x) const noexcept {
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);  // the first knot after x
    const auto k = static_cast<std::size_t>(it - x_.begin());
    return std::clamp<std::size_t>(k, 1, x_.size() - 1) - 1;
}

Joints JointSpline::position(double x) const noexcept {
    if (!(x > x_.front())) return q_.front();
    if (x >= x_.back()) return q_.back();
    const std::size_t k = segment(x);
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

Joints JointSpline::slope(double x) const noexcept {
    if (!(x > x_.front()) || x >= x_.back()) return {};
    const std::size_t k = segment(x);
    const double h = x_[k + 1] - x_[k];
    const double t = (x - x_[k]) / h;
    const double t2 = t * t;
    const double d00 = 6.0 * t2 - 6.0 * t;
    const double d10 = 3.0 * t2 - 4.0 * t + 1.0;
    const double d01 = -6.0 * t2 + 6.0 * t;
    const double d11 = 3.0 * t2 - 2.0 * t;
    Joints m{};
    for (std::size_t j = 0; j < kJointCount; ++j)
        m[j] = (d00 * q_[k][j] + d01 * q_[k + 1][j]) / h + d10 * m_[k][j] + d11 * m_[k + 1][j];
    return m;
}

// -------------------------------------------------------------------------- plan

namespace {

std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message)});
}

// A pose of the motion, or one filled in between two, at its time [s].
struct Point {
    double time = 0.0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

std::string where(const Point& p) {
    const Eigen::Vector3d r = p.pose.translation() * 1000.0;
    return std::format("at {:.3f} s ({:.1f}, {:.1f}, {:.1f} mm)", p.time, r.x(), r.y(), r.z());
}

// Whether two poses are the same: within a nanometre and a nanoradian.
bool same_pose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return (a.translation() - b.translation()).norm() <= 1e-9 && angle_between(a.linear(), b.linear()) <= 1e-9;
}

// How far a joint angle is from the nearest multiple of pi [rad].
double from_singular(double q) noexcept { return std::abs(std::remainder(q, std::numbers::pi)); }

// The motion with points inserted, along straight lines and even turns, so that
// no two neighbours are further apart than max_step or max_turn.
std::expected<std::vector<Point>, Error> fill_in(const PoseTrajectory& motion, const PlanOptions& o) {
    const auto seconds = [](Clock::duration d) { return std::chrono::duration<double>(d).count(); };
    std::vector<Point> points{{.time = seconds(motion.front().stamp), .pose = motion.front().value}};
    for (std::size_t k = 1; k < motion.size(); ++k) {
        const Point a = points.back();
        const Point b{.time = seconds(motion[k].stamp), .pose = motion[k].value};
        const double gap = (b.pose.translation() - a.pose.translation()).norm();
        const double turn = angle_between(a.pose.linear(), b.pose.linear());
        const double pieces = std::max({1.0, std::ceil(gap / o.max_step), std::ceil(turn / o.max_turn)});
        if (static_cast<double>(points.size()) + pieces > static_cast<double>(o.max_points))
            return invalid(std::format("the motion needs more than {} points {} mm and {} deg apart", o.max_points,
                                       o.max_step * 1000.0, rad_to_deg(o.max_turn)));
        const auto n = static_cast<std::size_t>(pieces);
        for (std::size_t i = 1; i < n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(n);
            points.push_back({.time = a.time + (b.time - a.time) * u, .pose = interpolate(a.pose, b.pose, u)});
        }
        points.push_back(b);
    }
    return points;
}

}  // namespace

std::expected<void, std::string> check_point(const Joints& q, const PlanOptions& o) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(q[j]) <= o.limits.joint_limit))
            return std::unexpected(
                std::format("joint {} at {:.1f} deg: beyond the joint limit", j + 1, rad_to_deg(q[j])));
    if (from_singular(q[4]) < o.singularity_margin)
        return std::unexpected(
            std::format("J5 at {:.1f} deg: within {:.1f} deg of the wrist singularity (J5 at 0 or 180 deg)",
                        rad_to_deg(q[4]), rad_to_deg(o.singularity_margin)));
    if (from_singular(q[2]) < o.singularity_margin)
        return std::unexpected(
            std::format("J3 at {:.1f} deg: within {:.1f} deg of the elbow singularity (the arm stretched or folded, "
                        "J3 at 0 or 180 deg)",
                        rad_to_deg(q[2]), rad_to_deg(o.singularity_margin)));
    return {};
}

std::expected<void, std::string> check_step(const Joints& from, const Joints& to, const PlanOptions& o) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(to[j] - from[j]) <= o.max_joint_step))
            return std::unexpected(std::format(
                "joint {} turns {:.1f} deg in one step: the arm would change configuration, or pass close to a "
                "singularity",
                j + 1, rad_to_deg(std::abs(to[j] - from[j]))));
    return check_point(to, o);
}

std::expected<JointTrajectory, Error> plan(const PoseTrajectory& motion, const Joints& start,
                                           const InverseKinematics& ik, const PlanOptions& o) {
    if (auto ok = ec::validate(motion); !ok) return invalid("not a motion: " + ok.error());
    if (!ik) return invalid("no inverse kinematics given");
    if (!(o.max_step > 0.0 && o.max_turn > 0.0 && o.period > 0.0))
        return invalid("max_step, max_turn and period must be positive");
    auto points = fill_in(motion, o);
    if (!points) return std::unexpected(points.error());

    // Solve every point, each near the one before, and check it.
    std::vector<double> times;
    std::vector<Joints> solutions;
    times.reserve(points->size());
    solutions.reserve(points->size());
    Joints reference = start;
    for (std::size_t k = 0; k < points->size(); ++k) {
        const Point& p = (*points)[k];
        if (k > 0 && same_pose(p.pose, (*points)[k - 1].pose)) {  // a pause: the same joints, held
            times.push_back(p.time);
            solutions.push_back(reference);
            continue;
        }
        auto q = ik(p.pose, reference);
        if (!q) {  // no solution, or no answer: the message says which
            Error e = std::move(q.error());
            e.message = std::format("inverse kinematics failed for the pose {}: {}", where(p), e.message);
            return std::unexpected(std::move(e));
        }
        const Joints solution = nearest_turn(*q, reference, o.limits.joint_limit);
        // The first point is checked where it is; it must be near `start` for the
        // trajectory to be played from there, which Timestamped::upload checks.
        if (auto ok = k == 0 ? check_point(solution, o) : check_step(reference, solution, o); !ok)
            return invalid(std::format("the pose {}: {}", where(p), ok.error()));
        reference = solution;
        times.push_back(p.time);
        solutions.push_back(solution);
    }

    // Interpolate over time, and sample.
    auto spline = JointSpline::make(std::move(times), std::move(solutions), JointSpline::Ends::Rest);
    if (!spline) return invalid(spline.error());
    const double end = spline->knots().back();
    const double period = std::min(o.period, end / 2.0);  // at least 3 points
    JointTrajectory trajectory;
    const auto steps = static_cast<std::size_t>(std::floor(end / period));
    trajectory.time.reserve(steps + 2);
    trajectory.position.reserve(steps + 2);
    for (std::size_t k = 0; k <= steps; ++k) {
        const double t = period * static_cast<double>(k);
        if (k > 0 && end - t < period / 2.0) break;  // the end comes next, not this close behind
        trajectory.time.push_back(t);
        trajectory.position.push_back(spline->position(t));
    }
    trajectory.time.push_back(end);
    trajectory.position.push_back(spline->position(end));
    if (auto ok = validate(trajectory, o.limits); !ok) return invalid(ok.error());
    return trajectory;
}

SingularityMargins singularity_margins(std::span<const Joints> positions) noexcept {
    SingularityMargins m;
    for (const Joints& q : positions) {
        m.wrist = std::min(m.wrist, from_singular(q[4]));
        m.elbow = std::min(m.elbow, from_singular(q[2]));
    }
    return m;
}

}  // namespace ec::motion
