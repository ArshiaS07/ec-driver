#include "ec/motion/planning.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>
#include <utility>

#include "ec/pose.hpp"

namespace ec::motion {

// ------------------------------------------------------------------ SpeedProfile

std::expected<SpeedProfile, std::string> SpeedProfile::make(double distance, double speed, double ramp) {
    if (!(std::isfinite(distance) && distance >= 0.0))
        return std::unexpected(std::format("the distance must be finite and not negative, not {} m", distance));
    if (!(std::isfinite(speed) && speed > 0.0))
        return std::unexpected(std::format("the speed must be positive, not {} m/s", speed));
    if (!(std::isfinite(ramp) && ramp >= 0.0))
        return std::unexpected(std::format("the ramp must be finite and not negative, not {} s", ramp));
    // Each ramp covers cruise * ramp / 2: two of them must fit into the distance.
    const double cruise = ramp > 0.0 ? std::min(speed, distance / ramp) : speed;
    return SpeedProfile(distance, cruise, ramp);
}

double SpeedProfile::duration() const noexcept {
    if (total_ == 0.0 || cruise_ == 0.0) return 0.0;
    return ramp_ + total_ / cruise_;
}

double SpeedProfile::ramped(double t) const noexcept {
    if (ramp_ == 0.0) return 0.0;
    return cruise_ / 2.0 * (t - ramp_ / std::numbers::pi * std::sin(std::numbers::pi * t / ramp_));
}

double SpeedProfile::distance(double t) const noexcept {
    const double end = duration();
    if (!(t > 0.0)) return 0.0;
    if (t >= end) return total_;
    if (t < ramp_) return ramped(t);
    if (t <= end - ramp_) return cruise_ * ramp_ / 2.0 + cruise_ * (t - ramp_);
    return std::clamp(total_ - ramped(end - t), 0.0, total_);
}

double SpeedProfile::speed(double t) const noexcept {
    const double end = duration();
    if (!(t > 0.0) || t >= end) return 0.0;
    if (t < ramp_) return cruise_ / 2.0 * (1.0 - std::cos(std::numbers::pi * t / ramp_));
    if (t <= end - ramp_) return cruise_;
    return cruise_ / 2.0 * (1.0 - std::cos(std::numbers::pi * (end - t) / ramp_));
}

double SpeedProfile::time_at(double d) const noexcept {
    const double end = duration();
    if (!(d > 0.0)) return 0.0;
    if (d >= total_) return end;
    double lo = 0.0;  // distance(lo) < d <= distance(hi)
    double hi = end;
    for (int k = 0; k < 60; ++k) {
        const double mid = (lo + hi) / 2.0;
        (distance(mid) < d ? lo : hi) = mid;
    }
    return hi;
}

double SpeedProfile::peak_acceleration() const noexcept {
    if (ramp_ == 0.0) return cruise_ > 0.0 ? std::numeric_limits<double>::infinity() : 0.0;
    return cruise_ * std::numbers::pi / (2.0 * ramp_);
}

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
    if (ends == Ends::Rest) {
        slopes.front() = slopes.back() = Joints{};
        // From rest, an end segment overshoots backwards unless the slope at its
        // other knot stays within three times its secant (Fritsch and Carlson):
        // keep it there, so that motion from rest starts forwards.
        const auto limit = [](double& slope, double chord) {
            slope = chord >= 0.0 ? std::clamp(slope, 0.0, 3.0 * chord) : std::clamp(slope, 3.0 * chord, 0.0);
        };
        if (n >= 3) {
            for (std::size_t j = 0; j < kJointCount; ++j) {
                limit(slopes[1][j], secant[0][j]);
                limit(slopes[n - 2][j], secant[n - 2][j]);
            }
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
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message), {}});
}

std::string where(const TimedPose& p) {
    const Eigen::Vector3d r = p.pose.translation() * 1000.0;
    return std::format("at {:.3f} s ({:.1f}, {:.1f}, {:.1f} mm)", p.time, r.x(), r.y(), r.z());
}

bool is_rotation(const Eigen::Matrix3d& r) {
    return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-6 &&
           r.determinant() > 0.0;
}

// How far a joint angle is from the nearest multiple of pi [rad].
double from_singular(double q) noexcept { return std::abs(std::remainder(q, std::numbers::pi)); }

// The path with points inserted, along straight lines and even turns, so that no
// two neighbours are further apart than max_step or max_turn.
std::expected<std::vector<TimedPose>, Error> fill_in(std::span<const TimedPose> path, const PlanOptions& o) {
    std::vector<TimedPose> points{path.front()};
    for (std::size_t k = 1; k < path.size(); ++k) {
        const TimedPose& a = path[k - 1];
        const TimedPose& b = path[k];
        const double gap = (b.pose.translation() - a.pose.translation()).norm();
        const double turn = angle_between(a.pose.linear(), b.pose.linear());
        const double pieces = std::max({1.0, std::ceil(gap / o.max_step), std::ceil(turn / o.max_turn)});
        if (static_cast<double>(points.size()) + pieces > static_cast<double>(o.max_points))
            return invalid(std::format("the path needs more than {} points {} mm and {} deg apart", o.max_points,
                                       o.max_step * 1000.0, rad_to_deg(o.max_turn)));
        const auto n = static_cast<std::size_t>(pieces);
        const Eigen::Quaterniond qa(a.pose.linear());
        const Eigen::Quaterniond qb(b.pose.linear());
        for (std::size_t i = 1; i < n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(n);
            TimedPose p;
            p.time = a.time + (b.time - a.time) * u;
            p.pose.translation() = a.pose.translation() + (b.pose.translation() - a.pose.translation()) * u;
            p.pose.linear() = qa.slerp(u, qb).toRotationMatrix();
            points.push_back(p);
        }
        points.push_back(b);
    }
    return points;
}

}  // namespace

std::expected<JointTrajectory, Error> plan(std::span<const TimedPose> path, const Joints& start,
                                           const InverseKinematics& ik, const PlanOptions& o) {
    if (path.size() < 2) return invalid(std::format("{} poses; a path needs at least 2", path.size()));
    if (!ik) return invalid("no inverse kinematics given");
    if (!(o.max_step > 0.0 && o.max_turn > 0.0 && o.period > 0.0))
        return invalid("max_step, max_turn and period must be positive");
    if (path.front().time != 0.0) return invalid(std::format("the path starts at {} s, not 0", path.front().time));
    for (std::size_t k = 0; k < path.size(); ++k) {
        const TimedPose& p = path[k];
        if (!std::isfinite(p.time) || (k > 0 && !(p.time > path[k - 1].time)))
            return invalid(std::format("point {}: its time, {} s, does not increase", k, p.time));
        if (!p.pose.translation().allFinite() || !is_rotation(p.pose.linear()))
            return invalid(std::format("point {}: not a pose (a finite position and a rotation)", k));
    }
    auto points = fill_in(path, o);
    if (!points) return std::unexpected(points.error());

    // Solve every point, each near the one before.
    std::vector<double> times;
    std::vector<Joints> solutions;
    times.reserve(points->size());
    solutions.reserve(points->size());
    Joints reference = start;
    for (std::size_t k = 0; k < points->size(); ++k) {
        const TimedPose& p = (*points)[k];
        auto q = ik(p.pose, reference);
        if (!q) {  // no solution, or no answer: the message says which
            Error e = std::move(q.error());
            e.message = std::format("inverse kinematics failed for the pose {}: {}", where(p), e.message);
            return std::unexpected(std::move(e));
        }
        const Joints solution = nearest_turn(*q, reference, o.limits.joint_limit);
        for (std::size_t j = 0; j < kJointCount; ++j) {
            if (k > 0 && std::abs(solution[j] - reference[j]) > o.max_joint_step)
                return invalid(std::format(
                    "joint {} turns {:.1f} deg between {:.3f} and {:.3f} s: the arm would "
                    "change configuration or pass close to a singularity",
                    j + 1, rad_to_deg(std::abs(solution[j] - reference[j])), (*points)[k - 1].time, p.time));
            if (std::abs(solution[j]) > o.limits.joint_limit)
                return invalid(std::format("joint {} at {:.1f} deg {}: beyond the joint limit", j + 1,
                                           rad_to_deg(solution[j]), where(p)));
        }
        if (from_singular(solution[4]) < o.singularity_margin)
            return invalid(
                std::format("J5 at {:.1f} deg {}: within {:.1f} deg of the wrist singularity (J5 at 0 or "
                            "180 deg)",
                            rad_to_deg(solution[4]), where(p), rad_to_deg(o.singularity_margin)));
        if (from_singular(solution[2]) < o.singularity_margin)
            return invalid(
                std::format("J3 at {:.1f} deg {}: within {:.1f} deg of the elbow singularity (the arm "
                            "stretched or folded, J3 at 0 or 180 deg)",
                            rad_to_deg(solution[2]), where(p), rad_to_deg(o.singularity_margin)));
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
