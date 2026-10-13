#include "ec/motion/planning.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>
#include <utility>

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

double SpeedProfile::peak_acceleration() const noexcept {
    if (ramp_ == 0.0) return cruise_ > 0.0 ? std::numeric_limits<double>::infinity() : 0.0;
    return cruise_ * std::numbers::pi / (2.0 * ramp_);
}

// ------------------------------------------------------------------- JointSpline

std::expected<JointSpline, std::string> JointSpline::make(std::vector<double> knots, std::vector<Joints> positions) {
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

// ------------------------------------------------------------------------ solve

std::expected<JointPath, Error> solve(std::vector<double> s, std::vector<Pose> poses, const Joints& start,
                                      const InverseKinematics& ik, const PlanOptions& options) {
    const auto invalid = [](std::string message) {
        return std::unexpected(Error{Error::Kind::Invalid, std::move(message), {}});
    };
    if (s.size() != poses.size()) return invalid(std::format("{} distances but {} poses", s.size(), poses.size()));
    if (s.size() < 2) return invalid(std::format("{} poses; a path needs at least 2", s.size()));
    if (!ik) return invalid("no inverse kinematics given");
    std::vector<Joints> joints;
    joints.reserve(poses.size());
    Joints reference = start;
    for (std::size_t k = 0; k < poses.size(); ++k) {
        const Pose& p = poses[k];
        auto q = ik(p, reference);
        if (!q) {
            Error e = std::move(q.error());
            e.message = std::format(
                "no joint positions for the pose at {:.1f} mm along the path ({:.1f}, {:.1f}, "
                "{:.1f} mm): {}",
                s[k] * 1000.0, p.position[0] * 1000.0, p.position[1] * 1000.0, p.position[2] * 1000.0, e.message);
            if (e.kind == Error::Kind::Command) e.kind = Error::Kind::Invalid;  // the controller found no solution
            return std::unexpected(std::move(e));
        }
        *q = nearest_turn(*q, reference, kEc66JointLimit);  // a solution given within one turn
        if (k > 0) {
            for (std::size_t j = 0; j < kJointCount; ++j) {
                const double step = std::abs((*q)[j] - reference[j]);
                if (step > options.max_joint_step)
                    return invalid(
                        std::format("joint {} turns {:.1f} deg between {:.1f} and {:.1f} mm along the path: "
                                    "the arm would change configuration or pass a singularity there",
                                    j + 1, rad_to_deg(step), s[k - 1] * 1000.0, s[k] * 1000.0));
            }
        }
        reference = *q;
        joints.push_back(*q);
    }
    auto spline = JointSpline::make(std::move(s), std::move(joints));
    if (!spline) return invalid(spline.error());
    return JointPath{.joints = std::move(*spline), .poses = std::move(poses)};
}

// --------------------------------------------------------------------- TimedPath

std::expected<TimedPath, Error> TimedPath::make(JointPath path, double speed, double ramp) {
    auto profile = SpeedProfile::make(path.length(), speed, ramp);
    if (!profile) return std::unexpected(Error{Error::Kind::Invalid, profile.error(), {}});
    return TimedPath(std::move(path), *profile);
}

Joints TimedPath::position(double t) const noexcept { return path_.joints.position(profile_.distance(t)); }

Joints TimedPath::velocity(double t) const noexcept {
    return scale(path_.joints.slope(profile_.distance(t)), profile_.speed(t));
}

JointTrajectory TimedPath::sample(double period) const {
    JointTrajectory out;
    const double end = duration();
    if (!(period > 0.0) || !(end > 0.0)) {
        out.time = {0.0};
        out.position = {position(0.0)};
        return out;
    }
    const auto steps = static_cast<std::size_t>(std::floor(end / period));
    out.time.reserve(steps + 2);
    out.position.reserve(steps + 2);
    for (std::size_t k = 0; k <= steps; ++k) {
        const double t = period * static_cast<double>(k);
        if (end - t < period / 2.0 && k > 0) break;  // the end comes next, not this close behind
        out.time.push_back(t);
        out.position.push_back(position(t));
    }
    out.time.push_back(end);
    out.position.push_back(position(end));
    return out;
}

}  // namespace ec::motion
