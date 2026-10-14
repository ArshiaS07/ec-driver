#include "ec/motion/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace ec::motion {

std::expected<void, std::string> validate(const JointTrajectory& t, const TrajectoryLimits& limits) {
    if (t.time.size() != t.position.size())
        return std::unexpected(std::format("{} times but {} positions", t.time.size(), t.position.size()));
    if (t.size() < 3) return std::unexpected(std::format("{} points; at least 3 are needed", t.size()));
    if (t.time.front() != 0.0) return std::unexpected(std::format("the first time is {} s, not 0", t.time.front()));
    for (std::size_t k = 0; k < t.size(); ++k) {
        for (std::size_t j = 0; j < kJointCount; ++j) {
            const double q = t.position[k][j];
            if (!std::isfinite(q) || std::abs(q) > limits.joint_limit)
                return std::unexpected(std::format("point {}: joint {} at {} rad is outside the limit", k, j + 1, q));
        }
        if (k == 0) continue;
        const double dt = t.time[k] - t.time[k - 1];
        if (!(dt > 0.0)) return std::unexpected(std::format("point {}: time {} s does not increase", k, t.time[k]));
        for (std::size_t j = 0; j < kJointCount; ++j) {
            const double v = std::abs(t.position[k][j] - t.position[k - 1][j]) / dt;
            if (v > limits.max_velocity[j])
                return std::unexpected(std::format("point {}: joint {} would move at {:.3f} rad/s, above {:.3f}", k,
                                                   j + 1, v, limits.max_velocity[j]));
        }
    }
    const Joints a = peak_acceleration(t);
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (a[j] > limits.max_acceleration[j] * (1.0 + 1e-9))
            return std::unexpected(std::format("joint {} would accelerate at {:.3f} rad/s^2, above {:.3f}", j + 1, a[j],
                                               limits.max_acceleration[j]));
    return {};
}

Joints peak_velocity(const JointTrajectory& t) noexcept {
    Joints peak{};
    for (std::size_t k = 1; k < t.size(); ++k) {
        const double dt = t.time[k] - t.time[k - 1];
        for (std::size_t j = 0; j < kJointCount; ++j)
            peak[j] = std::max(peak[j], std::abs(t.position[k][j] - t.position[k - 1][j]) / dt);
    }
    return peak;
}

Joints peak_acceleration(const JointTrajectory& t) noexcept {
    Joints peak{};
    const std::size_t n = t.size();
    if (n < 2) return peak;
    // The speed of segment k, from point k to k + 1; at rest before the first and after the last.
    const auto speed = [&t](std::size_t k, std::size_t j) {
        return (t.position[k + 1][j] - t.position[k][j]) / (t.time[k + 1] - t.time[k]);
    };
    for (std::size_t j = 0; j < kJointCount; ++j) {
        peak[j] = std::max(std::abs(speed(0, j)) / (t.time[1] - t.time[0]),
                           std::abs(speed(n - 2, j)) / (t.time[n - 1] - t.time[n - 2]));
        for (std::size_t k = 1; k + 1 < n; ++k) {
            const double dt = (t.time[k + 1] - t.time[k - 1]) / 2.0;
            peak[j] = std::max(peak[j], std::abs(speed(k, j) - speed(k - 1, j)) / dt);
        }
    }
    return peak;
}

Joints position_at(const JointTrajectory& t, double s) noexcept {
    if (t.size() == 0) return {};
    if (!(s > t.time.front())) return t.position.front();
    if (s >= t.time.back()) return t.position.back();
    const auto it = std::upper_bound(t.time.begin(), t.time.end(), s);  // the first point after s
    const auto k = static_cast<std::size_t>(it - t.time.begin());
    const double u = (s - t.time[k - 1]) / (t.time[k] - t.time[k - 1]);
    Joints q{};
    for (std::size_t j = 0; j < kJointCount; ++j)
        q[j] = t.position[k - 1][j] + (t.position[k][j] - t.position[k - 1][j]) * u;
    return q;
}

}  // namespace ec::motion
