#include "ec/path.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace ec {

std::expected<void, std::string> validate(const Circle& c) {
    if (!c.center.allFinite()) return std::unexpected(std::string("the centre is not finite"));
    if (!(std::isfinite(c.radius) && c.radius > 0.0))
        return std::unexpected(std::format("the radius must be positive, not {} m", c.radius));
    if (!c.normal.allFinite() || !(c.normal.norm() > 0.0))
        return std::unexpected(std::string("the normal must be a non-zero vector"));
    if (!c.start.allFinite()) return std::unexpected(std::string("the start direction is not finite"));
    if (!c.orientation.coeffs().allFinite() || std::abs(c.orientation.norm() - 1.0) > 1e-6)
        return std::unexpected(std::string("the orientation is not a unit quaternion"));
    if (!(std::isfinite(c.turns) && c.turns > 0.0))
        return std::unexpected(std::format("turns must be positive, not {}", c.turns));
    return {};
}

CircleAxes axes(const Circle& c) noexcept {
    const Eigen::Vector3d n = c.normal.normalized();
    const auto in_plane = [&n](const Eigen::Vector3d& d) -> Eigen::Vector3d { return d - n * n.dot(d); };
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    if (c.start.norm() > 0.0) u = in_plane(c.start.normalized());
    if (u.norm() < 1e-6) {
        // No usable direction given: base x, or base y when the normal is within 30 degrees of x.
        u = std::abs(n.x()) < std::cos(std::numbers::pi / 6.0) ? in_plane(Eigen::Vector3d::UnitX())
                                                               : in_plane(Eigen::Vector3d::UnitY());
    }
    u.normalize();
    return {u, n.cross(u), n};
}

double length(const Circle& c) noexcept { return 2.0 * std::numbers::pi * c.radius * c.turns; }

Eigen::Isometry3d pose_at(const Circle& c, double s) noexcept {
    const double angle = std::clamp(s, 0.0, length(c)) / c.radius;
    const CircleAxes a = axes(c);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = c.center + c.radius * (std::cos(angle) * a.u + std::sin(angle) * a.v);
    pose.linear() = c.orientation.toRotationMatrix();
    return pose;
}

double length(const Line& l) noexcept { return (l.to.translation() - l.from.translation()).norm(); }

Eigen::Isometry3d pose_at(const Line& l, double s) noexcept {
    const double total = length(l);
    if (!(total > 0.0)) return l.to;
    const double u = std::clamp(s, 0.0, total) / total;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = l.from.translation() + u * (l.to.translation() - l.from.translation());
    pose.linear() = Eigen::Quaterniond(l.from.linear()).slerp(u, Eigen::Quaterniond(l.to.linear())).toRotationMatrix();
    return pose;
}

}  // namespace ec
