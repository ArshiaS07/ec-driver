#include "ec/path.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace ec {
namespace {

bool is_finite(const Vec3& v) noexcept { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }

}  // namespace

std::expected<void, std::string> validate(const Circle& c) {
    if (!is_finite(c.center)) return std::unexpected(std::string("the centre is not finite"));
    if (!(std::isfinite(c.radius) && c.radius > 0.0))
        return std::unexpected(std::format("the radius must be positive, not {} m", c.radius));
    if (!is_finite(c.normal) || !(norm(c.normal) > 0.0))
        return std::unexpected(std::string("the normal must be a non-zero vector"));
    if (!is_finite(c.start)) return std::unexpected(std::string("the start direction is not finite"));
    if (!is_finite(c.rotation)) return std::unexpected(std::string("the rotation is not finite"));
    if (!(std::isfinite(c.turns) && c.turns > 0.0))
        return std::unexpected(std::format("turns must be positive, not {}", c.turns));
    return {};
}

CircleAxes axes(const Circle& c) noexcept {
    const Vec3 n = mul(c.normal, 1.0 / norm(c.normal));
    const auto in_plane = [&n](const Vec3& d) { return sub(d, mul(n, dot(d, n))); };
    Vec3 u{};
    if (const double length = norm(c.start); length > 0.0) u = in_plane(mul(c.start, 1.0 / length));
    if (norm(u) < 1e-6) {
        // No usable direction given: base x, or base y when the normal is within 30 degrees of x.
        u = std::abs(n[0]) < std::cos(std::numbers::pi / 6.0) ? in_plane(Vec3{1.0, 0.0, 0.0})
                                                              : in_plane(Vec3{0.0, 1.0, 0.0});
    }
    u = mul(u, 1.0 / norm(u));
    return {u, cross(n, u), n};
}

double length(const Circle& c) noexcept { return 2.0 * std::numbers::pi * c.radius * c.turns; }

Pose pose_at(const Circle& c, double s) noexcept {
    const double angle = std::clamp(s, 0.0, length(c)) / c.radius;
    const CircleAxes a = axes(c);
    const Vec3 offset = add(mul(a.u, std::cos(angle)), mul(a.v, std::sin(angle)));
    return {add(c.center, mul(offset, c.radius)), c.rotation};
}

}  // namespace ec
