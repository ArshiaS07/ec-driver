#pragma once
/// @file
/// A Cartesian pose in SI units, with the controller's rotation convention:
/// R = Rz(rz) * Ry(ry) * Rx(rx), measured on the EA66 (ADR 0012). Also the few
/// vector operations the paths need, and the conversion to the controller's
/// units (mm, rad).

#include <algorithm>
#include <array>
#include <cmath>

#include "ec/units.hpp"

namespace ec {

using Vec3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;

/// A pose: a position [m] and the controller's rotation angles rx, ry, rz [rad].
struct Pose {
    Vec3 position{};
    Vec3 rotation{};
    bool operator==(const Pose&) const = default;
};

// ---- vectors

[[nodiscard]] constexpr Vec3 add(const Vec3& a, const Vec3& b) noexcept {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
[[nodiscard]] constexpr Vec3 sub(const Vec3& a, const Vec3& b) noexcept {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
[[nodiscard]] constexpr Vec3 mul(const Vec3& a, double k) noexcept { return {a[0] * k, a[1] * k, a[2] * k}; }
[[nodiscard]] constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
[[nodiscard]] inline double norm(const Vec3& a) noexcept { return std::hypot(a[0], a[1], a[2]); }

/// The distance [m] between two positions.
[[nodiscard]] inline double distance(const Vec3& a, const Vec3& b) noexcept { return norm(sub(a, b)); }

// ---- rotations

/// The rotation matrix of rx, ry, rz: Rz(rz) * Ry(ry) * Rx(rx).
[[nodiscard]] inline Matrix3 rotation_matrix(const Vec3& r) noexcept {
    const double cx = std::cos(r[0]), sx = std::sin(r[0]);
    const double cy = std::cos(r[1]), sy = std::sin(r[1]);
    const double cz = std::cos(r[2]), sz = std::sin(r[2]);
    return {{{cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx},
             {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx},
             {-sy, cy * sx, cy * cx}}};
}

/// The angle [rad] of the rotation that takes orientation `a` to `b`, in [0, pi].
[[nodiscard]] inline double rotation_between(const Vec3& a, const Vec3& b) noexcept {
    const Matrix3 ra = rotation_matrix(a);
    const Matrix3 rb = rotation_matrix(b);
    double trace = 0.0;  // trace(ra^T * rb)
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t k = 0; k < 3; ++k) trace += ra[k][i] * rb[k][i];
    return std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
}

// ---- the controller's units

/// The pose as the controller writes one: x, y, z [mm], rx, ry, rz [rad].
[[nodiscard]] constexpr std::array<double, 6> to_mm_rad(const Pose& p) noexcept {
    return {p.position[0] * 1000.0, p.position[1] * 1000.0, p.position[2] * 1000.0,
            p.rotation[0],          p.rotation[1],          p.rotation[2]};
}

/// A pose the controller wrote: x, y, z [mm], rx, ry, rz [rad].
[[nodiscard]] constexpr Pose from_mm_rad(const std::array<double, 6>& p) noexcept {
    return {{p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0}, {p[3], p[4], p[5]}};
}

}  // namespace ec
