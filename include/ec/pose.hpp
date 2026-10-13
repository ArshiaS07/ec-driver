#pragma once
/// @file
/// A Cartesian pose in SI units, with the controller's rotation convention:
/// R = Rz(rz) * Ry(ry) * Rx(rx), measured on the EA66 (ADR 0012).

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

/// The distance [m] between two positions.
[[nodiscard]] inline double distance(const Vec3& a, const Vec3& b) noexcept {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

}  // namespace ec
