#pragma once
/// @file
/// SI types and the conversions to and from the controller's units (deg, mm),
/// for the one boundary where commands are built (ADR 0014).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace ec {

inline constexpr std::size_t kJointCount = 6;

/// One value per joint, J1..J6, in SI units (rad, rad/s, rad/s^2, ...).
using Joints = std::array<double, kJointCount>;

[[nodiscard]] constexpr double deg_to_rad(double deg) noexcept { return deg * (std::numbers::pi / 180.0); }
[[nodiscard]] constexpr double rad_to_deg(double rad) noexcept { return rad * (180.0 / std::numbers::pi); }

/// Element-wise deg_to_rad.
template <std::size_t N>
[[nodiscard]] constexpr std::array<double, N> deg_to_rad(const std::array<double, N>& deg) noexcept {
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = deg_to_rad(deg[i]);
    return out;
}

/// Element-wise rad_to_deg.
template <std::size_t N>
[[nodiscard]] constexpr std::array<double, N> rad_to_deg(const std::array<double, N>& rad) noexcept {
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = rad_to_deg(rad[i]);
    return out;
}

/// The largest absolute element-wise difference: how far apart two joint
/// vectors are on their worst joint.
[[nodiscard]] inline double max_abs_diff(const Joints& a, const Joints& b) noexcept {
    double worst = 0.0;
    for (std::size_t i = 0; i < kJointCount; ++i) worst = std::max(worst, std::abs(a[i] - b[i]));
    return worst;
}

/// Every element times `k`.
[[nodiscard]] constexpr Joints scale(const Joints& v, double k) noexcept {
    Joints out{};
    for (std::size_t i = 0; i < kJointCount; ++i) out[i] = v[i] * k;
    return out;
}

/// The largest absolute element.
[[nodiscard]] inline double max_abs(const Joints& v) noexcept {
    double worst = 0.0;
    for (double x : v) worst = std::max(worst, std::abs(x));
    return worst;
}

/// `q` with each joint moved by whole turns (2 pi) to lie nearest `reference`,
/// unless that would take it beyond +-`limit`: the arm in the same pose. For
/// joint positions given within one turn, e.g. inverse-kinematics solutions.
[[nodiscard]] inline Joints nearest_turn(Joints q, const Joints& reference, double limit) noexcept {
    constexpr double turn = 2.0 * std::numbers::pi;
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const double moved = q[i] + std::round((reference[i] - q[i]) / turn) * turn;
        if (std::abs(moved) <= limit) q[i] = moved;
    }
    return q;
}

/// The EC66's joint speed limits from its datasheet [rad/s]: J1-J2 150 deg/s,
/// J3 190 deg/s, J4-J6 260 deg/s. Every joint ranges over +-360 deg.
inline constexpr Joints kEc66MaxJointSpeed{deg_to_rad(150.0), deg_to_rad(150.0), deg_to_rad(190.0),
                                           deg_to_rad(260.0), deg_to_rad(260.0), deg_to_rad(260.0)};
inline constexpr double kEc66JointLimit = deg_to_rad(360.0);

}  // namespace ec
