#pragma once
/// @file
/// Paths of the TCP in the base frame: geometry only, in SI units. A path is
/// anything with a length and a pose at every distance along it; the planner
/// (ec/motion/planning.hpp) times it and solves it into joint positions. Pure.

#include <concepts>
#include <expected>
#include <string>

#include "ec/pose.hpp"

namespace ec {

/// A path of the TCP: `length(path)` [m], and `pose_at(path, s)`, the pose at
/// distance s [m] along it, for s in [0, length]. Both are found by
/// argument-dependent lookup, so a new kind of path needs no change here.
template <typename P>
concept CartesianPath = requires(const P& path, double s) {
    { length(path) } -> std::convertible_to<double>;
    { pose_at(path, s) } -> std::convertible_to<Pose>;
};

/// A circle, or an arc of one, run at a constant tool orientation.
struct Circle {
    Vec3 center{};               ///< [m], in the base frame
    double radius = 0.0;         ///< [m], > 0
    Vec3 normal{0.0, 0.0, 1.0};  ///< the axis, any length; the TCP turns counter-clockwise about it (right-hand rule)
    /// From the centre toward the first point, any length; only its part in the
    /// circle's plane counts. When that part is zero: base x, or base y when the
    /// normal is within 30 degrees of x; projected onto the plane.
    Vec3 start{};
    Vec3 rotation{};     ///< the tool orientation rx, ry, rz [rad], held all the way
    double turns = 1.0;  ///< > 0: 1 is a full circle, 0.5 a half
};

/// The circle's own axes: `u` toward the first point, `v` = n x u (the
/// direction the TCP starts in), and the unit normal `n`.
struct CircleAxes {
    Vec3 u{};
    Vec3 v{};
    Vec3 n{};
};

/// Why `c` is not a circle (a radius, normal or turn count that is not
/// positive and finite), or nothing.
[[nodiscard]] std::expected<void, std::string> validate(const Circle& c);

/// The axes of a valid circle.
[[nodiscard]] CircleAxes axes(const Circle& c) noexcept;

/// 2 pi radius turns [m].
[[nodiscard]] double length(const Circle& c) noexcept;

/// The pose at distance `s` [m] along a valid circle from its first point,
/// with `s` clamped to [0, length].
[[nodiscard]] Pose pose_at(const Circle& c, double s) noexcept;

static_assert(CartesianPath<Circle>);

}  // namespace ec
