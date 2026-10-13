#pragma once
/// @file
/// Paths of the TCP in the robot's base frame: geometry only, in SI units. A
/// path is anything with a length and a pose at every distance along it; timed
/// (ec::motion::sample), its poses become a planned path (ec::motion::plan).
/// Pure. Poses follow the notation of ec/pose.hpp.

#include <Eigen/Geometry>
#include <concepts>
#include <expected>
#include <string>

namespace ec {

/// A path of the TCP: `length(path)` [m], and `pose_at(path, s)`, the pose
/// T_base_tcp at distance s [m] along it, for s in [0, length]. Both are found
/// by argument-dependent lookup, so a new kind of path needs no change here.
template <typename P>
concept CartesianPath = requires(const P& path, double s) {
    { length(path) } -> std::convertible_to<double>;
    { pose_at(path, s) } -> std::convertible_to<Eigen::Isometry3d>;
};

/// A circle, or an arc of one, run at a constant tool orientation.
struct Circle {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();  ///< [m], in the base frame
    double radius = 0.0;                               ///< [m], > 0
    /// The axis, any length; the TCP turns counter-clockwise about it (right-hand rule).
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    /// From the centre toward the first point, any length; only its part in the
    /// circle's plane counts. When that part is zero: base x, or base y when the
    /// normal is within 30 degrees of x; projected onto the plane.
    Eigen::Vector3d start = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  ///< q_base_tcp, held all the way
    double turns = 1.0;                                               ///< > 0: 1 is a full circle, 0.5 a half
};

/// The circle's own axes, unit vectors in the base frame: `u` toward the first
/// point, `v` = n x u (the direction the TCP starts in), and the normal `n`.
struct CircleAxes {
    Eigen::Vector3d u = Eigen::Vector3d::UnitX();
    Eigen::Vector3d v = Eigen::Vector3d::UnitY();
    Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
};

/// Why `c` is not a circle (a radius, normal or turn count that is not
/// positive and finite, an orientation that is not a unit quaternion), or nothing.
[[nodiscard]] std::expected<void, std::string> validate(const Circle& c);

/// The axes of a valid circle.
[[nodiscard]] CircleAxes axes(const Circle& c) noexcept;

/// 2 pi radius turns [m].
[[nodiscard]] double length(const Circle& c) noexcept;

/// T_base_tcp at distance `s` [m] along a valid circle from its first point,
/// with `s` clamped to [0, length].
[[nodiscard]] Eigen::Isometry3d pose_at(const Circle& c, double s) noexcept;

/// A straight line from one pose to another, the orientation turning evenly
/// (slerp) along it. Its length is the distance between the two positions: a
/// pure turn has length 0, and is not a path.
struct Line {
    Eigen::Isometry3d from = Eigen::Isometry3d::Identity();  ///< T_base_tcp at the start
    Eigen::Isometry3d to = Eigen::Isometry3d::Identity();    ///< T_base_tcp at the end
};

/// The distance between the line's ends [m].
[[nodiscard]] double length(const Line& l) noexcept;

/// T_base_tcp at distance `s` [m] along the line, with `s` clamped to [0, length].
[[nodiscard]] Eigen::Isometry3d pose_at(const Line& l, double s) noexcept;

static_assert(CartesianPath<Circle>);
static_assert(CartesianPath<Line>);

}  // namespace ec
