#pragma once
/// @file
/// Poses of frames as rigid transforms, Eigen::Isometry3d, in SI units (m, rad),
/// and their conversion to and from the controller's units.
///
/// Notation: T_ab is the pose of frame b in frame a. It maps coordinates given
/// in b to coordinates in a, p_a = T_ab * p_b. Its translation r_ab_a is where
/// b's origin is, along a's axes; its rotation q_ab turns vectors from b's axes
/// into a's, v_a = q_ab * v_b. Poses chain as T_ac = T_ab * T_bc, and
/// T_ba = T_ab.inverse(). Code spells the frames out: T_base_tcp is the pose of
/// the current tool's centre point in the robot's base frame.
///
/// The controller writes a pose as x, y, z [mm] and rx, ry, rz [rad], the
/// rotation being R = Rz(rz) * Ry(ry) * Rx(rx) (measured on the EA66, ADR 0012).
///
/// A motion of the TCP is a PoseTrajectory: poses stamped with their time since
/// the start, filled in by the application (docs/motion.md 5.11).

#include <Eigen/Geometry>
#include <array>
#include <expected>
#include <string>
#include <vector>

#include "ec/stamped.hpp"

namespace ec {

/// The rotation of the controller's angles [rad]: Rz(rz) * Ry(ry) * Rx(rx).
[[nodiscard]] Eigen::Matrix3d rotation_from_rxyz(double rx, double ry, double rz) noexcept;

/// The controller's angles of a rotation: rx and rz in [-pi, pi], ry in
/// [-pi/2, pi/2]. At ry = +-pi/2 only rx + rz or rx - rz is defined; rx is then 0.
[[nodiscard]] Eigen::Vector3d rxyz_from_rotation(const Eigen::Matrix3d& r) noexcept;

/// A pose from a position [m] and the controller's angles rx, ry, rz [rad].
[[nodiscard]] Eigen::Isometry3d isometry(const std::array<double, 3>& position,
                                         const std::array<double, 3>& rxyz) noexcept;

/// A pose the controller wrote: x, y, z [mm], rx, ry, rz [rad].
[[nodiscard]] Eigen::Isometry3d from_mm_rad(const std::array<double, 6>& pose) noexcept;

/// A pose as the controller writes one: x, y, z [mm], rx, ry, rz [rad].
[[nodiscard]] std::array<double, 6> to_mm_rad(const Eigen::Isometry3d& pose) noexcept;

/// The angle [rad] of the rotation that takes orientation `a` to `b`, in [0, pi].
[[nodiscard]] double angle_between(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) noexcept;

/// The pose a fraction `u` in [0, 1] of the way from `a` to `b`: the position on
/// the straight line between them, the orientation turned evenly (slerp, the
/// shorter way round).
[[nodiscard]] Eigen::Isometry3d interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double u) noexcept;

/// A motion of the TCP: poses T_base_tcp, each stamped with its time since the
/// start; the first at 0, then increasing. Between two poses the TCP goes in a
/// straight line at constant speed, its orientation turning evenly
/// (interpolate()); pose_at() and motion::plan() both read it so.
using PoseTrajectory = std::vector<Stamped<Eigen::Isometry3d, Clock::duration>>;

/// Why `trajectory` is not one, or nothing: fewer than 2 poses, a first stamp
/// other than 0, a stamp that does not increase, or a pose that is not a rigid
/// transform (a finite position; a rotation, orthonormal with determinant 1).
[[nodiscard]] std::expected<void, std::string> validate(const PoseTrajectory& trajectory);

/// The pose of a valid trajectory at time `t` since its start; its first or last
/// pose outside its times. Identity for an empty one.
[[nodiscard]] Eigen::Isometry3d pose_at(const PoseTrajectory& trajectory, Clock::duration t) noexcept;

}  // namespace ec
