#pragma once
/// @file
/// ec::Model: the controller's model of the arm: its kinematics, the tools
/// (TCP and payload) and the user frames (SDK manual 2.2.2, 2.2.4;
/// docs/ec66.md §5.6). The controller owns the model and computes with it;
/// each operation is one call on the command connection (8055), converted to
/// SI. Poses are Eigen::Isometry3d as in ec/pose.hpp (T_ab: the pose of b in a).
///
///     auto model = robot.model();
///     auto q = model.ik(T_base_tcp, robot.monitor().latest()->positions());
///     auto T = model.fk(*q);

#include <Eigen/Geometry>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/units.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A tool's payload [S 2.2.2.19].
struct Payload {
    double mass = 0.0;  ///< kg; EA66: 0..7.2
    /// Its centre of mass [m], each coordinate within -5..5 m; the manual
    /// names no frame (the flange's, inferred).
    Eigen::Vector3d centre_of_mass = Eigen::Vector3d::Zero();
};

/// A handle onto a robot's model: cheap to copy, usable from any thread (calls
/// wait for each other). Once its Robot is gone, every operation fails with
/// Disconnected. The computations and readings work while a motion is open;
/// the settings are then Busy. Tool and user frame numbers are 0..7, else
/// Invalid with nothing sent; the settings need REMOTE mode.
class Model {
public:
    /// inverseKinematic [S 2.2.4.1]: joint positions [rad] that put the TCP of
    /// the current tool at T_base_tcp; the solution nearest `near`, if given.
    /// One solution, the controller's; none is a Command error.
    [[nodiscard]] std::expected<Joints, Error> ik(const Eigen::Isometry3d& T_base_tcp,
                                                  const std::optional<Joints>& near = std::nullopt) const;
    /// positiveKinematic [S 2.2.4.2]: T_base_tcp at joint positions [rad].
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> fk(const Joints& q) const;
    /// convertPoseFromCartToUser [S 2.2.4.3]: T_base_x as T_user_x, in user frame `user`.
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> to_user(const Eigen::Isometry3d& T_base_x, int user) const;
    /// convertPoseFromUserToCart [S 2.2.4.4]: T_user_x, in user frame `user`, as T_base_x.
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> from_user(const Eigen::Isometry3d& T_user_x, int user) const;

    /// getAutoRunToolNumber [S 2.2.2.36]: the tool REMOTE and PLAY use. (TEACH
    /// has its own, getToolNumber.)
    [[nodiscard]] std::expected<int, Error> active_tool() const;
    /// setAutoRunToolNumber [S 2.2.2.37].
    [[nodiscard]] std::expected<void, Error> select_tool(int tool) const;
    /// getTcpPos [S 2.2.2.25]: the tool's centre point, T_flange_tcp.
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> tool(int tool) const;
    /// cmd_set_tcp [S 2.2.2.20]: its position within -0.5..0.5 m on each axis.
    [[nodiscard]] std::expected<void, Error> set_tool(int tool, const Eigen::Isometry3d& T_flange_tcp) const;
    /// get_tool_payload [S 2.2.2.62]; the reply's shape is unverified on the EA66.
    [[nodiscard]] std::expected<Payload, Error> payload(int tool) const;
    /// cmd_set_payload [S 2.2.2.19]. A wrong payload spoils gravity
    /// compensation, hand guiding and collision detection (docs/ec66.md §5.6).
    [[nodiscard]] std::expected<void, Error> set_payload(int tool, const Payload& payload) const;

    /// getUserNumber [S 2.2.2.12]: the current user frame.
    [[nodiscard]] std::expected<int, Error> active_user_frame() const;
    /// setUserNumber [S 2.2.2.13].
    [[nodiscard]] std::expected<void, Error> select_user_frame(int user) const;
    /// getUserFrame [S 2.2.2.22]: T_base_user.
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> user_frame(int user) const;
    /// setUserFrame [S 2.2.2.24].
    [[nodiscard]] std::expected<void, Error> set_user_frame(int user, const Eigen::Isometry3d& T_base_user) const;

private:
    friend class Robot;
    explicit Model(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec
