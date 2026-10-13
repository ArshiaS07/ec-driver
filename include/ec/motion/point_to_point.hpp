#pragma once
/// @file
/// Planned point-to-point motion: one command, after which the controller plans
/// and times the move; done when 8056 shows the arm at rest at the target
/// (docs/motion.md 2.3). The specs are plain data in SI units; make_activity()
/// builds the activity that runs them.

#include <chrono>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include "ec/command/request.hpp"
#include "ec/motion/activity.hpp"
#include "ec/pose.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// Joint-space motion to joint positions (moveByJoint). The controller scales
/// `speed` by its speed setting (setSpeed; JBI manual 2.4).
struct MoveJoint {
    Joints target{};                            ///< [rad], within +-360 deg
    double speed = 0.1;                         ///< fraction of the joints' maximum speed, (0, 1]
    std::optional<double> acceleration{};       ///< fraction of the maximum, (0, 1]; controller default 0.8
    std::optional<double> deceleration{};       ///< fraction, (0, 1]; controller default: the acceleration
    double tolerance = deg_to_rad(0.2);         ///< [rad] on every joint, to count as arrived
    std::chrono::milliseconds timeout{60'000};  ///< from the command to arrival
};

/// Straight-line TCP motion to the pose of joint positions (moveByLine). The
/// speed is absolute (speed type AV): the speed setting does not scale it.
struct MoveLine {
    Joints target{};                       ///< [rad]: the TCP goes to these joints' pose
    double speed = 0.05;                   ///< TCP speed [m/s], [0.001, 3]
    std::optional<double> acceleration{};  ///< fraction, (0, 1]
    std::optional<double> deceleration{};  ///< fraction, (0, 1]
    double tolerance = deg_to_rad(0.2);    ///< [rad] on every joint
    std::chrono::milliseconds timeout{60'000};
};

/// Straight-line TCP motion to a pose in the base frame, for the current tool
/// (moveByLineCoord with the base frame as user frame). Absolute speed, as MoveLine.
struct MoveLineToPose {
    Pose target{};                                ///< position [m], rotation [rad] (R = Rz Ry Rx)
    double speed = 0.05;                          ///< TCP speed [m/s], [0.001, 3]
    std::optional<double> acceleration{};         ///< fraction, (0, 1]
    std::optional<double> deceleration{};         ///< fraction, (0, 1]
    double position_tolerance = 0.0005;           ///< [m]
    double rotation_tolerance = deg_to_rad(0.2);  ///< [rad]
    std::chrono::milliseconds timeout{60'000};
};

/// Runs one point-to-point command; see the file comment. Needs everything a
/// motion needs, and standstill to start. Its step(): sends the command, checks
/// the reply, then waits until the arm rests at the target; it fails if the arm
/// comes to rest elsewhere after moving, or on the time limit. Its close(): after
/// anything but arrival, sends stop and waits until the arm is at rest.
class PointToPoint {
public:
    /// Describes how far the arm is from the target, or nullopt once it is there.
    using Remaining = std::function<std::optional<std::string>(const feedback::State&)>;

    PointToPoint(std::string_view name, command::Request request, Remaining remaining,
                 std::chrono::milliseconds timeout);
    /// An activity that fails at its first step, without moving, with Kind::Invalid.
    static PointToPoint invalid(std::string_view name, std::string why);

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return name_; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    enum class Stage { Send, AwaitReply, Moving };

    std::string_view name_;
    command::Request request_;
    Remaining remaining_;
    std::chrono::milliseconds timeout_;
    std::optional<std::string> invalid_;
    Stage stage_ = Stage::Send;
    Clock::time_point sent_at_{};
    bool moved_ = false;
    int short_cycles_ = 0;
    std::optional<Clock::time_point> stop_sent_at_;
};

[[nodiscard]] PointToPoint make_activity(const MoveJoint& spec);
[[nodiscard]] PointToPoint make_activity(const MoveLine& spec);
[[nodiscard]] PointToPoint make_activity(const MoveLineToPose& spec);

static_assert(Spec<MoveJoint>);
static_assert(Spec<MoveLine>);
static_assert(Spec<MoveLineToPose>);

}  // namespace ec::motion
