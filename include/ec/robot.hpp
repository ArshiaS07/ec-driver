#pragma once
/// @file
/// ec::Robot: the one object applications use (docs/motion.md 5). It composes
/// the parts below it: a feedback::Receiver for the monitoring stream, the
/// newest state in an ec::Latest, and a motion::ControlLoop that owns the
/// command connection and runs one activity at a time.
///
/// Motion is in joint space; TCP poses become joints through the controller's
/// inverse kinematics (ADR 0017):
///
///     Joints q = robot.ik(T_base_tcp, near).value();         // the controller's IK
///     robot.move_joints({.target = q, .speed = 0.2}).wait();  // moveByJoint
///     auto trajectory = robot.plan(timed_poses).value();      // IK, checks, a joint trajectory
///     robot.play(trajectory).wait();                          // the timestamp service
///     auto servo = robot.servo().value();                     // live: push joints
///     servo.target(robot.ik(T_base_tcp, q).value());

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>

#include "ec/command/client.hpp"
#include "ec/command/request.hpp"
#include "ec/feedback/receiver.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/feedback/state.hpp"
#include "ec/motion/activity.hpp"
#include "ec/motion/bring_up.hpp"
#include "ec/motion/control_loop.hpp"
#include "ec/motion/error.hpp"
#include "ec/motion/planning.hpp"
#include "ec/motion/point_to_point.hpp"
#include "ec/motion/session.hpp"
#include "ec/motion/stream.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/pose.hpp"
#include "ec/units.hpp"

namespace ec {

/// How to reach the controller, and the settings of the parts.
struct RobotOptions {
    std::string host{};  ///< the controller's address; no default
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    std::chrono::milliseconds connect_timeout{2'000};
    std::chrono::milliseconds first_state_within{2'000};  ///< connect() fails without a frame by then
    feedback::Receiver::Options feedback{};               ///< its port is monitor_port
    motion::ControlOptions control{};
    /// Also called with every monitoring sample, on the receiver's thread: for
    /// recording while the arm moves (e.g. into a log::RawWriter). Must be quick
    /// and must not throw.
    std::function<void(const feedback::Sample&)> on_sample{};
};

/// The newest controller state, in SI.
struct Snapshot {
    std::uint64_t seq = 0;  ///< one more for every frame received
    feedback::State state{};
    std::chrono::steady_clock::time_point arrival{};

    /// How long ago it arrived.
    [[nodiscard]] std::chrono::nanoseconds age() const { return std::chrono::steady_clock::now() - arrival; }
};

/// See the file comment. Move-only. Destroying it stops whatever runs (as
/// stop() does, waiting for the arm to rest), then closes both connections.
/// Every member function may be called from any thread.
class Robot {
public:
    /// Connects both ports, waits for the first monitoring frame, and starts the
    /// control loop, returning after its first cycle (so status() is current).
    /// Fails with Kind::Disconnected if either port cannot be reached.
    [[nodiscard]] static std::expected<Robot, motion::Error> connect(RobotOptions options);

    Robot(Robot&&) noexcept;
    Robot& operator=(Robot&&) noexcept;
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    ~Robot();

    // ---- state

    /// The newest state; nullopt only if no frame has decoded yet.
    [[nodiscard]] std::optional<Snapshot> state() const;
    /// The first state newer than `seq`, waiting up to `timeout`: one per frame,
    /// for loops that run at the robot's rate (docs/motion.md 6.2 b).
    [[nodiscard]] std::optional<Snapshot> next_state(std::uint64_t seq, std::chrono::nanoseconds timeout) const;
    /// What the control loop reports: idle, running, closing or fault, the
    /// latched fault, the conditions.
    [[nodiscard]] motion::Status status() const;
    /// The monitoring connection's own status.
    [[nodiscard]] feedback::Receiver::Status feedback_status() const;

    // ---- kinematics, by the controller (computations: allowed at any time)

    /// The joint positions [rad] that put the current tool's TCP at T_base_tcp,
    /// the solution nearest `near` (inverseKinematic), each joint taken by whole
    /// turns to the value nearest it. A pose without a solution is Kind::Command.
    [[nodiscard]] std::expected<Joints, motion::Error> ik(const Eigen::Isometry3d& T_base_tcp, const Joints& near);
    /// T_base_tcp, the current tool's TCP, for joint positions (positiveKinematic).
    [[nodiscard]] std::expected<Eigen::Isometry3d, motion::Error> fk(const Joints& joints);

    // ---- planned motion and procedures

    /// Moves in joint space to joint positions (moveByJoint).
    [[nodiscard]] motion::Job move_joints(const motion::MoveJoint& move);
    /// A timed TCP path solved into a joint trajectory from where the arm is now:
    /// motion::plan() with ik(). Computations only: nothing moves.
    [[nodiscard]] std::expected<motion::JointTrajectory, motion::Error> plan(std::span<const motion::TimedPose> path,
                                                                             const motion::PlanOptions& options = {});
    /// Plays a joint trajectory through the controller's timestamp service
    /// (motion::PlayTrajectory), checked against `limits` (pass the ones plan()
    /// used). The arm must rest at its first point.
    [[nodiscard]] motion::Job play(motion::JointTrajectory trajectory, double speed = 1.0,
                                   const motion::TrajectoryLimits& limits = {});
    /// Starts any activity or spec (MoveJoint, Sequence, PlayTrajectory, ...).
    [[nodiscard]] motion::Job run(motion::AnyActivity activity);
    /// Clear alarm, reset collision, synchronise, servos on: what the state needs.
    [[nodiscard]] motion::Job bring_up(const motion::BringUp& spec = {});

    // ---- streamed motion: sessions to push targets into

    /// Streams joint positions [rad] (transparent transmission). Waits until the
    /// controller has accepted the stream, or why it did not.
    [[nodiscard]] std::expected<motion::Session<Joints>, motion::Error> servo(const motion::ServoOptions& options = {});
    /// Streams joint velocities [rad/s] (moveBySpeedj).
    [[nodiscard]] std::expected<motion::Session<Joints>, motion::Error> velocity(
        const motion::VelocityOptions& options = {});

    // ---- stop and recovery

    /// Ends whatever runs as fast as its family allows (within one cycle);
    /// when nothing runs, sends a plain stop. Not a safety function.
    void stop() noexcept;
    /// Clears a latched fault, after a person has dealt with its cause; then
    /// bring_up() if the state needs it.
    [[nodiscard]] std::expected<void, motion::Error> acknowledge();

    // ---- one-off calls

    /// One SDK call, sent by the control loop: queries and computations at any
    /// time, settings and actions only while nothing runs, motion commands never.
    template <command::Command C>
    [[nodiscard]] std::expected<typename C::Reply, motion::Error> call(const C& command) {
        auto result = call_json(command::to_request(command), C::kind);
        if (!result) return std::unexpected(std::move(result.error()));
        auto reply = command::decode_reply<C>(*result);
        if (!reply) return std::unexpected(motion::command_failed(C::method, reply.error()));
        return std::move(*reply);
    }

private:
    struct Impl;
    explicit Robot(std::unique_ptr<Impl> impl);
    [[nodiscard]] std::expected<command::Json, motion::Error> call_json(command::Request request, command::Kind kind);
    template <typename T>
    [[nodiscard]] std::expected<motion::Session<T>, motion::Error> open(std::shared_ptr<motion::Channel<T>> channel,
                                                                        motion::Job job);

    std::unique_ptr<Impl> impl_;
};

}  // namespace ec
