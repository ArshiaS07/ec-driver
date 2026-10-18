#pragma once
/// @file
/// ec::Robot: one controller (ADR 0018). It holds the command connection
/// (8055), on which every call is sent and answered one at a time, and, if
/// asked, the monitoring stream (8056), whose newest frame it keeps decoded.
/// Motions are opened on it, one at a time (ec/motion/move.hpp,
/// timestamped.hpp, servo_stream.hpp, move_along.hpp). Several arms are
/// several Robots.
///
///     auto robot = ec::Robot::connect({.host = "192.168.0.200"});
///     robot->bring_up();
///     auto mode = robot->call(ec::sdk::GetRobotMode{});   // any typed SDK call
///     auto f = robot->feedback();                             // the newest frame, in SI
///     ec::motion::move_along(*robot, poses);                  // a TCP motion, planned and run

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "ec/error.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/feedback/state.hpp"
#include "ec/sdk/request.hpp"
#include "ec/stamped.hpp"
#include "ec/units.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A monitoring frame, decoded.
struct Feedback {
    std::uint64_t seq = 0;        ///< one more for every frame
    Clock::time_point arrival{};  ///< when it arrived, on this PC's steady clock
    feedback::State state{};      ///< joints [rad], the TCP pose [m, rad], speeds, mode, state, ...
    feedback::Sample sample{};    ///< the frame as received, for recordings

    /// How long ago it arrived.
    [[nodiscard]] std::chrono::nanoseconds age() const { return Clock::now() - arrival; }
};

/// How to reach the controller.
struct RobotOptions {
    std::string host{};  ///< the controller's address; no default
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    bool monitor = true;  ///< receive the monitoring stream: feedback(), on_feedback
    std::chrono::milliseconds connect_timeout{2'000};
    /// Per call. Generous: the EA66 answers some actions slowly (set_servo_status).
    std::chrono::milliseconds reply_timeout{5'000};
    std::chrono::milliseconds first_feedback_within{
        2'000};  ///< with the monitor: connect() fails without a frame by then
    /// Called with every frame, on the receiver's thread: must be quick and must not throw.
    std::function<void(const Feedback&)> on_feedback{};
};

/// See the file comment. Move-only. Its member functions may be called from any
/// thread; calls wait for each other.
class Robot {
public:
    /// Connects the command port and, with the monitor, the monitoring port,
    /// waiting for the first frame. Disconnected if either cannot be reached.
    [[nodiscard]] static std::expected<Robot, Error> connect(RobotOptions options);

    Robot(Robot&&) noexcept;
    Robot& operator=(Robot&&) noexcept;
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    ~Robot();

    /// One typed SDK call, sent and answered. While a motion is open, only
    /// queries and computations: anything else is Busy.
    template <sdk::Command C>
    [[nodiscard]] std::expected<typename C::Reply, Error> call(const C& command) {
        auto result = call_json(sdk::to_request(command), C::kind);
        if (!result) return std::unexpected(std::move(result.error()));
        auto reply = sdk::decode_reply<C>(*result);
        if (!reply) return std::unexpected(from_call(C::method, reply.error()));
        return std::move(*reply);
    }

    /// The newest frame; nullopt without the monitor, or before a frame decoded.
    [[nodiscard]] std::optional<Feedback> feedback() const;
    /// The first frame newer than `after` (a Feedback::seq), waiting up to
    /// `timeout`: for loops paced by the robot's frames (about every 8 ms).
    [[nodiscard]] std::optional<Feedback> next_feedback(std::uint64_t after, std::chrono::nanoseconds timeout) const;

    /// The joint positions [rad] that put the current tool's TCP at T_base_tcp,
    /// the solution nearest `near` (the controller's inverseKinematic), each
    /// joint taken by whole turns to the value nearest it.
    [[nodiscard]] std::expected<Joints, Error> ik(const Eigen::Isometry3d& T_base_tcp, const Joints& near);
    /// T_base_tcp of the current tool for joint positions [rad] (positiveKinematic).
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> fk(const Joints& joints);
    /// The joint positions now [rad] (get_joint_pos).
    [[nodiscard]] std::expected<Joints, Error> joints();

    /// As the manual's examples: clears an alarm, synchronises the encoders,
    /// switches the servos on; only what the state needs. Returns once the
    /// servos report on (3 s at most).
    [[nodiscard]] std::expected<void, Error> bring_up();

    /// Stops: the open motion's own stop, then `stop`. From any thread; it waits
    /// for a call in flight to be answered first. The open motion's waits end
    /// with Stopped. Not a safety function: the E-stop is.
    void stop() noexcept;

    /// For the motion families: the connection they share with this robot.
    [[nodiscard]] std::shared_ptr<detail::Connection> share() const noexcept { return c_; }

private:
    explicit Robot(std::shared_ptr<detail::Connection> c) : c_(std::move(c)) {}
    [[nodiscard]] std::expected<sdk::Json, Error> call_json(const sdk::Request& request, sdk::Kind kind);

    std::shared_ptr<detail::Connection> c_;
};

}  // namespace ec
