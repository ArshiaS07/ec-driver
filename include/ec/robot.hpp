#pragma once
/// @file
/// ec::Robot: one controller, reached on two paths. The command connection
/// (8055), on which every call is sent and answered one at a time: the typed
/// calls (call()) and the parts that group them (joint(i), drives(),
/// model(), safety(), io(), programs(), system()). And, if asked, the monitoring stream (8056), which the
/// controller pushes about every 8 ms and the robot's Monitor turns into
/// ec::State (monitor()). Motions are opened on it, one at a time
/// (ec/motion/move.hpp, timestamped.hpp, servo_stream.hpp). Several arms are
/// several Robots.
///
///     auto robot = ec::Robot::connect({.host = "192.168.0.200"});
///     robot->bring_up();
///     auto on = robot->drives().servo_on();               // 8055: asks the controller now
///     auto mode = robot->call(ec::sdk::GetRobotMode{});  // 8055: any typed call
///     auto s = robot->monitor().latest();                 // 8056: the newest state, sends nothing

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ec/drives.hpp"
#include "ec/error.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/io.hpp"
#include "ec/joint.hpp"
#include "ec/model.hpp"
#include "ec/monitor.hpp"
#include "ec/motion/controller.hpp"
#include "ec/programs.hpp"
#include "ec/safety.hpp"
#include "ec/sdk/request.hpp"
#include "ec/state.hpp"
#include "ec/system.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// How to reach the controller.
struct RobotOptions {
    std::string host{};  ///< the controller's address; no default
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    std::uint16_t log_port = 8058;  ///< read by system().log(), one connection per read
    bool monitor = true;            ///< receive the monitoring stream: monitor()
    std::chrono::milliseconds connect_timeout{2'000};
    /// Per call. Generous: the EA66 answers some actions slowly (set_servo_status).
    std::chrono::milliseconds reply_timeout{5'000};
    /// With the monitor: connect() fails without a state by then.
    std::chrono::milliseconds first_state_within{2'000};
    /// With the monitor, on its thread (quick, no exceptions; Monitor::Options):
    /// every frame as received, for recordings; every state.
    std::function<void(const feedback::Sample&)> on_sample{};
    std::function<void(const State&)> on_state{};
};

/// See the file comment. Move-only; it owns the session. Destroying it (or
/// assigning another to it) stops a motion still open, as motion().stop(), and
/// closes the connections; it does not switch the servos off (shut_down does).
/// Its member functions may be called from any thread; calls wait for each other.
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

    /// The parts: handles onto this robot's command connection, cheap to copy;
    /// once the robot is gone their operations fail with Disconnected.
    [[nodiscard]] Joint joint(int number) const noexcept { return Joint(c_, number); }  ///< 1..6
    [[nodiscard]] Drives drives() const noexcept { return Drives(c_); }
    [[nodiscard]] Model model() const noexcept { return Model(c_); }
    [[nodiscard]] Safety safety() const noexcept { return Safety(c_); }
    [[nodiscard]] Io io() const noexcept { return Io(c_); }
    [[nodiscard]] Programs programs() const noexcept { return Programs(c_); }
    [[nodiscard]] System system() const noexcept { return System(c_); }

    /// The monitoring path: the newest State, the next one, the stream's
    /// status. Without the monitor (RobotOptions::monitor), it receives
    /// nothing. Valid as long as this robot (or the one it is moved into) lives.
    [[nodiscard]] const Monitor& monitor() const noexcept;

    /// As the manual's examples: clears an alarm, synchronises the encoders,
    /// switches the servos on; only what the state needs. Returns once the
    /// servos report on (3 s at most).
    [[nodiscard]] std::expected<void, Error> bring_up();

    /// The motion controller: the state, the wait for rest, the stop (from any
    /// thread), the speed slider, hand guiding. Its kinds of motion are the
    /// families in ec/motion/, each made on this robot (ec::motion::Move m(robot)).
    [[nodiscard]] motion::Controller motion() const noexcept { return motion::Controller(c_); }

    /// For the motion families: this robot's session, without keeping it.
    [[nodiscard]] std::weak_ptr<detail::Connection> session() const noexcept { return c_; }

private:
    explicit Robot(std::shared_ptr<detail::Connection> c) : c_(std::move(c)) {}
    [[nodiscard]] std::expected<sdk::Json, Error> call_json(const sdk::Request& request, sdk::Kind kind);
    void end() noexcept;  // the session ends: an open motion is stopped

    std::shared_ptr<detail::Connection> c_;
};

}  // namespace ec
