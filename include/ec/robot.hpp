#pragma once
/// @file
/// ec::Robot: one controller, reached on two paths. The command connection
/// (8055), on which every call is sent and answered one at a time; and, if
/// asked, the monitoring stream (8056), which the controller pushes about
/// every 8 ms and the robot's Monitor turns into ec::State (monitor()).
/// Motions are opened on it, one at a time (ec/motion/move.hpp,
/// timestamped.hpp, servo_stream.hpp). Several arms are several Robots.
///
///     auto robot = ec::Robot::connect({.host = "192.168.0.200"});
///     robot->bring_up();
///     auto mode = robot->call(ec::sdk::GetRobotMode{});  // 8055: asks the controller now
///     auto s = robot->monitor().latest();                 // 8056: the newest state, sends nothing

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ec/error.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/monitor.hpp"
#include "ec/sdk/request.hpp"
#include "ec/state.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// How to reach the controller.
struct RobotOptions {
    std::string host{};  ///< the controller's address; no default
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    bool monitor = true;  ///< receive the monitoring stream: monitor()
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

    /// The monitoring path: the newest State, the next one, the stream's
    /// status. Without the monitor (RobotOptions::monitor), it receives
    /// nothing. Valid as long as this robot (or the one it is moved into) lives.
    [[nodiscard]] const Monitor& monitor() const noexcept;

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
