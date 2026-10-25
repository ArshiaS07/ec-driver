#pragma once
/// @file
/// Jogging (SDK manual 2.2.3.8): one axis, or one direction of a coordinate
/// system, moved for as long as the command is repeated. As the manual's
/// example: setCurrentCoord, then jog every 0.1 s or so, then stop. The arm
/// keeps moving until 1 s after the last jog, or until stop. The application
/// repeats step() while it wants motion (a held button, a joystick).
///
///     ec::motion::Jog jog(robot, {.frame = ec::sdk::CoordinateSystem::Base, .speed = 0.1});
///     while (held()) { jog.step(3, ec::motion::Direction::Minus); sleep(100ms); }   // -z
///     jog.stop();

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/param.hpp"
#include "ec/stamped.hpp"

namespace ec::motion {

enum class Direction : std::uint8_t { Minus, Plus };

struct JogOptions {
    /// coord_mode: Joint (axes J1..J6) or Base, Tool, User, Cylindrical (x, y,
    /// z, rx, ry, rz in that system).
    sdk::CoordinateSystem frame = sdk::CoordinateSystem::Joint;
    /// A fraction of the manual jog speed, [0.0005, 1] (speed, 0.05..100 %).
    double speed = 0.1;
};

/// One jog session. It is the robot's open motion from the first step() until
/// stop(), or until a wait has seen the arm stop after the steps ended.
/// Move-only. Destroying it while open stops the arm (stop). Once its Robot is
/// gone, every call fails with Disconnected.
class Jog {
public:
    /// Sends nothing.
    explicit Jog(Robot& robot, const JogOptions& options = {});

    /// jog along `axis` (1..6: J1..J6, or x, y, z, rx, ry, rz) in `direction`;
    /// the first step selects the frame first (setCurrentCoord). Invalid for an
    /// axis outside 1..6 or a speed outside its range (nothing sent); Busy
    /// while another motion is open.
    [[nodiscard]] std::expected<void, Error> step(int axis, Direction direction);
    /// Waits up to `timeout` for the arm to stop once the steps have ended, as
    /// Move::wait_for. Invalid before a step.
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout);
    /// stop [S 2.2.3.9]: the arm stops at once; the session ends.
    [[nodiscard]] std::expected<void, Error> stop();

    Jog(Jog&& other) noexcept;
    Jog& operator=(Jog&& other) noexcept;
    Jog(const Jog&) = delete;
    Jog& operator=(const Jog&) = delete;
    ~Jog();

private:
    void end() noexcept;  // an open session is stopped and unregistered

    std::weak_ptr<detail::Connection> session_;
    JogOptions options_;
    std::uint64_t id_ = 0;  // the registration as the open motion, from the first step; 0: none
    Clock::time_point last_step_{};
    bool ran_ = false;
};

}  // namespace ec::motion
