#pragma once
/// @file
/// ec::motion::Controller: the motion controller, the part of the controller
/// that plans, interpolates and times every motion (docs/ec66.md §5.5, §5.7).
/// The families in this directory are its kinds of motion (Move, Path, Jog,
/// SpeedStream, ServoStream, Timestamped), one open at a time; this part is
/// what they share: the state, the wait for rest, the stop, the speed
/// slider, hand guiding.
///
///     auto motion = robot.motion();
///     motion.stop();                                   // from any thread
///     motion.wait_until_stopped(5s);

#include <chrono>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/status.hpp"

namespace ec {
class Robot;
namespace detail {
struct Connection;
}
}  // namespace ec

namespace ec::motion {

/// A handle onto a robot's motion controller: cheap to copy, usable from any
/// thread. Once its Robot is gone, every operation fails with Disconnected
/// (stop() then does nothing).
class Controller {
public:
    /// getRobotState [S 2.2.2.1] (also on the monitoring path: motion.state).
    [[nodiscard]] std::expected<RobotState, Error> state() const;
    /// Waits up to `timeout` for the arm to be at rest (state stop): nothing
    /// then; nullopt while it still moves; Fault on an emergency stop, an alarm
    /// or a collision. It reads the monitoring path while it streams, and
    /// otherwise asks getRobotState every 10 ms (the manual's wait_stop).
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_until_stopped(std::chrono::nanoseconds timeout) const;
    /// Stops whatever moves the arm: the open motion's own stop, then stop
    /// [S 2.2.3.9] (which also ends a program). From any thread; it waits for a
    /// call in flight to be answered first. The open motion's waits end with
    /// Stopped. Not a safety function: the emergency stop is.
    void stop() const noexcept;

    /// getSpeed [S 2.2.2.34]: the speed slider for REMOTE and PLAY, as a
    /// fraction. It scales the families' speeds (docs/ec66.md §5.5).
    [[nodiscard]] std::expected<double, Error> speed_scale() const;
    /// setSpeed [S 2.2.3.15]: [0.0005, 1] (0.05..100 %), else Invalid. Busy
    /// while a motion is open.
    [[nodiscard]] std::expected<void, Error> set_speed_scale(double fraction) const;

    /// get_drag_state [S 2.2.2.18]: hand guiding is on (also on the
    /// monitoring path: motion.hand_guiding).
    [[nodiscard]] std::expected<bool, Error> hand_guiding() const;
    /// drag_teach_switch [S 2.2.2.17]: on, the operator moves the arm by hand
    /// (REMOTE, servo on); off, the controller holds it again. Busy while a
    /// motion is open.
    [[nodiscard]] std::expected<void, Error> set_hand_guiding(bool on) const;

private:
    friend class ec::Robot;
    explicit Controller(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec::motion
