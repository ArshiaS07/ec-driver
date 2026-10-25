#pragma once
/// @file
/// Servo streaming, the controller's transparent transmission (SDK manual
/// 2.2.7): joint targets into the controller's queue, which plays one
/// every `period` after `lookahead`. As the manual's example (2.2.7.6): open()
/// clears a stream left open, then transparent_transmission_init; add_point()
/// is tt_put_servo_joint_to_buf, each answered; close() is
/// tt_clear_servo_joint_buf. The application paces the points and makes them
/// smooth: the first where the arm rests (the manual moves it there with
/// moveByJoint), the next ones close to it. No watchdog yet.
///
///     ec::motion::ServoStream stream(robot, {.lookahead = 40ms, .period = 8ms});
///     stream.open();  // then stream.add_point(q) every period, stream.close()

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/units.hpp"

namespace ec::motion {

struct ServoOptions {
    std::chrono::milliseconds lookahead{40};  ///< the controller's queue delay, [10, 1000] ms
    std::chrono::milliseconds period{8};      ///< between points, [2, 100] ms
    double smoothness = 0.1;                  ///< the controller's smoothing gain, [0, 1]
};

/// One stream. It is the robot's open motion from open() until close(), or
/// the object's destruction, which closes it as close() does. Move-only. Once
/// its Robot is gone, every call fails with Disconnected.
class ServoStream {
public:
    /// Sends nothing.
    explicit ServoStream(Robot& robot, const ServoOptions& options = {});

    /// Opens the stream. Invalid for options outside the manual's ranges; Busy
    /// while a motion is open.
    [[nodiscard]] std::expected<void, Error> open();
    /// tt_put_servo_joint_to_buf: the next point [rad]. Invalid outside
    /// [-360, 360] deg, or before open().
    [[nodiscard]] std::expected<void, Error> add_point(const Joints& q);
    /// tt_clear_servo_joint_buf: the stream ends; the arm stops where the queue
    /// has brought it.
    [[nodiscard]] std::expected<void, Error> close();
    /// tt_clear_servo_joint_buf, at once.
    void stop() noexcept;

    ServoStream(ServoStream&& other) noexcept;
    ServoStream& operator=(ServoStream&& other) noexcept;
    ServoStream(const ServoStream&) = delete;
    ServoStream& operator=(const ServoStream&) = delete;
    ~ServoStream();

private:
    void close_quietly() noexcept;  // close(), for the destructor and the move assignment

    std::weak_ptr<detail::Connection> session_;
    ServoOptions options_;
    std::uint64_t id_ = 0;  // the registration as the open motion, while open; 0: none
};

}  // namespace ec::motion
