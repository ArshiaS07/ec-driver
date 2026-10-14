#pragma once
/// @file
/// Servo streaming, the controller's transparent transmission (SDK manual
/// 2.2.7, ADR 0018): joint targets into the controller's queue, which plays one
/// every `period` after `lookahead`. As the manual's example (2.2.7.6): a
/// stream left open is cleared first, transparent_transmission_init, then
/// points from the application's loop, one every period, each answered; at the
/// end tt_clear_servo_joint_buf. The application paces the points and makes
/// them smooth: the first at where the arm rests, the next ones close to it.
/// No watchdog yet (ADR 0018, 4).

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/units.hpp"

namespace ec::motion {

struct ServoOptions {
    std::chrono::milliseconds lookahead{40};  ///< the controller's queue delay, [10, 1000] ms, longer than the period
    std::chrono::milliseconds period{8};      ///< between points, [2, 100] ms
    double smoothness = 0.1;                  ///< the controller's smoothing gain, [0, 1]
};

/// An open stream. It is the robot's open motion until close() or the
/// object's destruction (which closes it). Move-only.
class ServoStream {
public:
    /// Opens the stream. Invalid for options out of range; Busy while another
    /// motion is open.
    [[nodiscard]] static std::expected<ServoStream, Error> open(Robot& robot, const ServoOptions& options = {});

    /// tt_put_servo_joint_to_buf: the next point [rad]. Invalid beyond the
    /// joint limits.
    [[nodiscard]] std::expected<void, Error> add_point(const Joints& q);
    /// tt_clear_servo_joint_buf: the stream ends; the arm stops where the
    /// queue has brought it.
    [[nodiscard]] std::expected<void, Error> close();
    /// Clears the queue at once (tt_clear_servo_joint_buf).
    void stop() noexcept;

    ServoStream(ServoStream&& other) noexcept;
    ServoStream& operator=(ServoStream&& other) noexcept;
    ServoStream(const ServoStream&) = delete;
    ServoStream& operator=(const ServoStream&) = delete;
    ~ServoStream();

private:
    ServoStream(std::shared_ptr<detail::Connection> c, std::uint64_t id) : c_(std::move(c)), id_(id) {}
    void close_quietly() noexcept;  // close(), for the destructor and the move assignment

    std::shared_ptr<detail::Connection> c_;
    std::uint64_t id_ = 0;
};

}  // namespace ec::motion
