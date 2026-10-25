#pragma once
/// @file
/// Velocity streaming (SDK manual 2.2.3.16-19): joint velocities
/// (moveBySpeedj) or a TCP velocity (moveBySpeedl), each valid for its
/// duration; the newest replaces the one before without slowing down, and
/// when the last runs out the arm stops. stop() is stopj or stopl. The
/// application sends the next command before the last runs out (a
/// controller loop, teleoperation). Remote mode only.
///
///     ec::motion::SpeedStream speed(robot);
///     speed.joints(qd, {.acceleration = 1.0, .valid_for = 100ms});   // every ~20 ms
///     speed.stop();

#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/stamped.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// How a velocity command is followed.
struct SpeedOptions {
    /// Joints: [rad/s^2] (acc, a whole number of deg/s^2, at least 1). TCP: the
    /// linear acceleration [m/s^2] (acc, a whole number of mm/s^2, at least 1).
    double acceleration = 0.5;
    /// TCP only: the rotational acceleration [rad/s^2] (arot, whole deg/s^2);
    /// the controller's default if absent.
    std::optional<double> rotational_acceleration{};
    /// How long the command holds (t, > 0).
    std::chrono::milliseconds valid_for{100};
};

/// One velocity stream, of joint or of TCP velocities. It is the robot's open
/// motion from the first command until stop(). Move-only. Destroying it while
/// open stops it (stopj or stopl). Once its Robot is gone, every call fails
/// with Disconnected.
class SpeedStream {
public:
    /// Sends nothing.
    explicit SpeedStream(Robot& robot);

    /// moveBySpeedj [S 2.2.3.16]: the joints at `velocity` [rad/s]. Invalid
    /// for options outside their ranges, or after tcp() in the same stream;
    /// Busy while another motion is open.
    [[nodiscard]] std::expected<void, Error> joints(const Joints& velocity, const SpeedOptions& options = {});
    /// moveBySpeedl [S 2.2.3.18]: the TCP at `linear` [m/s] and `angular`
    /// [rad/s], base frame. As joints().
    [[nodiscard]] std::expected<void, Error> tcp(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular,
                                                 const SpeedOptions& options = {});
    /// stopj or stopl [S 2.2.3.17, 2.2.3.19] at the last command's
    /// acceleration; the stream ends. (robot.motion().stop() sends it at the
    /// first command's.) Invalid if nothing was sent.
    [[nodiscard]] std::expected<void, Error> stop();

    SpeedStream(SpeedStream&& other) noexcept;
    SpeedStream& operator=(SpeedStream&& other) noexcept;
    SpeedStream(const SpeedStream&) = delete;
    SpeedStream& operator=(const SpeedStream&) = delete;
    ~SpeedStream();

    /// The family's stop call, in the controller's units.
    struct Ending {
        bool joints = true;            // stopj, else stopl
        int acc = 1;                   // deg/s^2 or mm/s^2
        std::optional<int> acc_rot{};  // deg/s^2, stopl only
    };

private:
    // Registers the stream at its first command; Invalid if it changes space.
    [[nodiscard]] std::expected<void, Error> begin(detail::Connection& c, const Ending& ending);
    void end() noexcept;

    std::weak_ptr<detail::Connection> session_;
    std::uint64_t id_ = 0;  // the registration as the open motion, from the first command; 0: none
    Ending last_{};         // how the last command would be stopped
};

}  // namespace ec::motion
