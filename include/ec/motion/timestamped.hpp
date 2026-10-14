#pragma once
/// @file
/// The controller's timestamp service (SDK manual 2.2.9, ADR 0018): a joint
/// trajectory uploaded, then run on the controller's own clock. As the
/// manual's example (2.2.9.10): start_push_pos, 0.2 s, push_pos for every
/// point, each answered before the next, 1 s, stop_push_pos, check_trajectory;
/// then start_trajectory.

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/robot.hpp"
#include "ec/stamped.hpp"
#include "ec/units.hpp"

namespace ec::motion {

struct TimestampedOptions {
    TrajectoryLimits limits{};                 ///< the trajectory is checked against them before the upload
    double start_tolerance = deg_to_rad(0.5);  ///< [rad] on every joint: how close to the first point the arm must rest
};

/// One uploaded trajectory. It is the robot's open motion from the upload
/// until its run has ended (a wait says so) or the object is destroyed.
/// Move-only; a run goes on if the object is destroyed (stop() ends it).
class Timestamped {
public:
    /// Checks `trajectory` (validate()) and that the arm rests at its first
    /// point, then uploads it. Nothing moves. Invalid for a trajectory that
    /// fails the check or an arm elsewhere; Command if the controller refuses
    /// a point or the upload; Stopped if Robot::stop() came during the upload.
    [[nodiscard]] static std::expected<Timestamped, Error> upload(Robot& robot, const JointTrajectory& trajectory,
                                                                  const TimestampedOptions& options = {});

    /// start_trajectory: the arm moves, `speed` (0, 1] times the trajectory's own.
    [[nodiscard]] std::expected<void, Error> start(double speed = 1.0);
    [[nodiscard]] std::expected<void, Error> pause();   ///< pause_trajectory
    [[nodiscard]] std::expected<void, Error> resume();  ///< resume_trajectory

    /// Waits up to `timeout`: nullopt while the run goes on; nothing once the
    /// arm has stopped; else why (a fault, a stop).
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout);
    /// Waits until the run has ended, and says how.
    [[nodiscard]] std::expected<void, Error> wait();
    /// Ends the run at once (stop_trajectory).
    void stop() noexcept;

    Timestamped(Timestamped&& other) noexcept;
    Timestamped& operator=(Timestamped&& other) noexcept;
    Timestamped(const Timestamped&) = delete;
    Timestamped& operator=(const Timestamped&) = delete;
    ~Timestamped();

private:
    Timestamped(std::shared_ptr<detail::Connection> c, std::uint64_t id) : c_(std::move(c)), id_(id) {}

    std::shared_ptr<detail::Connection> c_;
    std::uint64_t id_ = 0;
    bool started_ = false;
    Clock::time_point started_at_{};
    bool ran_ = false;
    std::optional<std::expected<void, Error>> result_{};  // once the run has ended
};

}  // namespace ec::motion
