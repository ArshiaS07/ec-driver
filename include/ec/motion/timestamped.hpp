#pragma once
/// @file
/// The controller's timestamp service (SDK manual 2.2.9): a joint
/// trajectory uploaded, then run on the controller's own clock. upload() is the
/// manual's example (2.2.9.10): start_push_pos, 0.2 s, push_pos for every
/// point, each answered before the next, 1 s, stop_push_pos, check_trajectory.
/// start() is start_trajectory; pause(), resume() and stop() the calls of the
/// same names. The arm is to rest at the first point before start() (the
/// manual moves it there with moveByJoint).
///
///     ec::motion::Timestamped run(robot, trajectory);
///     run.upload();  // then run.start(), run.wait()

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/stamped.hpp"
#include "ec/trajectory.hpp"

namespace ec::motion {

/// One trajectory. It is the robot's open motion from its upload until a wait
/// has seen its run end. Move-only. Destroying it while it runs stops the run
/// (stop_trajectory): a run does not outlive the code that waits for it. Once
/// its Robot is gone, every call fails with Disconnected.
class Timestamped {
public:
    /// Sends nothing.
    Timestamped(Robot& robot, JointTrajectory trajectory);

    /// Uploads the trajectory; nothing moves. Invalid for fewer than 3 points,
    /// or stamps not from 0 increasing (the manual's ranges); Busy while a
    /// motion is open; Command if the controller refuses a call or finds the
    /// upload invalid; Stopped if robot.motion().stop() came during it.
    [[nodiscard]] std::expected<void, Error> upload();

    /// start_trajectory: the arm moves, at `speed` times the trajectory's own
    /// pace (speed_percent / 100). Invalid before an upload, and for a speed
    /// outside [0.001, 1]: the manual's floor, and the driver's ceiling, since
    /// above 1 the arm would move faster than the trajectory was planned for.
    [[nodiscard]] std::expected<void, Error> start(double speed = 1.0);
    [[nodiscard]] std::expected<void, Error> pause();   ///< pause_trajectory
    [[nodiscard]] std::expected<void, Error> resume();  ///< resume_trajectory

    /// Waits up to `timeout` for the run to end: nullopt while the arm still
    /// moves; nothing once it is at rest; Fault on an emergency stop, an alarm
    /// or a collision; Stopped after robot.motion().stop(). It reads the
    /// monitoring path while it streams, else asks getRobotState every 10 ms
    /// (the manual's wait_stop). Invalid before start().
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout);
    /// Waits until the run has ended, and says how.
    [[nodiscard]] std::expected<void, Error> wait();
    /// stop_trajectory: the run ends; a wait then sees it.
    void stop() noexcept;

    Timestamped(Timestamped&& other) noexcept;
    Timestamped& operator=(Timestamped&& other) noexcept;
    Timestamped(const Timestamped&) = delete;
    Timestamped& operator=(const Timestamped&) = delete;
    ~Timestamped();

private:
    void end() noexcept;  // a running trajectory is stopped; the registration given back

    std::weak_ptr<detail::Connection> session_;
    JointTrajectory trajectory_;
    std::uint64_t id_ = 0;  // the registration as the open motion, from the upload; 0: none
    bool started_ = false;
    Clock::time_point started_at_{};
    bool ran_ = false;
};

}  // namespace ec::motion
