#pragma once
/// @file
/// Waypoint paths (SDK manual 2.2.3.5-7, "2.0"): the controller runs a list
/// of waypoints, each reached by its own kind of move (joint, line, rotation
/// about the tool, arc), blending between them. run() makes the manual's
/// calls in its order: clearPathPoint, addPathPoint for each waypoint (each
/// answered), moveByPath; index() is getPathPointIndex; wait() waits for the
/// end. Remote mode only; the arm at rest.
///
///     using ec::sdk::PathMoveType;
///     ec::motion::Path path(robot, {{.target = q1}, {.target = q2, .move = PathMoveType::Line, .speed = 0.1}});
///     path.run();  // then path.wait()

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <vector>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/stamped.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// One waypoint (addPathPoint).
struct Waypoint {
    Joints target{};                                    ///< wayPoint [rad], within [-360, 360] deg
    sdk::PathMoveType move = sdk::PathMoveType::Joint;  ///< moveType: how it is reached
    /// For Joint, a fraction of the joints' maximum speed, [0.01, 1]; otherwise
    /// as speed_type says: Linear [0.001, 3] m/s, Rotational [1, 300] deg/s in
    /// rad/s, the absolute types within the controller's speed parameters.
    double speed = 0.1;
    sdk::SpeedType speed_type = sdk::SpeedType::Linear;  ///< for the moves of the TCP
    std::optional<double> acceleration{};                ///< fraction, [0.01, 1]; the controller's default otherwise
    std::optional<double> deceleration{};                ///< fraction, [0.01, 1]
    double blend_radius = 0.0;                           ///< circular_radius [m], [0, 2.608]; 0 for the last waypoint
    std::optional<sdk::StopCondition> until{};           ///< the path ends early once this holds (cond_*)
};

/// One path. It is the robot's open motion from run() until a wait has seen it
/// end. Move-only. Destroying it while it runs stops the arm (stop). Once its
/// Robot is gone, every call fails with Disconnected.
class Path {
public:
    /// Sends nothing.
    Path(Robot& robot, std::vector<Waypoint> waypoints);

    /// Loads the waypoints and runs them. Invalid for no waypoints, or values
    /// outside the manual's ranges (nothing sent); Busy while a motion is open;
    /// Command if the controller refuses a call or moveByPath reports another
    /// number of waypoints than were added.
    [[nodiscard]] std::expected<void, Error> run();
    /// getPathPointIndex [S 2.2.2.15]: the waypoint being run, from 0; -1
    /// outside a path.
    [[nodiscard]] std::expected<int, Error> index() const;

    /// Waits up to `timeout` for the path to end, as Move::wait_for.
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout);
    /// Waits until the path has ended, and says how.
    [[nodiscard]] std::expected<void, Error> wait();
    /// stop: the arm stops; a wait then sees the path end.
    void stop() noexcept;

    Path(Path&& other) noexcept;
    Path& operator=(Path&& other) noexcept;
    Path(const Path&) = delete;
    Path& operator=(const Path&) = delete;
    ~Path();

private:
    void end() noexcept;  // a running path is stopped and unregistered

    std::weak_ptr<detail::Connection> session_;
    std::vector<Waypoint> waypoints_;
    std::uint64_t id_ = 0;  // the registration as the open motion, while it runs; 0: none
    Clock::time_point started_{};
    bool ran_ = false;
};

}  // namespace ec::motion
