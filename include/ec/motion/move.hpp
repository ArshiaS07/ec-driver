#pragma once
/// @file
/// Point-to-point moves (SDK manual 2.2.3.1-4, ADR 0018): the controller plans
/// and times one move to a target, given as joint positions. joint() moves in
/// joint space (moveByJoint); line() moves the TCP in a straight line to the
/// target's pose (moveByLine); arc() along the circle through the pose of
/// `middle` to the target's (moveByArc); rotate() turns the tool about its TCP
/// to the target's orientation (moveByRotate). Each sends its one call;
/// wait() then waits as the manual's wait_stop. The manual: remote mode only,
/// and the arm at rest.
///
///     ec::motion::Move move(robot);
///     move.joint(q, {.speed = 0.1});  // then move.wait()
///     move.line(q2, {.speed = 0.05});  // [m/s]; then move.wait()

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>

#include "ec/error.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/stamped.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// A joint move's speed and acceleration (moveByJoint).
struct JointMoveOptions {
    double speed = 0.1;  ///< fraction of the joints' maximum speed, [0.0001, 1] (speed, [0.01, 100] %)
    std::optional<double> acceleration{};  ///< fraction, [0.01, 1] (acc, [1, 100] %); the controller's default 0.8
    std::optional<double> deceleration{};  ///< fraction, [0.01, 1] (dec); the controller's default: the acceleration
    std::optional<sdk::StopCondition> until{};  ///< the move ends early once this holds (cond_*)
};

/// A move of the TCP: its speed and acceleration (moveByLine, moveByArc, moveByRotate).
struct PathMoveOptions {
    sdk::SpeedType speed_type = sdk::SpeedType::Linear;  ///< what `speed` is (speed_type)
    /// [m/s] for Linear, [0.001, 3], and AbsoluteLinear; [rad/s] for Rotational,
    /// [1, 300] deg/s, and AbsoluteRotational. The absolute ones lie within the
    /// controller's own speed parameters.
    double speed = 0.05;
    std::optional<double> acceleration{};  ///< fraction, [0.01, 1] (acc, [1, 100] %); the controller's default 0.8
    std::optional<double> deceleration{};  ///< fraction, [0.01, 1] (dec); the controller's default: the acceleration
    std::optional<sdk::StopCondition> until{};  ///< the move ends early once this holds (cond_*)
};

/// Moves, one at a time. From a move's call until a wait has seen it end, it is
/// the robot's open motion. Move-only; a move goes on if the object is
/// destroyed (stop() ends it).
class Move {
public:
    /// Sends nothing.
    explicit Move(Robot& robot);

    /// moveByJoint to `target` [rad]. Returns once the controller has taken the
    /// move; the arm then moves. Invalid for a target outside [-360, 360] deg or
    /// options outside their ranges; Busy while a motion is open (this one's
    /// last move included, until a wait has seen it end).
    [[nodiscard]] std::expected<void, Error> joint(const Joints& target, const JointMoveOptions& options = {});
    /// moveByLine to the pose of `target` [rad]. As joint().
    [[nodiscard]] std::expected<void, Error> line(const Joints& target, const PathMoveOptions& options = {});
    /// moveByArc through the pose of `middle` to that of `target` [rad]. As joint().
    [[nodiscard]] std::expected<void, Error> arc(const Joints& middle, const Joints& target,
                                                 const PathMoveOptions& options = {});
    /// moveByRotate to the orientation of `target` [rad]. As joint().
    [[nodiscard]] std::expected<void, Error> rotate(const Joints& target, const PathMoveOptions& options = {});

    /// Waits up to `timeout` for the move to end, asking getRobotState every 10
    /// ms (the manual's wait_stop): nullopt while the arm still moves; nothing
    /// once it is at rest; Fault on an emergency stop, an alarm or a collision;
    /// Stopped after Robot::stop(). Invalid without a move to wait for.
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout);
    /// Waits until the move has ended, and says how.
    [[nodiscard]] std::expected<void, Error> wait();
    /// stop: the arm stops; a wait then sees the move end.
    void stop() noexcept;

    Move(Move&& other) noexcept;
    Move& operator=(Move&& other) noexcept;
    Move(const Move&) = delete;
    Move& operator=(const Move&) = delete;
    ~Move();

private:
    // Opens the move, sends `command`, and notes when; closes it again if the call fails.
    template <class C>
    [[nodiscard]] std::expected<void, Error> start(const C& command);

    std::shared_ptr<detail::Connection> c_;
    std::uint64_t id_ = 0;  // the move's registration as the open motion; 0: none
    Clock::time_point started_{};
    bool ran_ = false;
};

}  // namespace ec::motion
