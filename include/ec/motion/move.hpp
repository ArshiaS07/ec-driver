#pragma once
/// @file
/// Point-to-point moves (SDK manual 2.2.3.1, ADR 0018): the controller plans
/// and times one move to a target. Joint moves for now; the controller's line
/// and arc moves follow (milestone B).

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

struct MoveOptions {
    double speed = 0.1;                    ///< fraction of the joints' maximum speed, (0, 1]
    std::optional<double> acceleration{};  ///< fraction of the maximum, (0, 1]; the controller's default otherwise
    double tolerance = deg_to_rad(0.5);    ///< [rad] on every joint: how close to the target counts as arrived
};

/// One move. It is the robot's open motion until it has ended (a wait says
/// so) or the object is destroyed. Move-only; the move it started goes on if
/// the object is destroyed (stop() ends it).
class Move {
public:
    /// moveByJoint to `target` [rad]. Returns once the controller has taken
    /// the move; the arm then moves. Invalid for a target beyond the joint
    /// limits or a speed outside (0, 1]; Busy while another motion is open.
    [[nodiscard]] static std::expected<Move, Error> joints(Robot& robot, const Joints& target,
                                                           const MoveOptions& options = {});

    /// Waits up to `timeout`: nullopt while the arm still moves; nothing once
    /// it rests at the target; else why not (it rested elsewhere, a fault, a
    /// stop).
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_for(std::chrono::nanoseconds timeout);
    /// Waits until the move has ended, and says how.
    [[nodiscard]] std::expected<void, Error> wait();
    /// Ends the move at once (stop).
    void stop() noexcept;

    Move(Move&& other) noexcept;
    Move& operator=(Move&& other) noexcept;
    Move(const Move&) = delete;
    Move& operator=(const Move&) = delete;
    ~Move();

private:
    Move(std::shared_ptr<detail::Connection> c, std::uint64_t id, const Joints& target, double tolerance);

    std::shared_ptr<detail::Connection> c_;
    std::uint64_t id_ = 0;
    Joints target_{};
    double tolerance_ = 0.0;
    Clock::time_point started_{};
    bool ran_ = false;
    std::optional<std::expected<void, Error>> result_{};  // once it has ended
};

}  // namespace ec::motion
