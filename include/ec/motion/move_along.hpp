#pragma once
/// @file
/// A motion of the TCP (ADR 0018): poses stamped with their time, verified
/// with the controller's inverse kinematics, then run by the controller's
/// timestamp service after a joint move to the first one. Composed of plan()
/// (ec/motion/planning.hpp), Move and Timestamped.

#include <expected>

#include "ec/error.hpp"
#include "ec/motion/planning.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"

namespace ec::motion {

/// Verifies a motion of the TCP: plan() with the robot's inverse kinematics,
/// the first pose solved near the joints now. A checked joint trajectory, a
/// point every PlanOptions::period. Nothing moves.
[[nodiscard]] std::expected<JointTrajectory, Error> plan(Robot& robot, const PoseTrajectory& motion,
                                                         const PlanOptions& options = {});

struct AlongOptions {
    double approach_speed = 0.1;  ///< the joint move to the first pose: a fraction of the joints' maximum speed
    double speed = 1.0;           ///< the run: a fraction (0, 1] of the trajectory's own speed
    PlanOptions plan{};
};

/// The TCP along `motion`: plan(); a joint move to the first point, waited
/// for; the trajectory uploaded, started, and waited for. Returns once the arm
/// has stopped, or why it could not (Robot::stop() from another thread ends it
/// with Stopped).
[[nodiscard]] std::expected<void, Error> move_along(Robot& robot, const PoseTrajectory& motion,
                                                    const AlongOptions& options = {});

}  // namespace ec::motion
