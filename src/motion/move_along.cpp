#include "ec/motion/move_along.hpp"

#include "ec/motion/move.hpp"
#include "ec/motion/timestamped.hpp"

namespace ec::motion {

std::expected<JointTrajectory, Error> plan(Robot& robot, const PoseTrajectory& motion, const PlanOptions& options) {
    const auto now = robot.joints();
    if (!now) return std::unexpected(now.error());
    return plan(
        motion, *now, [&robot](const Eigen::Isometry3d& T, const Joints& near) { return robot.ik(T, near); }, options);
}

std::expected<void, Error> move_along(Robot& robot, const PoseTrajectory& motion, const AlongOptions& o) {
    const auto trajectory = plan(robot, motion, o.plan);
    if (!trajectory) return std::unexpected(trajectory.error());
    {
        auto move = Move::joints(robot, trajectory->position.front(), {.speed = o.approach_speed});
        if (!move) return std::unexpected(move.error());
        if (auto there = move->wait(); !there) return std::unexpected(there.error());
    }
    auto run = Timestamped::upload(robot, *trajectory, {.limits = o.plan.limits});
    if (!run) return std::unexpected(run.error());
    if (auto ok = run->start(o.speed); !ok) return std::unexpected(ok.error());
    return run->wait();
}

}  // namespace ec::motion
