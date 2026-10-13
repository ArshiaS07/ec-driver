#pragma once
// ec_move circle: the TCP draws a circle, planned in joint space with the
// controller's inverse kinematics (docs/motion.md 5.11). See circle.cpp.

#include <Eigen/Geometry>
#include <functional>
#include <optional>
#include <string>

#include "ec/robot.hpp"

namespace circle {

struct Options {
    std::optional<Eigen::Vector3d> center_mm{};  // in the base frame; none: where the TCP is now
    double radius_mm = 0.0;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    std::optional<Eigen::Vector3d> start_toward{};  // none: toward the TCP (or base x)
    double speed_mm_s = 20.0;
    double turns = 1.0;
    double ramp_s = 0.5;
    double approach_speed_mm_s = 20.0;
    double max_approach_mm = 300.0;
    bool stream = false;  // push the poses live, solved one by one, instead of playing the plan
    std::string save{};
    bool yes = false;
};

// Plans the circle with the controller's inverse kinematics (computations
// only), prints the plan, and draws it if `options.yes`. `interrupted` is
// polled: true stops the arm. Returns the tool's exit code.
[[nodiscard]] int run(ec::Robot& robot, const Options& options, const std::function<bool()>& interrupted);

}  // namespace circle
