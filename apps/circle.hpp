#pragma once
// ec_move circle: the TCP draws a circle, in each of the ways the driver can
// move the arm (docs/motion.md 5.11). See circle.cpp.

#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include "ec/pose.hpp"
#include "ec/robot.hpp"

namespace circle {

// How the circle is drawn.
enum class Mode {
    Arc,         // moveByArc, half a turn at a time: the controller's own arcs
    Path,        // a waypoint path of short lines with blended corners (moveByPath)
    Lines,       // straight lines from corner to corner, each to a pose (moveByLineCoord), stopping at each
    Trajectory,  // a timed joint trajectory uploaded to the controller and run on its clock
    Follow,      // the same trajectory streamed by the PC (transparent transmission)
    PoseStream,  // TCP poses pushed live, each solved by the controller's inverse kinematics
    Velocity,    // joint velocities pushed live (moveBySpeedj): the planned speed plus a position correction
};

// The modes by name, for the command line.
[[nodiscard]] std::optional<Mode> parse_mode(std::string_view name);
[[nodiscard]] std::string_view to_string(Mode mode);

struct Options {
    Mode mode = Mode::Follow;
    std::optional<ec::Vec3> center_mm{};  // in the base frame; none: where the TCP is now
    double radius_mm = 0.0;
    ec::Vec3 normal{0.0, 0.0, 1.0};
    std::optional<ec::Vec3> start_toward{};  // none: toward the TCP (or base x)
    double speed_mm_s = 20.0;
    double turns = 1.0;
    double ramp_s = 0.5;
    double approach_speed_mm_s = 20.0;
    double max_approach_mm = 300.0;
    int segments = 0;   // per turn, for path and lines; 0: their default
    double gain = 4.0;  // [1/s], velocity mode's position correction
    std::string save{};
    bool yes = false;
};

// Plans the circle with the controller's inverse kinematics (read-only), prints
// the plan, and draws it if `options.yes`. `interrupted` is polled: true stops
// the arm. Returns the tool's exit code.
[[nodiscard]] int run(ec::Robot& robot, const Options& options, const std::function<bool()>& interrupted);

}  // namespace circle
