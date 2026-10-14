// sbx_trajectory: a sandbox (temporary). The TCP draws the circle of circle(),
// set from the pendant: the robot is brought up, the poses are verified (the
// controller's inverse kinematics, checked) and executed by the controller's
// timestamp service. Ctrl+C stops the arm.
//
//    sbx_trajectory --host 192.168.0.200
//    (against the fake robot: python3 tools/fake_robot.py, then --host 127.0.0.1)

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <expected>
#include <numbers>
#include <string>
#include <string_view>

#include "cli.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"

namespace {

using namespace std::chrono_literals;

ec::Clock::duration at_seconds(double s) {
    return std::chrono::duration_cast<ec::Clock::duration>(std::chrono::duration<double>(s));
}

// The circle, as poses stamped with their time. Base frame, as on the pendant.
ec::PoseTrajectory circle() {
    const Eigen::Vector3d center(-496.0, -122.0, 424.4);          // [mm], read on the pendant
    const Eigen::Vector3d rxyz(180.0, 0.0, 90.0);                 // [deg], read on the pendant: the tool's orientation
    const double radius = 30.0;                                   // [mm]
    const double speed = 20.0;                                    // [mm/s] along the circle
    const int n = 200;                                            // number of poses
    const double turn = 2.0 * std::numbers::pi * radius / speed;  // [s] once round
    ec::PoseTrajectory poses;
    for (int k = 0; k <= n; ++k) {
        const double a = 2.0 * std::numbers::pi * k / n;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() =
            ec::rotation_from_rxyz(ec::deg_to_rad(rxyz.x()), ec::deg_to_rad(rxyz.y()), ec::deg_to_rad(rxyz.z()));
        T.translation() = (center + radius * Eigen::Vector3d(std::cos(a), std::sin(a), 0.0)) / 1000.0;  // [m]
        poses.push_back({at_seconds(turn * k / n), T});
    }
    return poses;
}

// Ctrl+C: the handler only sets a flag; wait() stops the arm when it sees it.
volatile std::sig_atomic_t g_interrupted = 0;
extern "C" void on_signal(int) { g_interrupted = 1; }

// Waits for a motion to end, and says how. Ctrl+C stops the arm: the controller
// would run the trajectory on without the program.
std::expected<void, ec::motion::Error> wait(ec::Robot& robot, const ec::motion::Job& job) {
    for (;;) {
        if (auto done = job.wait_for(20ms)) return *done;
        if (g_interrupted != 0) {
            g_interrupted = 0;
            robot.stop();
        }
    }
}

int fail(std::string_view step, const ec::motion::Error& e) {
    cli::err("{}: {}\n", step, describe(e));
    return cli::kFailed;
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    CLI::App app{"Sandbox: the TCP draws the circle of circle() in apps/sbx_trajectory.cpp."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str();
    app.add_option("--monitor-port", monitor_port, "Monitoring port")->capture_default_str();
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    auto robot = ec::Robot::connect({.host = host, .command_port = command_port, .monitor_port = monitor_port});
    if (!robot) return fail("connect", robot.error());
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // Bring up: clear an alarm, synchronise the encoders, servos on; whatever the state needs.
    if (auto up = wait(*robot, robot->bring_up()); !up) return fail("bring-up", up.error());

    // Verify: every pose solved by the controller's inverse kinematics and checked;
    // a joint trajectory, a point every 20 ms. Nothing moves.
    const auto plan = robot->plan(circle());
    if (!plan) return fail("verify", plan.error());
    cli::out("verified: {} points over {:.1f} s\n", plan->size(), plan->duration());

    // Execute: a joint move to the circle's first point, then the circle by the
    // timestamp service (upload, check, start, wait until it arrives).
    const ec::motion::MoveJoint to_start{.target = plan->position.front(), .speed = 0.1};
    if (auto there = wait(*robot, robot->move_joints(to_start)); !there) return fail("to the start", there.error());
    if (auto done = wait(*robot, robot->play(*plan)); !done) return fail("circle", done.error());
    cli::out("done\n");
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("sbx_trajectory", [&] { return run(argc, argv); });
}
