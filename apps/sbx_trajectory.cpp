// sbx_trajectory: a sandbox (temporary). The TCP draws the circle of circle():
// the robot is brought up, the poses are verified with the controller's
// inverse kinematics (planning.hpp), the arm makes a joint move to the first
// one, and the controller's timestamp service runs the circle
// (ec::motion::Timestamped: start_push_pos, push_pos for every point,
// stop_push_pos, check_trajectory, start_trajectory, as the SDK manual's
// example). Ctrl+C stops the arm.
//
//    sbx_trajectory --host 192.168.0.200
//    (against the fake robot: python3 tools/fake_robot.py, then --host 127.0.0.1)

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>

#include "cli.hpp"
#include "ec/motion/move.hpp"
#include "ec/motion/timestamped.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"
#include "ec/workflows.hpp"
#include "interrupt.hpp"
#include "planning.hpp"

namespace {

ec::Clock::duration at_seconds(double s) {
    return std::chrono::duration_cast<ec::Clock::duration>(std::chrono::duration<double>(s));
}

// The circle, as poses stamped with their time. Base frame, as on the pendant.
planning::PoseTrajectory circle() {
    // const Eigen::Vector3d center(-496.0, -122.0, 424.4);          // [mm], read on the pendant
    // const Eigen::Vector3d rxyz(180.0, 0.0, 90.0);                 // [deg], read on the pendant: the tool's
    // orientation
    const Eigen::Vector3d center(130.0, -315.0, 585.0);  // [mm], read on the pendant
    const Eigen::Vector3d rxyz(180.0, 0.0, -180.0);      // [deg], read on the pendant: the tool's orientation

    const double radius = 200.0;                              // [mm]
    const double speed = 150.0;                               // [mm/s] along the circle
    const double length = 2.0 * std::numbers::pi * radius;    // [mm] once round
    const int n = static_cast<int>(std::ceil(length / 2.0));  // number of poses: one every 2 mm
    // [s] to reach the speed at the start, and to stop at the end: at most 1 s,
    // less if the circle is too short to reach the speed in 1 s.
    const double ramp = std::min(1.0, length / speed);
    const double duration = length / speed + ramp;  // [s]: each ramp covers half of what the speed would

    // The distance [mm] along the circle t seconds after the start: the speed
    // rises as a half cosine over the ramp, holds, and falls the same way.
    const auto ramped = [&](double u) {
        return speed / 2.0 * (u - ramp / std::numbers::pi * std::sin(std::numbers::pi * u / ramp));
    };
    const auto distance = [&](double t) {
        if (t < ramp) return ramped(t);
        if (t > duration - ramp) return length - ramped(duration - t);
        return speed * ramp / 2.0 + speed * (t - ramp);
    };

    planning::PoseTrajectory poses;
    for (int k = 0; k <= n; ++k) {
        const double t = duration * k / n;
        const double a = distance(t) / radius;  // [rad] round the circle
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() =
            ec::rotation_from_rxyz(ec::deg_to_rad(rxyz.x()), ec::deg_to_rad(rxyz.y()), ec::deg_to_rad(rxyz.z()));
        T.translation() = (center + radius * Eigen::Vector3d(std::cos(a), std::sin(a), 0.0)) / 1000.0;  // [m]
        poses.push_back({at_seconds(t), T});
    }
    return poses;
}

int fail(std::string_view step, const ec::Error& e) {
    cli::err("{}: {}\n", step, describe(e));
    return cli::kFailed;
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";
    std::uint16_t command_port = 8055;
    CLI::App app{"Sandbox: the TCP draws the circle of circle() in apps/sbx_trajectory.cpp."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str();
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    // The command port only: nothing here needs the monitoring stream.
    auto robot = ec::Robot::connect({.host = host, .command_port = command_port, .monitor = false});
    if (!robot) return fail("connect", robot.error());
    const cli::StopOnInterrupt ctrl_c(*robot);

    // 1. Bring up: clear an alarm, synchronise the encoders, servos on; only what is needed.
    // The session switches the servos off again however this function returns.
    auto session = ec::Session::start(*robot);
    if (!session) {
        cli::err("{}", describe(session.error()));
        return cli::kFailed;
    }

    // 2. Verify: every pose solved by the controller's inverse kinematics near the
    // one before (the first near the joints now) and checked: no jump, no
    // singularity, within the limits; a joint trajectory, a point every 20 ms.
    auto plan = planning::plan(*robot, circle());
    if (!plan) return fail("verify", plan.error());
    cli::out("verified: {} points over {:.1f} s\n", plan->size(),
             std::chrono::duration<double>(plan->back().stamp).count());

    // 3. A joint move to the first point, at 10 % of the joints' speed.
    ec::motion::Move move(*robot);
    if (auto ok = move.joint(plan->front().value, {.speed = 0.1}); !ok) return fail("to the start", ok.error());
    if (auto there = move.wait(); !there) return fail("to the start", there.error());

    // 4. The circle: uploaded, started, and waited for.
    ec::motion::Timestamped circle_motion(*robot, std::move(*plan));
    if (auto ok = circle_motion.upload(); !ok) return fail("circle", ok.error());
    if (auto ok = circle_motion.start(); !ok) return fail("circle", ok.error());
    if (auto done = circle_motion.wait(); !done) return fail("circle", done.error());
    cli::out("done\n");

    // 5. The end of the session: stopped, at rest, servos off.
    if (const auto down = session->end(); !down.ok()) {
        cli::err("{}", describe(down));
        return cli::kFailed;
    }
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("sbx_trajectory", [&] { return run(argc, argv); });
}
