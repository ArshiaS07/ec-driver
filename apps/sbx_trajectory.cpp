// sbx_trajectory: a sandbox (temporary). The TCP draws the circle of circle()
// with the controller's timestamp service, as the SDK manual's example does it
// (2.2.9.10): the five calls start_push_pos, push_pos for every point (each
// answered before the next), stop_push_pos, check_trajectory, start_trajectory.
// Before them, as in the example: bring-up, and a joint move to the first point.
// One connection, to the command port (8055); every call waits for its answer.
// Ctrl+C stops the arm.
//
//    sbx_trajectory --host 192.168.0.200
//    (against the fake robot: python3 tools/fake_robot.py, then --host 127.0.0.1)

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <numbers>
#include <string>
#include <thread>

#include "cli.hpp"
#include "ec/command/client.hpp"
#include "ec/command/kinematics.hpp"
#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "ec/command/servo.hpp"
#include "ec/command/trajectory.hpp"
#include "ec/motion/planning.hpp"
#include "ec/pose.hpp"

namespace {

using namespace std::chrono_literals;
namespace cmd = ec::command;

ec::Clock::duration at_seconds(double s) {
    return std::chrono::duration_cast<ec::Clock::duration>(std::chrono::duration<double>(s));
}

// The circle, as poses stamped with their time. Base frame, as on the pendant.
ec::PoseTrajectory circle() {
    // const Eigen::Vector3d center(-496.0, -122.0, 424.4);          // [mm], read on the pendant
    // const Eigen::Vector3d rxyz(180.0, 0.0, 90.0);                 // [deg], read on the pendant: the tool's
    // orientation
    const Eigen::Vector3d center(130.0, -315.0, 585.0);  // [mm], read on the pendant
    const Eigen::Vector3d rxyz(180.0, 0.0, -180.0);      // [deg], read on the pendant: the tool's orientation

    const double radius = 200.0;                              // [mm]
    const double speed = 40.0;                                // [mm/s] along the circle
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

    ec::PoseTrajectory poses;
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

// Ctrl+C: the handler only sets a flag, which the waits below look at.
volatile std::sig_atomic_t g_interrupted = 0;
extern "C" void on_signal(int) { g_interrupted = 1; }

// Sends one call and waits for its answer (the manual's sendCMD). A failure, or
// an answer of false, becomes "method: why".
template <cmd::Command C>
std::expected<typename C::Reply, std::string> send(cmd::Client& client, const C& call) {
    auto reply = client.call(call);
    if (!reply)
        return std::unexpected(
            std::format("{}: {}: {}", C::method, to_string(reply.error().kind), reply.error().message));
    return std::move(*reply);
}

// Waits until the arm has moved and stopped, asking getRobotState every 10 ms
// (the manual's wait_stop). Until it is seen running, a stop counts only after a
// second: a motion just started may not show yet. Ctrl+C sends `stop_call`.
template <cmd::Command Stop>
std::expected<void, std::string> wait_stopped(cmd::Client& client, const Stop& stop_call) {
    const auto started = ec::Clock::now();
    bool ran = false;
    for (;;) {
        std::this_thread::sleep_for(10ms);
        if (g_interrupted != 0) {
            static_cast<void>(send(client, stop_call));
            return std::unexpected("interrupted (Ctrl+C): stop sent");
        }
        const auto state = send(client, cmd::GetRobotState{});
        if (!state) return std::unexpected(state.error());
        switch (*state) {
            case ec::RobotState::Running: ran = true; break;
            case ec::RobotState::Stop:
                if (ran || ec::Clock::now() - started > 1s) return {};
                break;
            case ec::RobotState::EmergencyStop: return std::unexpected("emergency stop");
            case ec::RobotState::Alarm: return std::unexpected("the controller is in alarm");
            case ec::RobotState::Collision: return std::unexpected("collision");
            default: break;  // paused: wait
        }
    }
}

// Bring-up, as the manual's example: clear an alarm, synchronise the encoders,
// switch the servos on; only what the state needs.
std::expected<void, std::string> bring_up(cmd::Client& client) {
    const auto state = send(client, cmd::GetRobotState{});
    if (!state) return std::unexpected(state.error());
    if (*state == ec::RobotState::Alarm) {
        if (auto ok = send(client, cmd::ClearAlarm{}); !ok) return std::unexpected(ok.error());
        std::this_thread::sleep_for(500ms);
    }
    const auto synced = send(client, cmd::GetMotorStatus{});
    if (!synced) return std::unexpected(synced.error());
    if (!*synced) {
        if (auto ok = send(client, cmd::SyncMotorStatus{}); !ok) return std::unexpected(ok.error());
        std::this_thread::sleep_for(500ms);
    }
    std::this_thread::sleep_for(500ms);
    const auto servo = send(client, cmd::GetServoStatus{});
    if (!servo) return std::unexpected(servo.error());
    if (!*servo) {
        if (auto ok = send(client, cmd::SetServoStatus{.on = true}); !ok) return std::unexpected(ok.error());
    }
    return {};
}

// The timestamp service: the five calls, as the manual's example. The arm must
// rest at the trajectory's first point. Returns once the controller has started
// the trajectory; it then runs on the controller's own clock.
std::expected<void, std::string> upload_and_start(cmd::Client& client, const ec::motion::JointTrajectory& plan) {
    if (auto ok = send(client, cmd::StartPushPos{.point_count = static_cast<int>(plan.size()),
                                                 .point_type = cmd::PointType::Joint,
                                                 .reference = ec::rad_to_deg(plan.position.front()),
                                                 .frame = {},
                                                 .replies = true});
        !ok)
        return std::unexpected(ok.error());
    std::this_thread::sleep_for(200ms);
    for (std::size_t k = 0; k < plan.size(); ++k) {
        if (g_interrupted != 0) return std::unexpected("interrupted (Ctrl+C) while uploading: nothing moved");
        if (auto ok = send(client, cmd::PushPos{.time_s = plan.time[k], .point = ec::rad_to_deg(plan.position[k])});
            !ok)
            return std::unexpected(std::format("point {}: {}", k, ok.error()));
    }
    std::this_thread::sleep_for(1s);
    if (auto ok = send(client, cmd::StopPushPos{}); !ok) return std::unexpected(ok.error());
    const auto check = send(client, cmd::CheckTrajectory{});
    if (!check) return std::unexpected(check.error());
    if (*check != cmd::TrajectoryCheck::Valid)
        return std::unexpected(std::format("check_trajectory: {}", static_cast<int>(*check)));
    if (auto ok = send(client, cmd::StartTrajectory{.speed_pct = 100.0}); !ok) return std::unexpected(ok.error());
    return {};
}

int fail(std::string_view step, const std::string& why) {
    cli::err("{}: {}\n", step, why);
    return cli::kFailed;
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";
    std::uint16_t command_port = 8055;
    CLI::App app{"Sandbox: the TCP draws the circle of circle() in apps/sbx_trajectory.cpp."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str();
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    auto client = cmd::Client::connect(host, {.port = command_port, .reply_timeout = 5s});
    if (!client) return fail("connect", client.error().message);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // 1. Bring up.
    if (auto up = bring_up(*client); !up) return fail("bring-up", up.error());

    // 2. Verify: every pose solved by the controller's inverse kinematics near the
    // solution before (the first near the joints now) and checked: no jump, no
    // singularity, within the limits; a joint trajectory, a point every 20 ms.
    const auto now = send(*client, cmd::GetJointPos{});
    if (!now) return fail("verify", now.error());
    const auto ik = [&client](const Eigen::Isometry3d& T,
                              const ec::Joints& near) -> std::expected<ec::Joints, ec::motion::Error> {
        auto q = client->call(cmd::InverseKinematic{.target = ec::to_mm_rad(T), .reference = ec::rad_to_deg(near)});
        if (!q) return std::unexpected(ec::motion::command_failed(cmd::InverseKinematic::method, q.error()));
        return ec::nearest_turn(ec::deg_to_rad(*q), near, ec::kEc66JointLimit);
    };
    const auto plan = ec::motion::plan(circle(), ec::deg_to_rad(*now), ik);
    if (!plan) return fail("verify", describe(plan.error()));
    cli::out("verified: {} points over {:.1f} s\n", plan->size(), plan->duration());

    // 3. A joint move to the first point (at 10 % of the joints' speed), and
    // wait until the arm rests there.
    if (auto ok = send(*client, cmd::MoveByJoint{.target = ec::rad_to_deg(plan->position.front()), .speed_pct = 10.0});
        !ok)
        return fail("to the start", ok.error());
    if (auto ok = wait_stopped(*client, cmd::Stop{}); !ok) return fail("to the start", ok.error());
    const auto there = send(*client, cmd::GetJointPos{});
    if (!there) return fail("to the start", there.error());
    if (ec::max_abs_diff(ec::deg_to_rad(*there), plan->position.front()) > ec::deg_to_rad(0.5))
        return fail("to the start", "the arm did not reach the first point");

    // 4. The circle: upload and start it, and wait until it has run.
    if (auto ok = upload_and_start(*client, *plan); !ok) return fail("circle", ok.error());
    if (auto ok = wait_stopped(*client, cmd::StopTrajectory{}); !ok) return fail("circle", ok.error());
    cli::out("done\n");
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("sbx_trajectory", [&] { return run(argc, argv); });
}
