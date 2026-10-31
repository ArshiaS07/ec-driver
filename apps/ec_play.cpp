// ec_play: run a trajectory from a CSV file, TCP poses or joint positions,
// each with its time: planned and checked here, executed by the controller's
// timestamp service (SDK manual 2.2.9).
//
//   ec_play FILE.csv [--host H] [--command-port N] [--monitor-port N] [--speed F] [--record FILE] [--yes]
//
// The file: a header row naming the columns, then one row per point; blank
// lines and lines starting with # are skipped. t starts at 0 and increases;
// the same point at two neighbouring times is a pause, held.
//
//   t,x,y,z,rx,ry,rz       the TCP in the base frame: t [s], x y z [mm], rx ry rz [deg],
//                          as the pendant shows it (R = Rz(rz) Ry(ry) Rx(rx)). Planned with
//                          the controller's inverse kinematics (apps/planning.hpp): straight
//                          lines between the poses, filled in every 3 mm and 1 deg, each
//                          solution checked for jumps, singularities and the joint limits.
//   t,j1,j2,j3,j4,j5,j6    joint positions [deg].
//
// Either way the joints follow a smooth spline over time through every point,
// at rest at both ends, sampled every 20 ms, checked against the joints'
// speed and acceleration limits.
//
// Without --yes it changes nothing: it reads the file, plans (the poses ask the
// controller's inverse kinematics, a computation) and prints the plan. With
// --yes, in one session (bring-up first; servos off at the end, however it
// ends): a joint move to the first point at 10 % of the joints' speed, the
// upload, the run at --speed times the file's pace, the wait. Ctrl+C stops the
// arm. --record FILE writes the monitoring stream meanwhile (raw, as
// ec_record --raw; ec_record --replay makes it HDF5).
//
// Exit codes: 0 done (or planned); 1 a bad file, a failure or a stop; 2 bad command line.

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include "cli.hpp"
#include "ec/motion/move.hpp"
#include "ec/motion/timestamped.hpp"
#include "ec/pose.hpp"
#include "ec/record/raw.hpp"
#include "ec/robot.hpp"
#include "ec/workflows.hpp"
#include "interrupt.hpp"
#include "planning.hpp"

namespace {

// --------------------------------------------------------------- the file

using Motion = std::variant<planning::PoseTrajectory, ec::JointTrajectory>;

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.remove_suffix(1);
    return s;
}

std::vector<std::string_view> split(std::string_view line) {
    std::vector<std::string_view> fields;
    for (;;) {
        const auto comma = line.find(',');
        fields.push_back(trim(line.substr(0, comma)));
        if (comma == std::string_view::npos) return fields;
        line.remove_prefix(comma + 1);
    }
}

std::optional<double> number(std::string_view s) {
    double v = 0.0;
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || end != s.data() + s.size() || !std::isfinite(v)) return std::nullopt;
    return v;
}

ec::Clock::duration seconds(double s) {
    return std::chrono::duration_cast<ec::Clock::duration>(std::chrono::duration<double>(s));
}

// The motion in `path`, or why it is not one (naming the line).
std::expected<Motion, std::string> read_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) return std::unexpected(std::format("{}: cannot open the file", path));
    const std::vector<std::string_view> pose_header{"t", "x", "y", "z", "rx", "ry", "rz"};
    const std::vector<std::string_view> joint_header{"t", "j1", "j2", "j3", "j4", "j5", "j6"};
    std::optional<bool> poses;  // which form, once the header is read
    planning::PoseTrajectory pose_motion;
    ec::JointTrajectory joint_motion;
    int number_of_line = 0;
    for (std::string text; std::getline(in, text);) {
        ++number_of_line;
        const std::string_view line = trim(text);
        if (line.empty() || line.front() == '#') continue;
        const auto fields = split(line);
        if (!poses) {
            if (fields == pose_header)
                poses = true;
            else if (fields == joint_header)
                poses = false;
            else
                return std::unexpected(
                    std::format("{}:{}: the header must be t,x,y,z,rx,ry,rz (TCP poses) or "
                                "t,j1,j2,j3,j4,j5,j6 (joint positions)",
                                path, number_of_line));
            continue;
        }
        if (fields.size() != 7)
            return std::unexpected(std::format("{}:{}: {} values, not 7", path, number_of_line, fields.size()));
        std::array<double, 7> v{};
        for (std::size_t i = 0; i < 7; ++i) {
            const auto x = number(fields[i]);
            if (!x)
                return std::unexpected(
                    std::format("{}:{}: column {}: \"{}\" is not a number", path, number_of_line, i + 1, fields[i]));
            v[i] = *x;
        }
        const auto t = seconds(v[0]);
        if (*poses) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.translation() = Eigen::Vector3d(v[1], v[2], v[3]) / 1000.0;
            T.linear() = ec::rotation_from_rxyz(ec::deg_to_rad(v[4]), ec::deg_to_rad(v[5]), ec::deg_to_rad(v[6]));
            pose_motion.push_back({t, T});
        } else {
            joint_motion.push_back({t, ec::deg_to_rad(ec::Joints{v[1], v[2], v[3], v[4], v[5], v[6]})});
        }
    }
    if (!poses) return std::unexpected(std::format("{}: no header, no points", path));
    if (*poses) return Motion(std::move(pose_motion));
    return Motion(std::move(joint_motion));
}

// --------------------------------------------------------------- the run

std::string degrees(const ec::Joints& q) {
    std::string s = "[";
    for (std::size_t j = 0; j < q.size(); ++j) s += std::format("{}{:.2f}", j == 0 ? "" : ", ", ec::rad_to_deg(q[j]));
    return s + "] deg";
}

double seconds_of(ec::Clock::duration d) { return std::chrono::duration<double>(d).count(); }

int fail(std::string_view step, const ec::Error& e) {
    cli::err("{}: {}\n", step, describe(e));
    return cli::kFailed;
}

// The motions, inside the session: each ends (stopped, if still open) before it does.
int play(ec::Robot& robot, ec::JointTrajectory plan, double speed) {
    ec::motion::Move to_start(robot);
    cli::out("to the start: {} at 10 % of the joints' speed\n", degrees(plan.front().value));
    if (auto ok = to_start.joint(plan.front().value, {.speed = 0.1}); !ok) return fail("to the start", ok.error());
    if (auto ok = to_start.wait(); !ok) return fail("to the start", ok.error());

    ec::motion::Timestamped run(robot, std::move(plan));
    cli::out("upload\n");
    if (auto ok = run.upload(); !ok) return fail("upload", ok.error());
    cli::out("run at {:g} times the file's pace\n", speed);
    if (auto ok = run.start(speed); !ok) return fail("run", ok.error());
    if (auto ok = run.wait(); !ok) return fail("run", ok.error());
    cli::out("run: done\n");
    return cli::kOk;
}

int run(int argc, char** argv) {
    std::string file;
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    double speed = 1.0;
    std::string record;
    bool yes = false;

    CLI::App app{"Run a trajectory from a CSV file of TCP poses or joint positions."};
    app.add_option("file", file, "CSV: t,x,y,z,rx,ry,rz [s, mm, deg] or t,j1,...,j6 [s, deg]")->required();
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str()->check(CLI::Range(1, 65535));
    app.add_option("--monitor-port", monitor_port, "Monitoring port")
        ->capture_default_str()
        ->check(CLI::Range(1, 65535));
    app.add_option("--speed", speed, "Times the file's pace, 0.001..1")
        ->capture_default_str()
        ->check(CLI::Range(0.001, 1.0));
    app.add_option("--record", record, "Write the monitoring stream to this raw recording meanwhile");
    app.add_flag("--yes", yes, "Really move the arm; without it, only the plan is printed");
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    auto motion = read_csv(file);
    if (!motion) {
        cli::err("{}\n", motion.error());
        return cli::kFailed;
    }

    // The recording outlives the robot, whose monitor thread writes to it.
    std::optional<ec::record::RawWriter> recording;
    if (!record.empty()) {
        auto writer = ec::record::RawWriter::create(record);
        if (!writer) {
            cli::err("{}\n", writer.error());
            return cli::kFailed;
        }
        recording.emplace(std::move(*writer));
    }
    ec::RobotOptions options{.host = host, .command_port = command_port, .monitor_port = monitor_port};
    if (recording) options.on_sample = [&recording](const ec::feedback::Sample& s) { recording->write(s); };
    auto robot = ec::Robot::connect(std::move(options));
    if (!robot) return fail("connect", robot.error());

    // Planned and checked before anything moves.
    const bool poses = std::holds_alternative<planning::PoseTrajectory>(*motion);
    auto plan = poses ? planning::plan(*robot, std::get<planning::PoseTrajectory>(*motion))
                      : planning::plan_joints(std::get<ec::JointTrajectory>(*motion));
    if (!plan) return fail("plan", plan.error());
    const std::size_t points =
        poses ? std::get<planning::PoseTrajectory>(*motion).size() : std::get<ec::JointTrajectory>(*motion).size();
    cli::out("{}: {} {} over {:.2f} s, planned into {} points\n", file, points, poses ? "TCP poses" : "joint positions",
             seconds_of(plan->back().stamp), plan->size());
    cli::out("starts at {}, ends at {}\n", degrees(plan->front().value), degrees(plan->back().value));
    if (const auto now = robot->monitor().latest()) {
        double furthest = 0.0;
        for (std::size_t j = 0; j < ec::kJointCount; ++j)
            furthest = std::max(furthest, std::abs(plan->front().value[j] - now->joints[j].position));
        cli::out("the arm is {:.2f} deg (on its furthest joint) from the start\n", ec::rad_to_deg(furthest));
    }
    if (!yes) {
        cli::out("nothing moved: add --yes to run it\n");
        return cli::kOk;
    }

    const cli::StopOnInterrupt ctrl_c(*robot);
    cli::out("bring-up:\n");
    auto session = ec::Session::start(*robot);
    if (!session) {
        cli::out("{}", describe(session.error()));
        return cli::kFailed;
    }
    cli::out("{}", describe(session->started()));
    const int code = play(*robot, std::move(*plan), speed);
    cli::out("shut-down:\n");
    const auto down = session->end();  // stopped, at rest, servos off
    cli::out("{}", describe(down));
    if (recording) {
        // The robot's monitor still writes until the robot is gone: the file closes with the process.
        cli::out("recorded {} frames to {}\n", recording->written(), record);
    }
    return down.ok() ? code : cli::kFailed;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_play", [&] { return run(argc, argv); });
}
