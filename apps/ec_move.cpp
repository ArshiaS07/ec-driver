// ec_move: move the arm through the driver's motion layer (docs/motion.md).
//
//   ec_move [--host H] [--command-port N] [--monitor-port N] [--yes] <command> ...
//
//   status                       the state, the conditions and the control loop's status
//   bring-up                     clear an alarm, reset a collision, synchronise, servos on: what is needed
//   joint J1 J2 J3 J4 J5 J6      move to joint positions [deg]            (--speed F, default 0.1)
//   nudge JOINT DEGREES          move one joint (1..6) by up to 30 deg    (--speed F)
//   sine JOINT AMPLITUDE PERIOD  stream a sine [deg, s] on one joint, from where it is
//                                (--seconds S, default two periods; --lookahead MS, default 40)
//   stop                         stop whatever moves the arm
//   acknowledge                  clear a latched fault, once a person has dealt with its cause
//
// Anything that moves the arm prints its plan and does nothing unless --yes is
// given. Ctrl+C stops the arm. The controller must be in remote mode.
// --record FILE writes every monitoring frame while the tool runs, as ec_record --raw does.
// Motions of the TCP are code, not options: apps/sbx_path.cpp.
//
// Exit codes: 0 done (or a plan printed); 1 refused, failed or stopped; 2 bad command line.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "ec/log/raw.hpp"
#include "ec/motion/point_to_point.hpp"
#include "ec/robot.hpp"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_interrupted = 0;
extern "C" void on_signal(int) { g_interrupted = 1; }

// Whether Ctrl+C was pressed since the last call.
bool interrupted() {
    if (g_interrupted == 0) return false;
    g_interrupted = 0;
    return true;
}

std::string degrees(const ec::Joints& q) {
    std::string s = "[";
    for (std::size_t j = 0; j < q.size(); ++j) s += std::format("{}{:.2f}", j == 0 ? "" : ", ", ec::rad_to_deg(q[j]));
    return s + "] deg";
}

void print_status(const ec::Robot& robot) {
    if (const auto s = robot.state()) {
        const auto& st = s->state;
        cli::out("mode {}, state {}, servos {}, encoders {}, collision {}, emergency stop {}\n", st.robot_mode,
                 st.robot_state, st.brakes_released ? "on" : "off",
                 st.motors_synced ? "synchronised" : "not synchronised", st.collision ? "yes" : "no",
                 st.emergency_stop ? "yes" : "no");
        cli::out("joints {}\n", degrees(st.joint_position));
    }
    const auto status = robot.status();
    cli::out("conditions: {}\n", ec::motion::to_string(status.conditions));
    cli::out("control loop: {}{}\n", ec::motion::to_string(status.mode),
             status.fault ? ", fault: " + ec::motion::describe(*status.fault) : std::string{});
}

// Waits for `job`, stopping the arm on Ctrl+C.
std::expected<void, ec::motion::Error> follow(ec::Robot& robot, const ec::motion::Job& job) {
    for (;;) {
        if (auto result = job.wait_for(20ms)) return *result;
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();
        }
    }
}

int report(const std::expected<void, ec::motion::Error>& result, std::string_view what) {
    if (result) {
        cli::out("{}: done\n", what);
        return cli::kOk;
    }
    cli::err("{}: {}\n", what, ec::motion::describe(result.error()));
    return cli::kFailed;
}

int dry_run(const std::string& plan) {
    cli::out("plan: {}\nnothing moved: add --yes to run it\n", plan);
    return cli::kOk;
}

int stream_sine(ec::Robot& robot, int joint, double amplitude_deg, double period_s, double seconds,
                std::chrono::milliseconds lookahead) {
    const auto now = robot.state();
    if (!now)
        return report(std::unexpected(ec::motion::Error{ec::motion::Error::Kind::Refused, "no state", {}}), "sine");
    const ec::Joints start = now->state.joint_position;
    auto session = robot.servo(ec::motion::ServoOptions{.lookahead = lookahead});
    if (!session) return report(std::unexpected(session.error()), "sine");
    const auto j = static_cast<std::size_t>(joint - 1);
    const auto t0 = Clock::now();
    auto next = t0;
    while (session->running() && g_interrupted == 0) {
        const double t = std::chrono::duration<double>(Clock::now() - t0).count();
        if (t >= seconds) break;
        ec::Joints q = start;
        q[j] += ec::deg_to_rad(amplitude_deg) * std::sin(2.0 * std::numbers::pi * t / period_s);
        session->target(q);
        next += 8ms;
        std::this_thread::sleep_until(next);
    }
    if (g_interrupted != 0) {
        cli::err("stopping\n");
        robot.stop();
    }
    return report(session->close(), "sine");
}

int act(ec::Robot& robot, const CLI::App& app, bool yes, double speed, const std::vector<double>& positions, int joint,
        double amount, double period, double seconds, int lookahead_ms);

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    std::string record;
    bool yes = false;
    double speed = 0.1;
    std::vector<double> positions;
    int joint = 0;
    double amount = 0.0;
    double period = 0.0;
    double seconds = 0.0;
    int lookahead_ms = 40;

    CLI::App app{"Move the arm through the driver's motion layer; see docs/motion.md."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str()->check(CLI::Range(1, 65535));
    app.add_option("--monitor-port", monitor_port, "Monitoring port")
        ->capture_default_str()
        ->check(CLI::Range(1, 65535));
    app.add_flag("--yes", yes, "Really move the arm; without it, only the plan is printed");
    app.add_option("--record", record, "Write every monitoring frame to this raw recording while the tool runs");
    app.require_subcommand(1, 1);
    app.fallthrough();

    app.add_subcommand("status", "The state, the conditions and the control loop's status");
    app.add_subcommand("bring-up", "Clear alarm, reset collision, synchronise, servos on");
    auto* move = app.add_subcommand("joint", "Move to joint positions [deg]");
    move->add_option("positions", positions, "J1..J6 [deg]")->required()->expected(6);
    move->add_option("--speed", speed, "Fraction of the joints' maximum speed")
        ->capture_default_str()
        ->check(CLI::Range(0.001, 1.0));
    auto* nudge = app.add_subcommand("nudge", "Move one joint by a few degrees");
    nudge->add_option("joint", joint, "1..6")->required()->check(CLI::Range(1, 6));
    nudge->add_option("degrees", amount, "-30..30")->required()->check(CLI::Range(-30.0, 30.0));
    nudge->add_option("--speed", speed, "Fraction of the joints' maximum speed")
        ->capture_default_str()
        ->check(CLI::Range(0.001, 1.0));
    auto* sine = app.add_subcommand("sine", "Stream a sine on one joint, from where it is");
    sine->add_option("joint", joint, "1..6")->required()->check(CLI::Range(1, 6));
    sine->add_option("amplitude", amount, "[deg], up to 20")->required()->check(CLI::Range(0.0, 20.0));
    sine->add_option("period", period, "[s], 0.5..60")->required()->check(CLI::Range(0.5, 60.0));
    sine->add_option("--seconds", seconds, "How long (default: two periods)")->check(CLI::Range(0.1, 600.0));
    sine->add_option("--lookahead", lookahead_ms, "[ms] the controller's queue delay")
        ->capture_default_str()
        ->check(CLI::Range(10, 1000));
    app.add_subcommand("stop", "Stop whatever moves the arm");
    app.add_subcommand("acknowledge", "Clear a latched fault");
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    // The recording outlives the robot, whose receiver thread writes to it.
    std::optional<ec::log::RawWriter> recording;
    if (!record.empty()) {
        auto writer = ec::log::RawWriter::create(record);
        if (!writer) {
            cli::err("{}\n", writer.error());
            return cli::kFailed;
        }
        recording.emplace(std::move(*writer));
    }
    ec::RobotOptions options{.host = host, .command_port = command_port, .monitor_port = monitor_port};
    if (recording) options.on_sample = [&recording](const ec::feedback::Sample& s) { recording->write(s); };

    int code = cli::kFailed;
    {
        auto robot = ec::Robot::connect(std::move(options));
        if (!robot) {
            cli::err("{}\n", ec::motion::describe(robot.error()));
        } else {
            std::signal(SIGINT, on_signal);
            std::signal(SIGTERM, on_signal);
            code = act(*robot, app, yes, speed, positions, joint, amount, period, seconds, lookahead_ms);
        }
    }
    if (recording) {
        if (const auto closed = recording->close(); !closed) {
            cli::err("recording {}: {}\n", record, closed.error());
            return cli::kFailed;
        }
        cli::out("recorded {} frames to {}\n", recording->written(), record);
    }
    return code;
}

// The subcommand, with the robot connected.
int act(ec::Robot& robot, const CLI::App& app, bool yes, double speed, const std::vector<double>& positions, int joint,
        double amount, double period, double seconds, int lookahead_ms) {
    const auto* status = app.get_subcommand("status");
    const auto* stop = app.get_subcommand("stop");
    const auto* acknowledge = app.get_subcommand("acknowledge");
    const auto* bring_up = app.get_subcommand("bring-up");
    const auto* move = app.get_subcommand("joint");
    const auto* nudge = app.get_subcommand("nudge");
    const auto* sine = app.get_subcommand("sine");
    const auto now = robot.state();
    if (!now) {
        cli::err("no state from the monitoring port\n");
        return cli::kFailed;
    }
    const ec::Joints here = now->state.joint_position;

    if (status->parsed()) {
        print_status(robot);
        return cli::kOk;
    }
    if (stop->parsed()) {
        robot.stop();
        const auto later = robot.next_state(now->seq + 3, 200ms);  // a few cycles, for the stop to go out
        static_cast<void>(later);
        cli::out("stop sent\n");
        return cli::kOk;
    }
    if (acknowledge->parsed()) {
        const auto r = robot.acknowledge();
        return report(r, "acknowledge");
    }
    if (bring_up->parsed()) {
        if (!yes) return dry_run("clear alarm, reset collision, synchronise and switch the servos on, as needed");
        return report(follow(robot, robot.bring_up()), "bring-up");
    }
    if (move->parsed() || nudge->parsed()) {
        ec::Joints target = here;
        if (move->parsed()) {
            for (std::size_t j = 0; j < target.size(); ++j) target[j] = ec::deg_to_rad(positions[j]);
        } else {
            target[static_cast<std::size_t>(joint - 1)] += ec::deg_to_rad(amount);
        }
        const std::string plan = std::format("move_joint from {} to {} at {:g} % of the joints' speed", degrees(here),
                                             degrees(target), speed * 100.0);
        if (!yes) return dry_run(plan);
        cli::out("{}\n", plan);
        return report(follow(robot, robot.run(ec::motion::MoveJoint{.target = target, .speed = speed})), "move_joint");
    }
    if (sine->parsed()) {
        if (seconds <= 0.0) seconds = 2.0 * period;
        const std::string plan =
            std::format("stream {:g} deg x sin(2 pi t / {:g} s) on J{} for {:g} s, lookahead {} ms", amount, period,
                        joint, seconds, lookahead_ms);
        if (!yes) return dry_run(plan);
        cli::out("{}\n", plan);
        return stream_sine(robot, joint, amount, period, seconds, std::chrono::milliseconds{lookahead_ms});
    }
    return cli::kUsage;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_move", [&] { return run(argc, argv); });
}
