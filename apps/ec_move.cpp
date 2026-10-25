// ec_move: move the arm through the driver's motion families.
//
//   ec_move [--host H] [--command-port N] [--monitor-port N] [--log-port N] [--yes] <command> ...
//
//   status                       the state the monitoring port reports, and what a motion still needs
//   bring-up                     clear an alarm, synchronise, servos on: what is needed (ec::bring_up)
//   recover                      the last alarms and log errors, then bring-up (ec::recover)
//   shut-down                    stop, wait for rest, servos off (ec::shut_down)
//   joint J1 J2 J3 J4 J5 J6      move to joint positions [deg]            (--speed F, default 0.1)
//   nudge JOINT DEGREES          move one joint (1..6) by up to 30 deg    (--speed F)
//   sine JOINT AMPLITUDE PERIOD  stream a sine [deg, s] on one joint, from where it is
//                                (--seconds S, default two periods; --lookahead MS, default 40)
//   stop                         stop whatever moves the arm
//
// Anything that moves the arm prints its plan and does nothing unless --yes is
// given. Ctrl+C stops the arm. The controller must be in remote mode.
// --record FILE writes every monitoring frame while the tool runs, as ec_record --raw does;
// --calls FILE every exchange on the command connection, as JSON lines (docs/recording-format.md).
// Motions of the TCP are code, not options: apps/sbx_trajectory.cpp.
//
// Exit codes: 0 done (or a plan printed); 1 refused, failed or stopped; 2 bad command line.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "ec/motion/move.hpp"
#include "ec/motion/servo_stream.hpp"
#include "ec/readiness.hpp"
#include "ec/record/calls.hpp"
#include "ec/record/raw.hpp"
#include "ec/robot.hpp"
#include "ec/workflows.hpp"
#include "interrupt.hpp"

namespace {

using namespace std::chrono_literals;

std::string degrees(const ec::Joints& q) {
    std::string s = "[";
    for (std::size_t j = 0; j < q.size(); ++j) s += std::format("{}{:.2f}", j == 0 ? "" : ", ", ec::rad_to_deg(q[j]));
    return s + "] deg";
}

int report(const std::expected<void, ec::Error>& result, std::string_view what) {
    if (result) {
        cli::out("{}: done\n", what);
        return cli::kOk;
    }
    cli::err("{}: {}\n", what, describe(result.error()));
    return cli::kFailed;
}

// A workflow's report: every step to stdout; 1 if it stopped short.
int report(const ec::Report& r) {
    cli::out("{}", describe(r));
    return r.ok() ? cli::kOk : cli::kFailed;
}

int dry_run(const std::string& plan) {
    cli::out("plan: {}\nnothing moved: add --yes to run it\n", plan);
    return cli::kOk;
}

// Streams q(t) = start + amplitude sin(2 pi t / period) on one joint, a point
// every period of the stream, paced here (ServoStream: the application paces).
std::expected<void, ec::Error> stream_sine(ec::Robot& robot, const cli::StopOnInterrupt& ctrl_c,
                                           const ec::Joints& start, std::size_t joint, double amplitude_deg,
                                           double period_s, double seconds, std::chrono::milliseconds lookahead) {
    const ec::motion::ServoOptions options{.lookahead = lookahead};
    ec::motion::ServoStream stream(robot, options);
    if (auto ok = stream.open(); !ok) return std::unexpected(ok.error());
    const auto t0 = ec::Clock::now();
    for (auto next = t0; !ctrl_c.interrupted(); next += options.period) {
        const double t = std::chrono::duration<double>(ec::Clock::now() - t0).count();
        if (t >= seconds) break;
        ec::Joints q = start;
        q[joint] += ec::deg_to_rad(amplitude_deg) * std::sin(2.0 * std::numbers::pi * t / period_s);
        if (auto ok = stream.add_point(q); !ok) return std::unexpected(ok.error());
        std::this_thread::sleep_until(next + options.period);
    }
    if (ctrl_c.interrupted()) return std::unexpected(ec::Error{ec::Error::Kind::Stopped, "Ctrl+C"});
    return stream.close();
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    std::uint16_t log_port = 8058;
    std::string record;
    std::string calls;
    bool yes = false;
    double speed = 0.1;
    std::vector<double> positions;
    int joint = 0;
    double amount = 0.0;
    double period = 0.0;
    double seconds = 0.0;
    int lookahead_ms = 40;

    CLI::App app{"Move the arm through the driver's motion families."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str()->check(CLI::Range(1, 65535));
    app.add_option("--monitor-port", monitor_port, "Monitoring port")
        ->capture_default_str()
        ->check(CLI::Range(1, 65535));
    app.add_option("--log-port", log_port, "The controller's log port (recover reads it)")
        ->capture_default_str()
        ->check(CLI::Range(1, 65535));
    app.add_flag("--yes", yes, "Really move the arm; without it, only the plan is printed");
    app.add_option("--record", record, "Write every monitoring frame to this raw recording while the tool runs");
    app.add_option("--calls", calls, "Write every exchange on the command connection to this call log (JSON lines)");
    app.require_subcommand(1, 1);
    app.fallthrough();

    auto* status = app.add_subcommand("status", "The state the monitoring port reports");
    auto* bring_up = app.add_subcommand("bring-up", "Clear an alarm, synchronise, servos on");
    auto* recover = app.add_subcommand("recover", "The last alarms and log errors, then bring-up");
    auto* shut_down = app.add_subcommand("shut-down", "Stop, wait for rest, servos off");
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
    auto* stop = app.add_subcommand("stop", "Stop whatever moves the arm");
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    // The recording outlives the robot, whose receiver thread writes to it.
    std::optional<ec::record::RawWriter> recording;
    if (!record.empty()) {
        auto writer = ec::record::RawWriter::create(record);
        if (!writer) {
            cli::err("{}\n", writer.error());
            return cli::kFailed;
        }
        recording.emplace(std::move(*writer));
    }
    std::optional<ec::record::CallWriter> call_log;
    if (!calls.empty()) {
        auto writer = ec::record::CallWriter::create(calls);
        if (!writer) {
            cli::err("{}\n", writer.error());
            return cli::kFailed;
        }
        call_log.emplace(std::move(*writer));
    }
    ec::RobotOptions options{
        .host = host, .command_port = command_port, .monitor_port = monitor_port, .log_port = log_port};
    if (recording) options.on_sample = [&recording](const ec::feedback::Sample& s) { recording->write(s); };
    if (call_log) options.on_call = [&call_log](const ec::CallRecord& c) { call_log->write(c); };

    int code = cli::kFailed;
    {
        auto robot = ec::Robot::connect(std::move(options));
        if (!robot) {
            cli::err("{}\n", describe(robot.error()));
            return cli::kFailed;
        }
        const cli::StopOnInterrupt ctrl_c(*robot);
        const auto now = robot->monitor().latest();
        if (!now) {
            cli::err("no state from the monitoring port\n");
            return cli::kFailed;
        }
        const ec::Joints here = now->positions();

        if (status->parsed()) {
            const ec::State& st = *now;
            cli::out("mode {}, state {}, servos {}, encoders {}, collision {}, emergency stop {}\n", st.system.mode,
                     st.motion.state, st.drives.servo_on ? "on" : "off",
                     st.drives.synced ? "synchronised" : "not synchronised", st.safety.collision ? "yes" : "no",
                     st.safety.emergency_stop ? "yes" : "no");
            cli::out("joints {}\n", degrees(here));
            if (const auto blocker = ec::readiness(st); blocker != ec::Blocker::None)
                cli::out("before a motion: {}: {}\n", blocker, ec::remedy(blocker));
            code = cli::kOk;
        } else if (stop->parsed()) {
            robot->motion().stop();
            cli::out("stop sent\n");
            code = cli::kOk;
        } else if (bring_up->parsed()) {
            code = !yes ? dry_run("clear alarm, synchronise and switch the servos on, as needed")
                        : report(ec::bring_up(*robot));
        } else if (recover->parsed()) {
            code = !yes ? dry_run("read the last alarms and log errors, then bring up") : report(ec::recover(*robot));
        } else if (shut_down->parsed()) {
            code = !yes ? dry_run("stop, wait for rest, switch the servos off") : report(ec::shut_down(*robot));
        } else if (move->parsed() || nudge->parsed()) {
            ec::Joints target = here;
            if (move->parsed()) {
                for (std::size_t j = 0; j < target.size(); ++j) target[j] = ec::deg_to_rad(positions[j]);
            } else {
                target[static_cast<std::size_t>(joint - 1)] += ec::deg_to_rad(amount);
            }
            const std::string plan = std::format("move_joint from {} to {} at {:g} % of the joints' speed",
                                                 degrees(here), degrees(target), speed * 100.0);
            if (!yes) {
                code = dry_run(plan);
            } else {
                cli::out("{}\n", plan);
                ec::motion::Move m(*robot);
                const auto started = m.joint(target, {.speed = speed});
                code = report(started ? m.wait() : started, "move_joint");
            }
        } else if (sine->parsed()) {
            if (seconds <= 0.0) seconds = 2.0 * period;
            const std::string plan =
                std::format("stream {:g} deg x sin(2 pi t / {:g} s) on J{} for {:g} s, lookahead {} ms", amount, period,
                            joint, seconds, lookahead_ms);
            if (!yes) {
                code = dry_run(plan);
            } else {
                cli::out("{}\n", plan);
                code = report(stream_sine(*robot, ctrl_c, here, static_cast<std::size_t>(joint - 1), amount, period,
                                          seconds, std::chrono::milliseconds{lookahead_ms}),
                              "sine");
            }
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

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_move", [&] { return run(argc, argv); });
}
