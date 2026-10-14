// sbx_path: a sandbox, to read and to change (temporary). It moves the TCP along
// a motion written below as a plain function, through the driver, one plain
// step after another, so that every call can be read where it happens.
//
// Two halves, kept apart:
//   the robot    where it is and whether to move: the command line (--host, the
//                ports, --yes, --record), nothing else
//   the motion   what the TCP does: functions from the TCP's pose now to poses
//                stamped with their time (square, circle, pivot, or your own);
//                drive() picks one, and how to run it: play() hands the plan to
//                the controller, stream() pushes the poses live
//
// Nothing moves without --yes: without it, the motion is planned and checked and
// the plan printed. Ctrl+C stops the arm at once. Every step either works or
// says what failed and exits with 1, the arm stopped; nothing is retried. The
// controller must be in remote mode with the servos on (ec_move --yes bring-up).
//
//    sbx_path                                    plan only
//    sbx_path --yes --record runs/square.ecraw   move; record every monitoring frame
//                                                (and the plan beside it, .plan.csv)
//    (against the fake robot: python3 tools/fake_robot.py --ready, then --host 127.0.0.1)
//
// Exit codes: 0 done (or planned); 1 failed or stopped; 2 bad command line.

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
#include "ec/log/clock.hpp"
#include "ec/log/raw.hpp"
#include "ec/motion/planning.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Clock;
using ec::Joints;
using ec::PoseTrajectory;
using ec::motion::Error;
using ec::motion::JointTrajectory;

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

// ================================================================== the robot

// Where the robot is, and whether to move it: all the command line sets.
struct Settings {
    std::string host = "192.168.0.200";  // the robot; 127.0.0.1 for the fake
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    bool yes = false;    // really move
    std::string record;  // a raw recording of every monitoring frame, and the plan beside it
};

// ================================================================= the motion
//
// A motion is an ec::PoseTrajectory: poses of the TCP, T_base_tcp [m], each
// stamped with its time since the start (ec/pose.hpp has the notation:
// p_base = T_base_tcp * p_tcp). The first pose must be where the TCP is, at 0 s,
// and the stamps must increase. Between two poses the TCP goes straight, its
// orientation turning evenly. The plan passes through every pose at its time
// and rests only at the start, at the end, and where a pose is stamped twice (a
// pause); to stop anywhere else, the times must slow down to it, as move_to()
// does. [[maybe_unused]]: drive() uses one motion at a time.

constexpr auto kStep = 50ms;  // between the poses these functions stamp

// 0 to 1 as x goes from 0 to 1, from rest to rest without a jolt at either end:
// 6x^5 - 15x^4 + 10x^3, the "smootherstep".
double ease(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

// Adds a move from the motion's last pose to `to`, taking `duration`, from rest
// to rest: a pose every kStep along the way.
void move_to(PoseTrajectory& motion, const Eigen::Isometry3d& to, Clock::duration duration) {
    const auto [t0, from] = motion.back();
    for (auto t = kStep; t < duration; t += kStep)
        motion.push_back({t0 + t, ec::interpolate(from, to, ease(seconds(t) / seconds(duration)))});
    motion.push_back({t0 + duration, to});
}

// A square of 30 mm in the base x-y plane: along base +x, +y, -x and -y, 3 s a
// side, stopping at each corner. The tool keeps its orientation.
[[maybe_unused]] PoseTrajectory square(const Eigen::Isometry3d& start) {
    const double side = 0.030;  // [m]
    PoseTrajectory motion{{0s, start}};
    for (const Eigen::Vector3d& corner : {Eigen::Vector3d(side, 0, 0), Eigen::Vector3d(side, side, 0),
                                          Eigen::Vector3d(0, side, 0), Eigen::Vector3d(0, 0, 0)}) {
        Eigen::Isometry3d to = start;
        to.translation() += corner;  // in base coordinates
        move_to(motion, to, 3s);
    }
    return motion;
}

// A circle of radius 30 mm in the base x-y plane through the TCP, its centre 30
// mm along base -x: once round, counter-clockwise seen from above, in 10 s from
// rest to rest. The tool keeps its orientation.
[[maybe_unused]] PoseTrajectory circle(const Eigen::Isometry3d& start) {
    const double radius = 0.030;  // [m]
    const auto duration = 10s;
    const Eigen::Vector3d center = start.translation() - radius * Eigen::Vector3d::UnitX();
    PoseTrajectory motion;
    for (auto t = 0ms; t < duration; t += kStep) {
        const double angle = 2.0 * std::numbers::pi * ease(seconds(t) / seconds(duration));
        Eigen::Isometry3d pose = start;
        pose.translation() = center + radius * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
        motion.push_back({t, pose});
    }
    motion.push_back({duration, start});
    return motion;
}

// The tool tilts 10 deg about its own x axis and back, then about its own y axis
// and back, 2 s each way, the TCP staying where it is. Also a check of the
// tool's TCP setting, by eye: if the setting is wrong, the real tip moves (the
// TCP the controller reports stays put either way).
[[maybe_unused]] PoseTrajectory pivot(const Eigen::Isometry3d& start) {
    const double tilt = ec::deg_to_rad(10.0);
    PoseTrajectory motion{{0s, start}};
    for (const Eigen::Vector3d& axis : {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)}) {
        // T_base_tcp * T_tcp_tilted: turned about the tool's own axis, through the TCP.
        move_to(motion, start * Eigen::AngleAxisd(tilt, axis), 2s);
        move_to(motion, start, 2s);
    }
    return motion;
}

// ============================================================ the machinery

// Ctrl+C: the handler only sets a flag, which the loops below look at.
volatile std::sig_atomic_t g_interrupted = 0;
extern "C" void on_signal(int) { g_interrupted = 1; }

bool interrupted() {
    if (g_interrupted == 0) return false;
    g_interrupted = 0;
    return true;
}

// A step failed: say which and why; the exit code.
int fail(std::string_view step, const Error& e) {
    cli::err("{}: {}\n", step, describe(e));
    return cli::kFailed;
}
int fail(std::string_view step, std::string_view why) {
    cli::err("{}: {}\n", step, why);
    return cli::kFailed;
}

std::string mm(const Eigen::Vector3d& p) {
    return std::format("({:.1f}, {:.1f}, {:.1f}) mm", p.x() * 1000.0, p.y() * 1000.0, p.z() * 1000.0);
}

std::string degrees(const Joints& q) {
    std::string s = "[";
    for (std::size_t j = 0; j < q.size(); ++j) s += std::format("{}{:.2f}", j == 0 ? "" : ", ", ec::rad_to_deg(q[j]));
    return s + "] deg";
}

// The TCP's pose now. state() is the newest monitoring frame (8056, about every
// 8 ms), decoded and in SI units: joints [rad], the TCP pose [m, rad]; a copy.
// The plan uses the controller's kinematics, so they must describe the TCP it
// reports: fk(), its positiveKinematic of the joints now (one call), must give
// the same pose. If not, a tool or user frame setting differs between the two,
// and the plan would move another point.
std::expected<Eigen::Isometry3d, std::string> tcp_now(ec::Robot& robot) {
    const auto now = robot.state();
    if (!now) return std::unexpected("no frame from the monitoring port");
    const Eigen::Isometry3d T_base_tcp = ec::isometry(now->state.tcp_position_base, now->state.tcp_rotation_base);
    cli::out("the TCP is at {}, the joints at {}\n", mm(T_base_tcp.translation()), degrees(now->state.joint_position));
    const auto fk = robot.fk(now->state.joint_position);
    if (!fk) return std::unexpected("forward kinematics: " + describe(fk.error()));
    const double off = (fk->translation() - T_base_tcp.translation()).norm();
    const double turn = ec::angle_between(fk->linear(), T_base_tcp.linear());
    cli::out("kinematics check: the controller's FK is {:.3f} mm and {:.3f} deg from the reported TCP\n", off * 1000.0,
             ec::rad_to_deg(turn));
    if (off > 0.001 || turn > ec::deg_to_rad(0.5))
        return std::unexpected("more than 1 mm or 0.5 deg: check the tool and user frame settings");
    return T_base_tcp;
}

// Whether the motion may be planned from `start`: a valid trajectory
// (ec::validate), starting where the TCP is, never further from it than kReach
// (a guard against a slip of units: millimetres written as metres). Prints where
// it goes.
constexpr double kReach = 0.3;  // [m]

std::expected<void, std::string> check(const PoseTrajectory& motion, const Eigen::Isometry3d& start) {
    if (auto ok = ec::validate(motion); !ok) return ok;
    const Eigen::Isometry3d& first = motion.front().value;
    if ((first.translation() - start.translation()).norm() > 0.001 ||
        ec::angle_between(first.linear(), start.linear()) > ec::deg_to_rad(0.5))
        return std::unexpected("it does not start where the TCP is (within 1 mm and 0.5 deg)");
    Eigen::Vector3d low = start.translation();
    Eigen::Vector3d high = low;
    for (const auto& [t, pose] : motion) {
        const double away = (pose.translation() - start.translation()).norm();
        if (away > kReach)
            return std::unexpected(std::format("the pose at {:.2f} s is {:.0f} mm from the start, more than {:.0f}",
                                               seconds(t), away * 1000.0, kReach * 1000.0));
        low = low.cwiseMin(pose.translation());
        high = high.cwiseMax(pose.translation());
    }
    cli::out(
        "motion: {} poses over {:.1f} s; the TCP stays within x {:.1f}..{:.1f}, y {:.1f}..{:.1f}, z {:.1f}..{:.1f} "
        "mm\n",
        motion.size(), seconds(motion.back().stamp), low.x() * 1000.0, high.x() * 1000.0, low.y() * 1000.0,
        high.y() * 1000.0, low.z() * 1000.0, high.z() * 1000.0);
    return {};
}

// How far the measured TCP strayed from the motion: for every monitoring frame,
// its distance to the nearest straight piece between two poses, and the turn of
// the tool to the nearest orientation on the way (ten to a piece, so exact to a
// twentieth of a piece's turn); the largest of each. add() only keeps the
// frame's TCP; report() compares, after the motion, so that watching costs the
// loops that move the arm nothing.
class Watch {
public:
    explicit Watch(const PoseTrajectory& motion) {
        orientations_.emplace_back(motion.front().value.linear());
        for (std::size_t k = 0; k < motion.size(); ++k) {
            positions_.push_back(motion[k].value.translation());
            if (k == 0) continue;
            const Eigen::Quaterniond a(motion[k - 1].value.linear());
            const Eigen::Quaterniond b(motion[k].value.linear());
            for (int i = 1; i <= 10; ++i) orientations_.push_back(a.slerp(i / 10.0, b));
        }
    }

    // The TCP from 8056 (machinePose), as a pose: position [m], rotation from rx, ry, rz.
    void add(const ec::feedback::State& s) {
        measured_.push_back(ec::isometry(s.tcp_position_base, s.tcp_rotation_base));
    }

    void report() const {
        if (measured_.empty()) return;
        double worst = 0.0;
        double worst_turn = 0.0;
        for (const Eigen::Isometry3d& T_base_tcp : measured_) {
            const Eigen::Vector3d p = T_base_tcp.translation();
            const Eigen::Quaterniond q(T_base_tcp.linear());
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t k = 1; k < positions_.size(); ++k) {
                const Eigen::Vector3d& a = positions_[k - 1];  // the piece from pose k-1 to pose k
                const Eigen::Vector3d ab = positions_[k] - a;
                const double n2 = ab.squaredNorm();
                const double u = n2 > 0.0 ? std::clamp((p - a).dot(ab) / n2, 0.0, 1.0) : 0.0;
                nearest = std::min(nearest, (p - (a + u * ab)).norm());
            }
            double closest = 0.0;  // the cosine of half the turn to the nearest orientation
            for (const Eigen::Quaterniond& o : orientations_) closest = std::max(closest, std::abs(q.dot(o)));
            worst = std::max(worst, nearest);
            worst_turn = std::max(worst_turn, 2.0 * std::acos(std::min(closest, 1.0)));
        }
        cli::out(
            "the TCP stayed within {:.2f} mm of the motion's path, the tool within {:.2f} deg of its "
            "orientations ({} frames)\n",
            worst * 1000.0, ec::rad_to_deg(worst_turn), measured_.size());
    }

private:
    std::vector<Eigen::Vector3d> positions_;        // the poses'
    std::vector<Eigen::Quaterniond> orientations_;  // the poses', and on the way
    std::vector<Eigen::Isometry3d> measured_;
};

// Planned: robot.play() hands the joint trajectory to the controller's timestamp
// service. The driver's control thread uploads it (start_push_pos, a push_pos
// per point, stop_push_pos), has the controller check it (check_trajectory) and
// starts it (start_trajectory); the controller then runs it on its own clock.
// play() returns at once with a Job, which ends when the arm rests at the last
// point, or with the reason it could not: a refusal, a lost condition (the
// E-stop, remote mode left, ...), a timeout. On any ending but arrival the
// control thread stops the arm (stop_trajectory, stop) and clears the upload.
[[maybe_unused]] std::expected<void, Error> play(ec::Robot& robot, const JointTrajectory& plan, Watch& watch) {
    const ec::motion::Job job = robot.play(plan);
    std::uint64_t seq = 0;
    for (;;) {
        if (auto done = job.wait_for(0ms)) return *done;  // ended: nothing, or why
        // next_state() waits for a frame newer than `seq`: one per frame, no busy loop.
        if (const auto s = robot.next_state(seq, 20ms)) {
            seq = s->seq;
            watch.add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();  // the job then ends with Kind::Stopped, the arm stopped
        }
    }
}

// Streamed: no plan handed over; targets pushed live, as an application that
// only knows the next pose would. Every 8 ms: the motion's pose at that time,
// solved by the controller's inverse kinematics near the last solution, checked
// as the plan checks its points (check_step: no jump, no singularity, within the
// joint limits), and pushed with target() into the position stream (transparent
// transmission). The stream shapes each target within speed and acceleration
// limits, and stops the arm if the targets stop coming (its watchdog). Closing
// it lets the arm settle and clears the controller's queue. ([[maybe_unused]]:
// drive() uses play() or this.)
[[maybe_unused]] std::expected<void, Error> stream(ec::Robot& robot, const PoseTrajectory& motion, Watch& watch) {
    // servo() opens the stream at standstill, from where the arm is, and returns
    // once the controller accepted it: nothing moves until the first target.
    auto session = robot.servo();
    if (!session) return std::unexpected(session.error());
    const auto now = robot.state();
    if (!now) return std::unexpected(Error{Error::Kind::Refused, "no state", {}});
    Joints last = now->state.joint_position;
    const ec::motion::PlanOptions checks;          // the plan's checks, for every target
    const auto end = motion.back().stamp + 500ms;  // the end, and half a second for the arm to settle there
    const auto t0 = Clock::now();
    std::uint64_t seq = 0;
    for (auto next = t0; session->running(); next += 8ms) {  // running(): false once it ended by itself (an error)
        const auto t = Clock::now() - t0;
        if (t > end) break;
        // ik(): the controller's inverseKinematic for this pose, the solution
        // nearest `last` [rad]. One call on the command port, answered between
        // the control thread's cycles.
        auto q = robot.ik(ec::pose_at(motion, t), last);
        if (q) {
            if (auto ok = ec::motion::check_step(last, *q, checks); !ok)
                q = std::unexpected(
                    Error{Error::Kind::Invalid, std::format("at {:.2f} s: {}", seconds(t), ok.error()), {}});
        }
        if (!q) {
            robot.stop();  // no next target: end the stream at once
            const auto closed = session->close();
            static_cast<void>(closed);  // the error that matters is the pose's
            return std::unexpected(std::move(q.error()));
        }
        last = *q;
        session->target(last);  // the newest target wins; the control thread sends it
        if (const auto s = robot.state(); s && s->seq != seq) {
            seq = s->seq;
            watch.add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();  // ends the stream at once: stop, clear
        }
        std::this_thread::sleep_until(next + 8ms);
    }
    return session->close();  // nothing if it ended well, else why it ended
}

// The plan as CSV, a row per joint point: its time, the motion's pose then (the
// controller's units: mm; rx, ry, rz in rad), and the joints [deg]. `started` is
// when the motion started, on the steady clock: the time base of the recording.
bool save(const std::string& file, const PoseTrajectory& motion, const JointTrajectory& plan,
          std::optional<Clock::time_point> started) {
    std::ofstream f(file);
    f << "# sbx_path plan; started at steady clock "
      << (started ? std::to_string(ec::log::to_ns(*started)) + " ns" : std::string("- (not run)"))
      << ", the time base of ec_record and --record files\n";
    f << "t_s,x_mm,y_mm,z_mm,rx_rad,ry_rad,rz_rad,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg\n";
    for (std::size_t k = 0; k < plan.size(); ++k) {
        const auto p = ec::to_mm_rad(ec::pose_at(
            motion, std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(plan.time[k]))));
        const Joints q = ec::rad_to_deg(plan.position[k]);
        f << std::format("{:.3f},{:.4f},{:.4f},{:.4f},{:.6f},{:.6f},{:.6f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n",
                         plan.time[k], p[0], p[1], p[2], p[3], p[4], p[5], q[0], q[1], q[2], q[3], q[4], q[5]);
    }
    return static_cast<bool>(f);
}

// ===================================================================== drive

// With the robot connected: from the TCP now to the motion done.
int drive(ec::Robot& robot, const Settings& s) {
    // 1. Where the TCP is, checked against the controller's kinematics.
    const auto start = tcp_now(robot);
    if (!start) return fail("the TCP", start.error());

    // 2. The motion: square, circle or pivot (above), or your own.
    const PoseTrajectory motion = square(*start);
    if (const auto ok = check(motion, *start); !ok) return fail("motion", ok.error());

    // 3. Plan. robot.plan() solves every pose by the controller's inverse
    // kinematics (inverseKinematic, one call each, near the solution before; the
    // first near the joints now) and checks it: no joint turning more than 10 deg
    // between neighbours (a change of arm configuration), J5 and J3 at least 5 deg
    // from 0 and 180 (the wrist and elbow singularities), the joint limits. The
    // solutions are interpolated over time, sampled every 20 ms, and checked for
    // joint speed and acceleration. Computations only: nothing moves.
    const auto planned_at = Clock::now();
    const auto plan = robot.plan(motion);
    if (!plan) return fail("plan", plan.error());
    if (interrupted()) return fail("plan", "interrupted (Ctrl+C): nothing moved");  // planning takes a while
    const Joints fastest = ec::motion::peak_velocity(*plan);
    const auto worst = static_cast<std::size_t>(std::max_element(fastest.begin(), fastest.end()) - fastest.begin());
    const auto margins = ec::motion::singularity_margins(plan->position);
    cli::out("plan: {} joint points over {:.1f} s (solved in {:.1f} s); the fastest joint is J{} at {:.1f} deg/s\n",
             plan->size(), plan->duration(), seconds(Clock::now() - planned_at), worst + 1,
             ec::rad_to_deg(fastest[worst]));
    cli::out("closest to a singularity: J5 {:.1f} deg from 0 or 180 (wrist), J3 {:.1f} deg (elbow)\n",
             ec::rad_to_deg(margins.wrist), ec::rad_to_deg(margins.elbow));

    // The plan goes beside the recording: what the arm was asked to do, next to
    // what it did.
    const std::string plan_file = s.record.empty() ? std::string{} : s.record + ".plan.csv";
    if (!s.yes) {
        if (!plan_file.empty() && !save(plan_file, motion, *plan, std::nullopt))
            return fail("save", std::format("could not write {}", plan_file));
        cli::out("nothing moved: add --yes to run it\n");
        return cli::kOk;
    }

    // 4. Ready? The driver refuses to start a motion unless the state is fresh,
    // the controller in remote mode, without E-stop, alarm or collision, the
    // servos on, the encoders synchronised and the arm at a standstill.
    if (const auto missing = ec::motion::kCanStartMotion.missing_from(robot.status().conditions); !missing.empty())
        return fail("ready", std::format("missing: {} (ec_move --yes bring-up does what software may)",
                                         ec::motion::to_string(missing)));

    // 5. Move: play() hands the plan to the controller; stream() pushes the poses
    // live instead, each solved on the way. Pick one.
    Watch watch(motion);
    const auto started = Clock::now();
    const auto moved = play(robot, *plan, watch);  // or: stream(robot, motion, watch)
    const double took = seconds(Clock::now() - started);

    // 6. Report.
    if (!plan_file.empty() && !save(plan_file, motion, *plan, started))
        cli::err("could not write {}\n", plan_file);  // the motion's result matters more
    watch.report();
    if (!moved) return fail("motion", moved.error());
    cli::out("done in {:.1f} s\n", took);
    return cli::kOk;
}

int run(int argc, char** argv) {
    // The command line (CLI11; apps/cli.hpp gives the exit codes): the robot
    // only. The motion is in the code above.
    Settings s;
    CLI::App app{
        "Sandbox: moves the TCP along a motion written in apps/sbx_path.cpp, planned with the "
        "controller's inverse kinematics."};
    app.add_option("--host", s.host, "Controller address")->capture_default_str();
    app.add_option("--command-port", s.command_port, "Command port")->capture_default_str();
    app.add_option("--monitor-port", s.monitor_port, "Monitoring port")->capture_default_str();
    app.add_flag("--yes", s.yes, "Really move the arm; without it, only the plan is printed");
    app.add_option("--record", s.record,
                   "Write every monitoring frame to this raw recording, and the plan to FILE.plan.csv");
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    // The recording: the receiver thread hands every frame to on_sample, which
    // writes it as received (the format of ec_record --raw). The file must outlive
    // the robot, whose receiver writes to it: it is opened first, closed last.
    std::optional<ec::log::RawWriter> recording;
    if (!s.record.empty()) {
        auto writer = ec::log::RawWriter::create(s.record);
        if (!writer) return fail("record", writer.error());
        recording.emplace(std::move(*writer));
    }
    ec::RobotOptions options{.host = s.host, .command_port = s.command_port, .monitor_port = s.monitor_port};
    if (recording) options.on_sample = [&recording](const ec::feedback::Sample& sample) { recording->write(sample); };

    int code = cli::kFailed;
    {
        // Connect. Robot::connect opens the command port (8055: a request, then its
        // reply) and the monitoring port (8056: frames the robot pushes about every
        // 8 ms), starts the thread that receives the frames and the control thread
        // that owns the command port and runs motions, and returns once the first
        // frame has come and the control thread has run its first cycle.
        // Destroying the Robot stops whatever runs, waits for the arm to rest, and
        // closes both ports.
        auto robot = ec::Robot::connect(std::move(options));
        if (!robot) return fail("connect", robot.error());
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        code = drive(*robot, s);
    }
    if (recording) {
        if (const auto closed = recording->close(); !closed) return fail("record", closed.error());
        cli::out("recorded {} frames to {}\n", recording->written(), s.record);
    }
    return code;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("sbx_path", [&] { return run(argc, argv); });
}
