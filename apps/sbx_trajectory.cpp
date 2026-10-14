// sbx_trajectory: a sandbox, to read and to change (temporary). The TCP draws a
// circle, through the driver, one plain step after another.
//
// The circle is struct Circle below: edit its numbers, rebuild, run. The program
// turns it into poses stamped with their time (trajectory()), plans them from
// where the arm is (the controller's inverse kinematics, checked), prints the
// plan, and asks before anything moves. Planned (the default), the controller's
// timestamp service plays the joint trajectory; with --stream, the poses are
// pushed live instead, each solved on the way. Ctrl+C stops the arm. The
// controller must be in remote mode with the servos on (ec_move --yes bring-up).
//
//    sbx_trajectory --host 192.168.0.200                             planned
//    sbx_trajectory --host 192.168.0.200 --stream                    streamed
//    sbx_trajectory --host 192.168.0.200 --record runs/circle.ecraw  and every monitoring frame recorded
//    (against the fake robot: python3 tools/fake_robot.py --ready, then --host 127.0.0.1)
//
// Exit codes: 0 done, or not run; 1 failed or stopped; 2 bad command line.

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <format>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
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

// ================================================================= the circle

// The circle to draw: edit these numbers, rebuild, run. Millimetres, in the
// robot's base frame.
struct Circle {
    // Its centre; empty: where the TCP is when the program starts. To set one:
    // center_mm = Eigen::Vector3d(-496.0, -122.0, 424.4).
    std::optional<Eigen::Vector3d> center_mm{};
    double radius_mm = 30.0;
    // The TCP goes round it counter-clockwise (right-hand rule): 0 0 1 is level,
    // counter-clockwise seen from above; 0 0 -1 clockwise.
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    double speed_mm_s = 20.0;  // the top speed along the way
};

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

// 0 to 1 as x goes from 0 to 1, from rest to rest: 6x^5 - 15x^4 + 10x^3. Its top
// speed, at x = 0.5, is 1.875 times its average.
double ease(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

// Adds a move to the end of `motion`, from rest to rest: pose(s) as s goes from 0
// to 1, eased, a pose every 50 ms. It is `length` [m] long and goes at most at
// `speed` [m/s]; it takes half a second at least.
template <typename PoseOf>
void add_move(PoseTrajectory& motion, double length, double speed, const PoseOf& pose) {
    const double duration = std::max(0.5, 1.875 * length / speed);
    const auto t0 = motion.back().stamp;
    const auto end = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(duration));
    for (auto t = Clock::duration{50ms}; t + 1ms < end; t += 50ms)
        motion.push_back({t0 + t, pose(ease(seconds(t) / duration))});
    motion.push_back({t0 + end, pose(1.0)});
}

// The circle as poses stamped with their time (ec::PoseTrajectory, a
// std::vector of ec::Stamped<Eigen::Isometry3d>), from the TCP's pose now (`tcp`,
// T_base_tcp [m]): if the TCP is not on the circle, first a straight line to the
// circle's point nearest to it (from the centre itself: toward base x, or y for
// an axis along x); then once round. The tool keeps its orientation all the way.
// Or why the circle will not do.
std::expected<PoseTrajectory, std::string> trajectory(const Circle& c, const Eigen::Isometry3d& tcp) {
    if (!(c.radius_mm >= 1.0 && c.radius_mm <= 200.0)) return std::unexpected("the radius must be 1 to 200 mm");
    if (!(c.speed_mm_s > 0.0 && c.speed_mm_s <= 100.0)) return std::unexpected("the speed must be 0 to 100 mm/s");
    if (!(c.axis.allFinite() && c.axis.norm() > 1e-6)) return std::unexpected("the axis must not be zero");
    const Eigen::Vector3d center = c.center_mm ? Eigen::Vector3d(*c.center_mm / 1000.0) : tcp.translation();
    const double radius = c.radius_mm / 1000.0;
    const double speed = c.speed_mm_s / 1000.0;

    // The circle's own axes: n along its axis; u from the centre toward its first
    // point, in its plane; v = n x u, the way the TCP sets off.
    const Eigen::Vector3d n = c.axis.normalized();
    const auto in_plane = [&n](const Eigen::Vector3d& w) { return Eigen::Vector3d(w - w.dot(n) * n); };
    Eigen::Vector3d u = in_plane(tcp.translation() - center);
    if (u.norm() < 1e-6) u = in_plane(std::abs(n.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY());
    u.normalize();
    const Eigen::Vector3d v = n.cross(u);
    const auto on_circle = [&](double angle) {
        Eigen::Isometry3d pose = tcp;  // the tool's orientation, kept
        pose.translation() = center + radius * (std::cos(angle) * u + std::sin(angle) * v);
        return pose;
    };

    PoseTrajectory motion{{0s, tcp}};
    const Eigen::Isometry3d first = on_circle(0.0);
    const double gap = (first.translation() - tcp.translation()).norm();
    if (gap > 0.1)
        return std::unexpected(
            std::format("the TCP is {:.0f} mm from the circle: move it closer (100 mm at most)", gap * 1000.0));
    if (gap > 1e-5) add_move(motion, gap, speed, [&](double s) { return ec::interpolate(tcp, first, s); });
    const double turn = 2.0 * std::numbers::pi;
    add_move(motion, turn * radius, speed, [&](double s) { return on_circle(turn * s); });
    return motion;
}

// ============================================================== the machinery

// Ctrl+C while the arm moves: the handler only sets a flag, which the loops below
// look at. (Before that, Ctrl+C ends the program at once: nothing moves then.)
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

// Asks on the terminal: true only for an answer starting with y. The end of the
// input is a no.
bool confirmed(std::string_view question) {
    cli::out("{} [y/N] ", question);
    std::fflush(stdout);
    std::string answer;
    if (!std::getline(std::cin, answer)) return false;
    return answer.starts_with('y') || answer.starts_with('Y');
}

// How far the measured TCP strays from the motion: for every monitoring frame,
// its distance to the nearest straight piece between two of the motion's poses,
// and the turn of the tool from its orientation at the start (which the circle
// keeps); the largest of each. add() only keeps the frame's TCP; report()
// compares, after the motion, so that watching costs the loops that move the arm
// nothing.
class Watch {
public:
    explicit Watch(const PoseTrajectory& motion) : orientation_(motion.front().value.linear()) {
        for (const auto& [t, pose] : motion) positions_.push_back(pose.translation());
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
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t k = 1; k < positions_.size(); ++k) {
                const Eigen::Vector3d& a = positions_[k - 1];  // the piece from pose k-1 to pose k
                const Eigen::Vector3d ab = positions_[k] - a;
                const double n2 = ab.squaredNorm();
                const double u = n2 > 0.0 ? std::clamp((p - a).dot(ab) / n2, 0.0, 1.0) : 0.0;
                nearest = std::min(nearest, (p - (a + u * ab)).norm());
            }
            worst = std::max(worst, nearest);
            worst_turn = std::max(worst_turn, ec::angle_between(T_base_tcp.linear(), orientation_));
        }
        cli::out(
            "the TCP stayed within {:.2f} mm of the motion's path, the tool within {:.2f} deg of its "
            "orientation ({} frames)\n",
            worst * 1000.0, ec::rad_to_deg(worst_turn), measured_.size());
    }

private:
    Eigen::Matrix3d orientation_;
    std::vector<Eigen::Vector3d> positions_;
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
std::expected<void, Error> play(ec::Robot& robot, const ec::motion::JointTrajectory& plan, Watch& watch) {
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
// it lets the arm settle and clears the controller's queue.
std::expected<void, Error> stream(ec::Robot& robot, const PoseTrajectory& motion, Watch& watch) {
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

// ======================================================================= draw

// With the robot connected: the circle, from where the arm is to done.
int draw(ec::Robot& robot, bool streamed) {
    // 1. Where the arm is. state() is the newest monitoring frame (8056, about every
    // 8 ms), decoded and in SI units: joints [rad], the TCP pose [m, rad]; a copy.
    // The TCP the circle starts from is the controller's forward kinematics of the
    // joints (positiveKinematic, one call): the tool its inverse kinematics uses
    // too. 8056's own TCP is printed beside it; far apart, they mean two tools.
    const auto now = robot.state();
    if (!now) return fail("state", "no frame from the monitoring port");
    const Joints here = now->state.joint_position;
    const auto tcp = robot.fk(here);
    if (!tcp) return fail("forward kinematics", tcp.error());
    const Eigen::Vector3d reported(now->state.tcp_position_base[0], now->state.tcp_position_base[1],
                                   now->state.tcp_position_base[2]);
    const double apart = (reported - tcp->translation()).norm();
    cli::out("the TCP is at {} (8056 reports it {:.3f} mm from there{}), the joints at {}\n", mm(tcp->translation()),
             apart * 1000.0, apart > 0.001 ? ": another tool?" : "", degrees(here));

    // 2. The circle, as poses stamped with their time.
    const auto motion = trajectory(Circle{}, *tcp);
    if (!motion) return fail("circle", motion.error());
    Eigen::Vector3d low = tcp->translation();
    Eigen::Vector3d high = low;
    for (const auto& [t, pose] : *motion) {
        low = low.cwiseMin(pose.translation());
        high = high.cwiseMax(pose.translation());
    }
    const Eigen::Vector3d start = motion->back().value.translation();  // once round: it ends where it starts
    if ((start - tcp->translation()).norm() > 1e-5)
        cli::out("first a straight line of {:.1f} mm to the circle\n", (start - tcp->translation()).norm() * 1000.0);
    cli::out(
        "the circle starts and ends at {}; the TCP stays within x {:.1f}..{:.1f}, y {:.1f}..{:.1f}, "
        "z {:.1f}..{:.1f} mm; {} poses over {:.1f} s\n",
        mm(start), low.x() * 1000.0, high.x() * 1000.0, low.y() * 1000.0, high.y() * 1000.0, low.z() * 1000.0,
        high.z() * 1000.0, motion->size(), seconds(motion->back().stamp));

    // 3. Plan. robot.plan() solves every pose by the controller's inverse
    // kinematics (inverseKinematic, one call each, near the solution before; the
    // first near the joints now) and checks it: no joint turning more than 10 deg
    // between neighbours (a change of arm configuration), J5 and J3 at least 5 deg
    // from 0 and 180 (the wrist and elbow singularities), the joint limits. The
    // solutions are interpolated over time, sampled every 20 ms, and checked for
    // joint speed and acceleration. Computations only: nothing moves.
    const auto planned_at = Clock::now();
    const auto plan = robot.plan(*motion);
    if (!plan) return fail("plan", plan.error());
    const Joints fastest = ec::motion::peak_velocity(*plan);
    const auto worst = static_cast<std::size_t>(std::max_element(fastest.begin(), fastest.end()) - fastest.begin());
    const auto margins = ec::motion::singularity_margins(plan->position);
    cli::out("plan: {} joint points over {:.1f} s (solved in {:.1f} s); the fastest joint is J{} at {:.1f} deg/s\n",
             plan->size(), plan->duration(), seconds(Clock::now() - planned_at), worst + 1,
             ec::rad_to_deg(fastest[worst]));
    cli::out("closest to a singularity: J5 {:.1f} deg from 0 or 180 (wrist), J3 {:.1f} deg (elbow)\n",
             ec::rad_to_deg(margins.wrist), ec::rad_to_deg(margins.elbow));

    // 4. Ready, and asked. The driver refuses to start a motion unless the state
    // is fresh, the controller in remote mode, without E-stop, alarm or collision,
    // the servos on, the encoders synchronised and the arm at a standstill.
    if (const auto missing = ec::motion::kCanStartMotion.missing_from(robot.status().conditions); !missing.empty())
        return fail("ready", std::format("missing: {} (ec_move --yes bring-up does what software may)",
                                         ec::motion::to_string(missing)));
    if (!confirmed(streamed ? "Run it, streamed (the poses pushed live)?"
                            : "Run it, planned (played by the controller's timestamp service)?")) {
        cli::out("nothing moved\n");
        return cli::kOk;
    }
    // While it waited for the answer, the arm must not have moved: the plan
    // starts where it was.
    if (const auto again = robot.state();
        !again || ec::max_abs_diff(again->state.joint_position, here) > ec::deg_to_rad(0.5))
        return fail("ready", "the arm moved since it was planned: run again");

    // 5. Move.
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    Watch watch(*motion);
    const auto started = Clock::now();
    const auto moved = streamed ? stream(robot, *motion, watch) : play(robot, *plan, watch);

    // 6. Report.
    watch.report();
    if (!moved) return fail("motion", moved.error());
    cli::out("done in {:.1f} s\n", seconds(Clock::now() - started));
    return cli::kOk;
}

int run(int argc, char** argv) {
    // The command line (CLI11; apps/cli.hpp gives the exit codes): the robot, and
    // how to run the circle. The circle itself is struct Circle above.
    std::string host = "192.168.0.200";  // the robot; 127.0.0.1 for the fake
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    bool streamed = false;
    std::string record;
    CLI::App app{
        "Sandbox: the TCP draws the circle set in apps/sbx_trajectory.cpp, planned with the controller's "
        "inverse kinematics; asks before it moves."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str();
    app.add_option("--monitor-port", monitor_port, "Monitoring port")->capture_default_str();
    app.add_flag("--stream", streamed, "Push the poses live, each solved, instead of playing the planned trajectory");
    app.add_option("--record", record, "Write every monitoring frame to this raw recording");
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    // The recording: the receiver thread hands every frame to on_sample, which
    // writes it as received (the format of ec_record --raw). The file must outlive
    // the robot, whose receiver writes to it: it is opened first, closed last.
    std::optional<ec::log::RawWriter> recording;
    if (!record.empty()) {
        auto writer = ec::log::RawWriter::create(record);
        if (!writer) return fail("record", writer.error());
        recording.emplace(std::move(*writer));
    }
    ec::RobotOptions options{.host = host, .command_port = command_port, .monitor_port = monitor_port};
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
        code = draw(*robot, streamed);
    }
    if (recording) {
        if (const auto closed = recording->close(); !closed) return fail("record", closed.error());
        cli::out("recorded {} frames to {}\n", recording->written(), record);
    }
    return code;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("sbx_trajectory", [&] { return run(argc, argv); });
}
