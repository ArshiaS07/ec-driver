// sbx_path: a sandbox, to read and to change (temporary). It takes a list of TCP
// points through the driver to the robot, one plain step after another, so that
// every call can be read where it happens:
//
//    1 command line   2 recording   3 connect   4 the state now   5 kinematics check
//    6 the path       7 plan        8 show, save   (without --yes: stop here)
//    9 ready?        10 play, or stream          11 report
//
// The path is made of pieces that start and end at rest. It starts where the TCP
// is now and keeps the tool's orientation:
//
//    --shape square   a square of side --size [mm] in the base x-y plane, and back to the start
//    --shape circle   out to a circle of radius --size around the TCP, once round, and back in
//    --shape points   to each point of --points FILE in turn: "x y z" [mm, base frame] per line
//
// To make your own path, edit make_path(): anything that returns timed TCP poses
// from rest to rest, starting at the TCP, goes.
//
// Nothing moves without --yes. Ctrl+C stops the arm at once. Every step either
// works or says what failed and exits with 1, the arm stopped; nothing is retried
// or recovered. The controller must be in remote mode to move.
//
//    sbx_path --shape square --size 30                                      plan only
//    sbx_path --yes --shape circle --size 30 --record recordings/circle.ecraw
//    sbx_path --yes --shape points --points my_points.txt --speed 10
//    (against the fake robot: python3 tools/fake_robot.py --ready, then --host 127.0.0.1)
//
// Exit codes: 0 done (or planned); 1 failed or stopped; 2 bad command line.

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
#include "ec/log/clock.hpp"
#include "ec/log/raw.hpp"
#include "ec/motion/planning.hpp"
#include "ec/path.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using ec::Joints;
using ec::motion::Error;
using ec::motion::JointTrajectory;
using ec::motion::TimedPose;

// ------------------------------------------------------------------ settings

// Everything the command line sets, with its default.
struct Settings {
    std::string host = "192.168.0.200";  // the robot; 127.0.0.1 for the fake
    std::uint16_t command_port = 8055;
    std::uint16_t monitor_port = 8056;
    std::string shape = "square";           // square, circle or points
    double size_mm = 30.0;                  // the square's side, the circle's radius
    std::string points_file;                // --shape points: "x y z" [mm] per line
    std::vector<Eigen::Vector3d> points{};  // its points [m], read before connecting
    double speed_mm_s = 20.0;               // TCP speed on every piece
    double ramp_s = 0.5;                    // [s] to reach the speed, and to stop, on every piece
    double max_distance_mm = 300.0;         // no point of the path further than this from the start
    bool stream = false;                    // push the poses live instead of playing the plan
    bool bring_up = false;                  // clear an alarm, synchronise, servos on, if needed
    bool yes = false;                       // really move
    std::string record;                     // a raw recording of every monitoring frame
    std::string save;                       // the plan as CSV
};

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

// ------------------------------------------------------------------ the path

// Appends `piece` (any path: ec::Line, ec::Circle, ...) to `path`, timed from
// rest to rest: SpeedProfile raises the speed over `ramp` seconds as a half
// cosine, holds it, and lowers it the same way; sample() takes a pose every 3 mm
// of the piece, and more often in the ramps where it is slow. The piece must
// start where the path ends; its first pose, the same as the path's last, is
// left out. A piece of length 0 adds nothing.
template <ec::CartesianPath Piece>
std::expected<void, std::string> append(std::vector<TimedPose>& path, const Piece& piece, double speed, double ramp) {
    const auto profile = ec::motion::SpeedProfile::make(length(piece), speed, ramp);
    if (!profile) return std::unexpected(profile.error());
    const double offset = path.empty() ? 0.0 : path.back().time;
    for (TimedPose p : ec::motion::sample(piece, *profile, 0.003)) {
        if (!path.empty() && p.time == 0.0) continue;  // the piece's start: the path's end already
        p.time += offset;
        path.push_back(p);
    }
    return {};
}

// A path from `start` through `points` in turn, in straight lines, stopping at
// each point, with the start's orientation all the way.
std::expected<std::vector<TimedPose>, std::string> through(const Eigen::Isometry3d& start,
                                                           const std::vector<Eigen::Vector3d>& points, double speed,
                                                           double ramp) {
    std::vector<TimedPose> path{{.time = 0.0, .pose = start}};
    Eigen::Isometry3d from = start;
    for (const Eigen::Vector3d& p : points) {
        Eigen::Isometry3d to = start;
        to.translation() = p;
        if (auto ok = append(path, ec::Line{.from = from, .to = to}, speed, ramp); !ok)
            return std::unexpected(ok.error());
        from = to;
    }
    return path;
}

// The points of a file: one "x y z" per line [mm, base frame], separated by
// spaces, tabs or commas; empty lines and lines starting with # are skipped.
std::expected<std::vector<Eigen::Vector3d>, std::string> read_points(const std::string& file) {
    std::ifstream in(file);
    if (!in) return std::unexpected(std::format("cannot open {}", file));
    std::vector<Eigen::Vector3d> points;
    std::string line;
    for (int number = 1; std::getline(in, line); ++number) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream words(line);
        std::array<double, 3> v{};
        std::string first;
        if (!(words >> first) || first.starts_with('#')) continue;
        words.str(line);
        words.clear();
        if (!(words >> v[0] >> v[1] >> v[2]) || !std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
            return std::unexpected(std::format("{} line {}: not three numbers: {}", file, number, line));
        std::string extra;
        if (words >> extra) return std::unexpected(std::format("{} line {}: more than three numbers", file, number));
        points.emplace_back(v[0] / 1000.0, v[1] / 1000.0, v[2] / 1000.0);
    }
    if (points.empty()) return std::unexpected(std::format("{}: no points", file));
    return points;
}

// The path, as timed TCP poses: TimedPose{time [s] from the start, T_base_tcp},
// T_base_tcp being the pose of the tool's centre point in the robot's base frame
// (ec/pose.hpp has the notation: p_base = T_base_tcp * p_tcp). It starts where the
// TCP is now, so the arm needs no move to its first point, and it keeps the
// tool's orientation. Change this to make your own: plan() takes any poses from
// rest to rest, starting at the TCP, with times from 0 increasing.
std::expected<std::vector<TimedPose>, std::string> make_path(const Eigen::Isometry3d& start, const Settings& s) {
    const Eigen::Vector3d o = start.translation();
    const double d = s.size_mm / 1000.0;
    const double speed = s.speed_mm_s / 1000.0;
    if (s.shape == "square") {  // along base +x, then +y, back along -x, -y
        return through(start,
                       {o + Eigen::Vector3d(d, 0, 0), o + Eigen::Vector3d(d, d, 0), o + Eigen::Vector3d(0, d, 0), o},
                       speed, s.ramp_s);
    }
    if (s.shape == "circle") {  // out along base +x, once round counter-clockwise seen from above, back in
        ec::Circle circle;
        circle.center = o;
        circle.radius = d;
        circle.normal = Eigen::Vector3d::UnitZ();
        circle.start = Eigen::Vector3d::UnitX();
        circle.orientation = Eigen::Quaterniond(start.linear());
        if (auto ok = validate(circle); !ok) return std::unexpected(ok.error());
        const Eigen::Isometry3d on_circle = ec::pose_at(circle, 0.0);
        std::vector<TimedPose> path{{.time = 0.0, .pose = start}};
        if (auto ok = append(path, ec::Line{.from = start, .to = on_circle}, speed, s.ramp_s); !ok)
            return std::unexpected(ok.error());
        if (auto ok = append(path, circle, speed, s.ramp_s); !ok) return std::unexpected(ok.error());
        if (auto ok = append(path, ec::Line{.from = on_circle, .to = start}, speed, s.ramp_s); !ok)
            return std::unexpected(ok.error());
        return path;
    }
    return through(start, s.points, speed, s.ramp_s);  // read in step 1
}

// The path's pose at time t: between its poses, the position moves in a
// straight line and the orientation turns evenly (slerp).
Eigen::Isometry3d pose_at_time(const std::vector<TimedPose>& path, double t) {
    if (!(t > path.front().time)) return path.front().pose;
    if (t >= path.back().time) return path.back().pose;
    const auto next =
        std::upper_bound(path.begin(), path.end(), t, [](double time, const TimedPose& p) { return time < p.time; });
    const TimedPose& b = *next;
    const TimedPose& a = *(next - 1);
    const double u = (t - a.time) / (b.time - a.time);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = a.pose.translation() + u * (b.pose.translation() - a.pose.translation());
    pose.linear() =
        Eigen::Quaterniond(a.pose.linear()).slerp(u, Eigen::Quaterniond(b.pose.linear())).toRotationMatrix();
    return pose;
}

// ------------------------------------------------------------------ watching

// How far the measured TCP strays from the path: for every monitoring frame,
// the distance to the nearest of the path's straight pieces between its poses,
// and the turn of the tool from the orientation there; the largest of each.
class PathError {
public:
    explicit PathError(const std::vector<TimedPose>& path) : path_(path) {}

    void add(const ec::feedback::State& s) {
        // The TCP from 8056 (machinePose), as a pose: position [m], rotation from rx, ry, rz.
        const Eigen::Isometry3d T_base_tcp = ec::isometry(s.tcp_position_base, s.tcp_rotation_base);
        const Eigen::Vector3d p = T_base_tcp.translation();
        double nearest = std::numeric_limits<double>::infinity();
        std::size_t at = 0;
        for (std::size_t k = 1; k < path_.size(); ++k) {
            const Eigen::Vector3d a = path_[k - 1].pose.translation();
            const Eigen::Vector3d ab = path_[k].pose.translation() - a;
            const double u = ab.squaredNorm() > 0.0 ? std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0) : 0.0;
            if (const double d = (p - (a + u * ab)).norm(); d < nearest) {
                nearest = d;
                at = k - 1;
            }
        }
        worst_ = std::max(worst_, nearest);
        worst_turn_ = std::max(worst_turn_, ec::angle_between(T_base_tcp.linear(), path_[at].pose.linear()));
        ++frames_;
    }

    void report() const {
        if (frames_ == 0) return;
        cli::out("the TCP stayed within {:.2f} mm of the path, its orientation within {:.2f} deg ({} frames)\n",
                 worst_ * 1000.0, ec::rad_to_deg(worst_turn_), frames_);
    }

private:
    const std::vector<TimedPose>& path_;
    double worst_ = 0.0;
    double worst_turn_ = 0.0;
    std::size_t frames_ = 0;
};

// ------------------------------------------------------------------ moving

// Play: robot.play() hands the trajectory to the controller's timestamp
// service. The driver's control thread uploads it (start_push_pos, a push_pos
// per point, stop_push_pos), has the controller check it (check_trajectory),
// and starts it (start_trajectory); the controller then runs it on its own
// clock. play() returns at once with a Job; the job ends when the arm rests at
// the last point, or with the reason it could not: a refusal, a lost condition
// (the E-stop, remote mode left, ...), a timeout. On any ending but arrival the
// control thread stops the arm (stop_trajectory, stop) and clears the upload.
std::expected<void, Error> play(ec::Robot& robot, JointTrajectory trajectory, PathError& error) {
    const ec::motion::Job job = robot.play(std::move(trajectory));
    std::uint64_t seq = 0;
    for (;;) {
        if (auto done = job.wait_for(0ms)) return *done;  // ended: nothing, or why
        // next_state() waits for a frame newer than `seq`: one per frame, no busy loop.
        if (const auto s = robot.next_state(seq, 20ms)) {
            seq = s->seq;
            error.add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();  // the job then ends with Kind::Stopped, the arm stopped
        }
    }
}

// Stream (--stream): instead of handing the controller the whole plan, push a
// target every 8 ms: the path's pose at that time, solved by the controller's
// inverse kinematics near the last solution, as joints into the position stream
// (transparent transmission). The stream shapes each target within speed and
// acceleration limits, and stops the arm if the targets stop coming (its
// watchdog). Closing it lets the arm settle and clears the controller's queue.
std::expected<void, Error> stream(ec::Robot& robot, const std::vector<TimedPose>& path, PathError& error) {
    // servo() opens the stream at standstill, from where the arm is, and returns
    // once the controller accepted it: nothing moves until the first target.
    auto session = robot.servo();
    if (!session) return std::unexpected(session.error());
    const auto now = robot.state();
    if (!now) return std::unexpected(Error{Error::Kind::Refused, "no state", {}});
    Joints last = now->state.joint_position;
    const auto t0 = Clock::now();
    auto next = t0;
    std::uint64_t seq = 0;
    while (session->running()) {  // false once the stream ended by itself (an error)
        const double t = std::chrono::duration<double>(Clock::now() - t0).count();
        if (t > path.back().time + 0.5) break;  // the end, and half a second for the arm to settle there
        // ik(): the controller's inverseKinematic for this pose, the solution nearest
        // `last`, in rad. One call on the command port, answered between the
        // control thread's cycles.
        auto q = robot.ik(pose_at_time(path, t), last);
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
            error.add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();  // ends the stream at once: stop, clear
        }
        next += 8ms;
        std::this_thread::sleep_until(next);
    }
    return session->close();  // nothing if it ended well, else why it ended
}

// The plan as CSV, a row per trajectory point: its time, the path's pose then
// (the controller's units: mm, rx, ry, rz in rad), and the joints [deg].
bool save(const std::string& file, const std::vector<TimedPose>& path, const JointTrajectory& t,
          std::optional<Clock::time_point> started) {
    std::ofstream f(file);
    f << "# sbx_path plan; started at steady clock "
      << (started ? std::to_string(ec::log::to_ns(*started)) + " ns" : std::string("- (not run)"))
      << ", the time base of ec_record and --record files\n";
    f << "t_s,x_mm,y_mm,z_mm,rx_rad,ry_rad,rz_rad,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg\n";
    for (std::size_t k = 0; k < t.size(); ++k) {
        const auto p = ec::to_mm_rad(pose_at_time(path, t.time[k]));
        const Joints q = ec::rad_to_deg(t.position[k]);
        f << std::format("{:.3f},{:.4f},{:.4f},{:.4f},{:.6f},{:.6f},{:.6f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n",
                         t.time[k], p[0], p[1], p[2], p[3], p[4], p[5], q[0], q[1], q[2], q[3], q[4], q[5]);
    }
    return static_cast<bool>(f);
}

// ------------------------------------------------------------------ the steps

// With the robot connected: steps 4 to 11.
int with_robot(ec::Robot& robot, const Settings& s) {
    // 4. The state now. state() is the newest monitoring frame (8056, about every
    // 8 ms), decoded and in SI units: joints [rad], the TCP pose [m, rad]. It is a
    // copy: it does not change while it is looked at.
    const auto now = robot.state();
    if (!now) return fail("state", "no frame from the monitoring port");
    const Joints here = now->state.joint_position;
    const Eigen::Isometry3d T_base_tcp = ec::isometry(now->state.tcp_position_base, now->state.tcp_rotation_base);
    cli::out("the TCP is at {}, the joints at {}\n", mm(T_base_tcp.translation()), degrees(here));
    cli::out("mode {}, state {}, conditions: {}\n", now->state.robot_mode, now->state.robot_state,
             ec::motion::to_string(robot.status().conditions));

    // 5. Do the controller's kinematics describe the TCP it reports? fk() is the
    // controller's positiveKinematic (one call) of the joints now; it must give
    // the pose 8056 reports (machinePose). If not, a tool or frame setting
    // differs between the two, and the plan would move a different point.
    const auto fk = robot.fk(here);
    if (!fk) return fail("forward kinematics", fk.error());
    const double off = (fk->translation() - T_base_tcp.translation()).norm();
    const double turn = ec::angle_between(fk->linear(), T_base_tcp.linear());
    cli::out("kinematics check: the controller's FK is {:.3f} mm and {:.3f} deg from the reported TCP\n", off * 1000.0,
             ec::rad_to_deg(turn));
    if (off > 0.001 || turn > ec::deg_to_rad(0.5))
        return fail("kinematics check", "more than 1 mm or 0.5 deg: check the tool and user frame settings");

    // 6. The path: timed TCP poses, from the TCP now.
    const auto path = make_path(T_base_tcp, s);
    if (!path) return fail("path", path.error());
    double farthest = 0.0;
    Eigen::Vector3d low = T_base_tcp.translation();
    Eigen::Vector3d high = low;
    for (const TimedPose& p : *path) {
        farthest = std::max(farthest, (p.pose.translation() - T_base_tcp.translation()).norm());
        low = low.cwiseMin(p.pose.translation());
        high = high.cwiseMax(p.pose.translation());
    }
    cli::out(
        "path: {} poses over {:.1f} s; the TCP stays within x {:.1f}..{:.1f}, y {:.1f}..{:.1f}, z "
        "{:.1f}..{:.1f} mm\n",
        path->size(), path->back().time, low.x() * 1000.0, high.x() * 1000.0, low.y() * 1000.0, high.y() * 1000.0,
        low.z() * 1000.0, high.z() * 1000.0);
    if (farthest > s.max_distance_mm / 1000.0)
        return fail("path", std::format("a point is {:.0f} mm from the start, more than --max-distance {:.0f}",
                                        farthest * 1000.0, s.max_distance_mm));

    // 7. Plan. Every pose is solved by the controller's inverse kinematics
    // (inverseKinematic, one call each, near the solution before; the first near
    // the joints now) and checked: no joint jumping more than 10 deg between
    // neighbours (a change of arm configuration), J5 and J3 at least 5 deg from 0
    // and 180 (the wrist and elbow singularities), the joint limits. The
    // solutions are interpolated over time, sampled every 20 ms, and checked for
    // joint speed and acceleration. Computations only: nothing moves.
    const auto planned_at = Clock::now();
    const auto trajectory = robot.plan(*path);
    if (!trajectory) return fail("plan", trajectory.error());
    const double took = std::chrono::duration<double>(Clock::now() - planned_at).count();

    // 8. Show the plan, and save it.
    const Joints fastest = ec::motion::peak_velocity(*trajectory);
    const auto worst = static_cast<std::size_t>(std::max_element(fastest.begin(), fastest.end()) - fastest.begin());
    const auto margins = ec::motion::singularity_margins(trajectory->position);
    cli::out("plan: {} joint points over {:.1f} s (solved in {:.1f} s); the fastest joint is J{} at {:.1f} deg/s\n",
             trajectory->size(), trajectory->duration(), took, worst + 1, ec::rad_to_deg(fastest[worst]));
    cli::out("closest to a singularity: J5 {:.1f} deg from 0 or 180 (wrist), J3 {:.1f} deg (elbow)\n",
             ec::rad_to_deg(margins.wrist), ec::rad_to_deg(margins.elbow));
    cli::out("ends at the joints {}\n", degrees(trajectory->position.back()));
    if (!s.save.empty() && !save(s.save, *path, *trajectory, std::nullopt))
        return fail("save", std::format("could not write {}", s.save));
    if (!s.yes) {
        cli::out("nothing moved: add --yes to run it ({})\n",
                 s.stream ? "streamed: a pose every 8 ms, each solved" : "played by the timestamp service");
        return cli::kOk;
    }

    // 9. Ready? The driver refuses to start a motion unless the state is fresh,
    // the controller in remote mode, without E-stop, alarm or collision, the
    // servos on, the encoders synchronised and the arm at a standstill. Bring-up
    // fixes what software may (alarm, collision, synchronisation, servos); remote
    // mode and the E-stop are a person's.
    if (const auto missing = ec::motion::kCanStartMotion.missing_from(robot.status().conditions); !missing.empty()) {
        if (!s.bring_up)
            return fail("ready", std::format("missing: {}; add --bring-up, or run ec_move --yes bring-up",
                                             ec::motion::to_string(missing)));
        cli::out("bring-up: {}\n", ec::motion::to_string(missing));
        if (const auto up = robot.bring_up().wait(); !up) return fail("bring-up", up.error());
    }

    // 10. Move.
    PathError error(*path);
    const auto started = Clock::now();
    const auto result = s.stream ? stream(robot, *path, error) : play(robot, *trajectory, error);
    if (!s.save.empty() && !save(s.save, *path, *trajectory, started))
        cli::err("could not write {}\n", s.save);  // the motion's result matters more

    // 11. Report.
    error.report();
    if (!result) return fail("motion", result.error());
    cli::out("done in {:.1f} s\n", std::chrono::duration<double>(Clock::now() - started).count());
    return cli::kOk;
}

int run(int argc, char** argv) {
    // 1. The command line (CLI11; apps/cli.hpp gives the exit codes).
    Settings s;
    CLI::App app{
        "Sandbox: a list of TCP points, planned with the controller's inverse kinematics and played "
        "(or streamed) through the driver."};
    app.add_option("--host", s.host, "Controller address")->capture_default_str();
    app.add_option("--command-port", s.command_port, "Command port")->capture_default_str();
    app.add_option("--monitor-port", s.monitor_port, "Monitoring port")->capture_default_str();
    app.add_option("--shape", s.shape, "square, circle or points")
        ->capture_default_str()
        ->check(CLI::IsMember({"square", "circle", "points"}));
    app.add_option("--size", s.size_mm, "[mm] the square's side, the circle's radius")
        ->capture_default_str()
        ->check(CLI::Range(1.0, 200.0));
    auto* points = app.add_option("--points", s.points_file, "File of points for --shape points: x y z [mm] per line");
    app.add_option("--speed", s.speed_mm_s, "[mm/s] TCP speed")->capture_default_str()->check(CLI::Range(1.0, 100.0));
    app.add_option("--ramp", s.ramp_s, "[s] to reach the speed, and to stop")
        ->capture_default_str()
        ->check(CLI::Range(0.1, 5.0));
    app.add_option("--max-distance", s.max_distance_mm, "[mm] no point further from the start")
        ->capture_default_str()
        ->check(CLI::Range(1.0, 1000.0));
    app.add_flag("--stream", s.stream, "Push the poses live, each solved, instead of playing the plan");
    app.add_flag("--bring-up", s.bring_up, "Clear an alarm, synchronise and switch the servos on if needed");
    app.add_flag("--yes", s.yes, "Really move the arm; without it, only the plan is printed");
    app.add_option("--record", s.record, "Write every monitoring frame to this raw recording");
    app.add_option("--save", s.save, "Write the plan as CSV");
    if (const auto code = cli::parse(app, argc, argv)) return *code;
    if (s.shape == "points") {  // read now: a bad file fails before anything connects
        if (points->count() == 0) {
            cli::err("--shape points needs --points FILE\n");
            return cli::kUsage;
        }
        auto read = read_points(s.points_file);
        if (!read) return fail("points", read.error());
        s.points = std::move(*read);
    }

    // 2. The recording: the receiver thread hands every frame to on_sample, which
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
        // 3. Connect. Robot::connect opens the command port (8055: a request, then
        // its reply) and the monitoring port (8056: frames the robot pushes about
        // every 8 ms), starts the thread that receives the frames and the control
        // thread that owns the command port and runs motions, and returns once the
        // first frame has come and the control thread has run its first cycle.
        // Destroying the Robot stops whatever runs, waits for the arm to rest, and
        // closes both ports.
        auto robot = ec::Robot::connect(std::move(options));
        if (!robot) return fail("connect", robot.error());
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        code = with_robot(*robot, s);
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
