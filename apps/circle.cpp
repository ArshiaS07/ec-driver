// ec_move circle: plans a circle of the TCP with the controller's inverse
// kinematics (computations only: nothing moves), prints the plan and points to
// check by hand, and with --yes draws it in the chosen mode, after a straight
// approach to its first point. While it draws, every monitoring frame is
// compared with the circle, and the largest deviations are reported.

#include "circle.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
#include "ec/log/clock.hpp"
#include "ec/motion/follow.hpp"
#include "ec/motion/planning.hpp"
#include "ec/motion/point_to_point.hpp"
#include "ec/motion/sequence.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/motion/waypoints.hpp"
#include "ec/path.hpp"

namespace circle {
namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using ec::Joints;
using ec::Pose;
using ec::Vec3;
using ec::motion::Error;

constexpr std::array<std::pair<Mode, std::string_view>, 7> kModes{{{Mode::Arc, "arc"},
                                                                   {Mode::Path, "path"},
                                                                   {Mode::Lines, "lines"},
                                                                   {Mode::Trajectory, "trajectory"},
                                                                   {Mode::Follow, "follow"},
                                                                   {Mode::PoseStream, "pose-stream"},
                                                                   {Mode::Velocity, "velocity"}}};

constexpr double kPeriod = 0.008;            // [s] the control loop's, and so the streams'
constexpr double kUploadPeriod = 0.02;       // [s] between the points of an uploaded trajectory
constexpr double kStreamSettle = 0.5;        // [s] a stream goes on after the circle, for the arm to settle
constexpr double kArrived = 0.0005;          // [m] closer than this to the first point: no approach
constexpr int kPathSegments = 36;            // per turn
constexpr int kLineSegments = 12;            // per turn
constexpr double kBlendShare = 0.3;          // a path's blend radius, as a share of its segments
constexpr double kKinematicsAgree = 0.001;   // [m] forward kinematics against the reported TCP
constexpr double kKinematicsAgreeDeg = 0.5;  // [deg]

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

std::string mm(const Vec3& p) {
    return std::format("({:.1f}, {:.1f}, {:.1f}) mm", p[0] * 1000.0, p[1] * 1000.0, p[2] * 1000.0);
}

std::string degrees(const Vec3& r) {
    return std::format("({:.2f}, {:.2f}, {:.2f}) deg", ec::rad_to_deg(r[0]), ec::rad_to_deg(r[1]),
                       ec::rad_to_deg(r[2]));
}

std::string degrees(const Joints& q) {
    std::string s = "[";
    for (std::size_t j = 0; j < q.size(); ++j) s += std::format("{}{:.2f}", j == 0 ? "" : ", ", ec::rad_to_deg(q[j]));
    return s + "] deg";
}

int fail(const Error& e, std::string_view what) {
    cli::err("{}: {}\n", what, describe(e));
    return cli::kFailed;
}

// How far the TCP strays from the circle: from its radius, from its plane, and
// from the tool orientation.
class Deviation {
public:
    explicit Deviation(const ec::Circle& c) : circle_(c), normal_(ec::axes(c).n) {}

    void add(const ec::feedback::State& s) {
        const Vec3 d = ec::sub(s.tcp_position_base, circle_.center);
        const double along = ec::dot(d, normal_);
        const double out = ec::norm(ec::sub(d, ec::mul(normal_, along)));
        radial_ = std::max(radial_, std::abs(out - circle_.radius));
        axial_ = std::max(axial_, std::abs(along));
        turn_ = std::max(turn_, ec::rotation_between(s.tcp_rotation_base, circle_.rotation));
        ++frames_;
    }

    void report() const {
        if (frames_ == 0) return;
        cli::out(
            "the TCP stayed within {:.2f} mm of the circle and {:.2f} mm of its plane, its orientation within "
            "{:.2f} deg ({} frames)\n",
            radial_ * 1000.0, axial_ * 1000.0, ec::rad_to_deg(turn_), frames_);
    }

private:
    ec::Circle circle_;
    Vec3 normal_;
    double radial_ = 0.0;
    double axial_ = 0.0;
    double turn_ = 0.0;
    std::size_t frames_ = 0;
};

// Waits for `job`, adding every frame to `deviation` (if any); Ctrl+C stops the arm.
std::expected<void, Error> watch(ec::Robot& robot, const ec::motion::Job& job, Deviation* deviation,
                                 const std::function<bool()>& interrupted) {
    std::uint64_t seq = 0;
    for (;;) {
        if (auto result = job.wait_for(0ms)) return *result;
        if (const auto s = robot.next_state(seq, 20ms)) {
            seq = s->seq;
            if (deviation != nullptr) deviation->add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();
        }
    }
}

// Pose-stream mode: the poses of the circle as time goes, pushed every period.
std::expected<void, Error> stream_poses(ec::Robot& robot, const ec::Circle& c, const ec::motion::SpeedProfile& profile,
                                        Deviation& deviation, const std::function<bool()>& interrupted) {
    auto session = robot.servo_pose();
    if (!session) return std::unexpected(session.error());
    const auto t0 = Clock::now();
    auto next = t0;
    std::uint64_t seq = 0;
    while (session->running()) {
        const double t = seconds(Clock::now() - t0);
        if (t > profile.duration() + kStreamSettle) break;
        session->target(pose_at(c, profile.distance(t)));
        if (const auto s = robot.state(); s && s->seq != seq) {
            seq = s->seq;
            deviation.add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();
        }
        next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(kPeriod));
        std::this_thread::sleep_until(next);
    }
    return session->close();
}

// Velocity mode: the planned joint velocities plus `gain` times the position
// error, pushed every period; after the circle, the correction alone until the
// arm is at the end.
std::expected<void, Error> stream_velocities(ec::Robot& robot, const ec::motion::TimedPath& path, double gain,
                                             Deviation& deviation, const std::function<bool()>& interrupted) {
    auto session = robot.velocity();
    if (!session) return std::unexpected(session.error());
    const Joints end = path.position(path.duration());
    const auto t0 = Clock::now();
    auto next = t0;
    std::uint64_t seq = 0;
    while (session->running()) {
        const double t = seconds(Clock::now() - t0);
        const auto s = robot.state();
        if (!s) break;
        if (s->seq != seq) {
            seq = s->seq;
            deviation.add(s->state);
        }
        const Joints& q = s->state.joint_position;
        if (t > path.duration() && (ec::max_abs_diff(q, end) < ec::deg_to_rad(0.05) || t > path.duration() + 2.0))
            break;
        const Joints wanted = path.position(t);
        const Joints feed = path.velocity(t);
        Joints v{};
        for (std::size_t j = 0; j < v.size(); ++j) v[j] = feed[j] + gain * (wanted[j] - q[j]);
        session->target(v);
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();
        }
        next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(kPeriod));
        std::this_thread::sleep_until(next);
    }
    return session->close();
}

// The plan as CSV: a row every period, the pose and the joints.
bool save(const std::string& file, const Options& o, const ec::Circle& c, const ec::motion::TimedPath& path,
          std::optional<Clock::time_point> started) {
    std::ofstream f(file);
    f << std::format("# ec_move circle {}: centre {}, radius {:.1f} mm, normal ({}, {}, {}), {} turns, {:.1f} mm/s\n",
                     to_string(o.mode), mm(c.center), c.radius * 1000.0, c.normal[0], c.normal[1], c.normal[2], c.turns,
                     o.speed_mm_s);
    f << "# started at steady clock "
      << (started ? std::to_string(ec::log::to_ns(*started)) + " ns" : std::string("- (not run)"))
      << ", the time base of ec_record and ec_move --record files\n";
    f << "t_s,x_mm,y_mm,z_mm,rx_rad,ry_rad,rz_rad,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg\n";
    const double end = path.duration();
    for (std::size_t k = 0;; ++k) {
        const double t = std::min(end, kPeriod * static_cast<double>(k));
        const Pose p = pose_at(c, path.profile().distance(t));
        const Joints q = ec::rad_to_deg(path.position(t));
        f << std::format("{:.3f},{:.4f},{:.4f},{:.4f},{:.6f},{:.6f},{:.6f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n",
                         t, p.position[0] * 1000.0, p.position[1] * 1000.0, p.position[2] * 1000.0, p.rotation[0],
                         p.rotation[1], p.rotation[2], q[0], q[1], q[2], q[3], q[4], q[5]);
        if (t >= end) break;
    }
    return static_cast<bool>(f);
}

// The joints of the circle's pose at distance s, solved near the planned ones there.
std::expected<Joints, Error> joints_at(ec::Robot& robot, const ec::Circle& c, const ec::motion::JointPath& path,
                                       double s) {
    return robot.inverse_kinematics(pose_at(c, s), path.joints.position(s));
}

}  // namespace

std::optional<Mode> parse_mode(std::string_view name) {
    for (const auto& [mode, text] : kModes)
        if (text == name) return mode;
    return std::nullopt;
}

std::string_view to_string(Mode mode) {
    for (const auto& [m, text] : kModes)
        if (m == mode) return text;
    return "?";
}

int run(ec::Robot& robot, const Options& o, const std::function<bool()>& interrupted) {
    const auto now = robot.state();
    if (!now) {
        cli::err("no state from the monitoring port\n");
        return cli::kFailed;
    }
    const Joints here = now->state.joint_position;
    const Pose tcp{now->state.tcp_position_base, now->state.tcp_rotation_base};

    // The circle is planned with the controller's kinematics but checked against
    // the TCP it reports: the two must be the same tool.
    const auto fk = robot.forward_kinematics(here);
    if (!fk) return fail(fk.error(), "forward kinematics");
    const double fk_off = ec::distance(fk->position, tcp.position);
    const double fk_turn = ec::rotation_between(fk->rotation, tcp.rotation);
    if (fk_off > kKinematicsAgree || fk_turn > ec::deg_to_rad(kKinematicsAgreeDeg)) {
        cli::err(
            "the controller's forward kinematics put the TCP {:.2f} mm and {:.2f} deg from the TCP it reports: "
            "a tool or user frame differs; nothing planned\n",
            fk_off * 1000.0, ec::rad_to_deg(fk_turn));
        return cli::kFailed;
    }

    ec::Circle c;
    c.center = o.center_mm ? ec::mul(*o.center_mm, 0.001) : tcp.position;
    c.radius = o.radius_mm / 1000.0;
    c.normal = o.normal;
    c.start = o.start_toward ? *o.start_toward : ec::sub(tcp.position, c.center);
    c.rotation = tcp.rotation;
    c.turns = o.turns;
    if (const auto ok = validate(c); !ok) {
        cli::err("not a circle: {}\n", ok.error());
        return cli::kUsage;
    }
    const double total = length(c);
    const ec::CircleAxes axes = ec::axes(c);
    const Pose first = pose_at(c, 0.0);
    const double gap = ec::distance(tcp.position, first.position);

    cli::out("circle: centre {}, radius {:.1f} mm, {:g} turn{} counter-clockwise about ({:.3f}, {:.3f}, {:.3f})\n",
             mm(c.center), c.radius * 1000.0, c.turns, c.turns == 1.0 ? "" : "s", axes.n[0], axes.n[1], axes.n[2]);
    cli::out("tool orientation kept at {} (rx, ry, rz: the TCP's now)\n", degrees(c.rotation));
    cli::out("points to check: start (0 deg) {}, 90 deg {}, 180 deg {}, 270 deg {}\n", mm(first.position),
             mm(pose_at(c, c.radius * std::numbers::pi / 2.0).position),
             mm(pose_at(c, c.radius * std::numbers::pi).position),
             mm(pose_at(c, c.radius * std::numbers::pi * 1.5).position));
    cli::out("the TCP is at {}, {:.1f} mm from the start\n", mm(tcp.position), gap * 1000.0);
    if (gap > o.max_approach_mm / 1000.0) {
        cli::err("the start is more than {:.0f} mm away: move closer first, or raise --max-approach\n",
                 o.max_approach_mm);
        return cli::kFailed;
    }

    // Solve and time the circle.
    const ec::motion::InverseKinematics ik = [&robot](const Pose& p, const Joints& reference) {
        return robot.inverse_kinematics(p, reference);
    };
    const ec::motion::PlanOptions plan{};
    cli::out("solving it with the controller's inverse kinematics every {:.0f} mm ({} points)...\n",
             plan.spacing * 1000.0, static_cast<std::size_t>(std::ceil(total / plan.spacing)) + 1);
    auto solved = ec::motion::solve(c, here, ik, plan);
    if (!solved) return fail(solved.error(), "planning");
    auto timed = ec::motion::TimedPath::make(std::move(*solved), o.speed_mm_s / 1000.0, o.ramp_s);
    if (!timed) return fail(timed.error(), "timing");
    const ec::motion::JointPath& path = timed->path();
    const Joints start = path.joints.positions().front();
    const ec::motion::JointTrajectory fine = timed->sample(kPeriod);
    const Joints fastest = ec::motion::peak_velocity(fine);
    const auto worst = static_cast<std::size_t>(std::max_element(fastest.begin(), fastest.end()) - fastest.begin());
    cli::out("timing: {:.1f} mm/s{}, {:.1f} s ramps, {:.1f} s in all\n", timed->profile().cruise_speed() * 1000.0,
             timed->profile().cruise_speed() * 1000.0 < o.speed_mm_s - 1e-6 ? " (lowered: the circle is short)" : "",
             o.ramp_s, timed->duration());
    cli::out("joints at the start {}; the fastest is J{} at {:.1f} deg/s\n", degrees(start), worst + 1,
             ec::rad_to_deg(fastest[worst]));

    // What the mode sends.
    std::optional<ec::motion::AnyActivity> drawing;
    const double speed = o.speed_mm_s / 1000.0;
    switch (o.mode) {
        case Mode::Arc: {
            const int arcs =
                std::max(1, static_cast<int>(std::ceil(c.turns * 2.0 - 1e-9)));  // at most half a turn each
            std::vector<ec::motion::AnyActivity> steps;
            for (int i = 0; i < arcs; ++i) {
                const auto middle = joints_at(robot, c, path, total * (i + 0.5) / arcs);
                const auto end = joints_at(robot, c, path, total * (i + 1.0) / arcs);
                if (!middle) return fail(middle.error(), "planning the arcs");
                if (!end) return fail(end.error(), "planning the arcs");
                steps.emplace_back(ec::motion::MoveArc{.middle = *middle, .target = *end, .speed = speed});
            }
            cli::out("mode arc: {} moveByArc of {:.0f} deg each, at {:.1f} mm/s (the controller's own arcs)\n", arcs,
                     360.0 * c.turns / arcs, o.speed_mm_s);
            drawing.emplace(ec::motion::Sequence(std::move(steps)));
            break;
        }
        case Mode::Path: {
            const int n =
                std::max(3, static_cast<int>(std::lround((o.segments > 0 ? o.segments : kPathSegments) * c.turns)));
            const double chord = 2.0 * c.radius * std::sin(std::numbers::pi * c.turns / n);
            ec::motion::MovePath m;
            for (int k = 1; k <= n; ++k) {
                const auto q = joints_at(robot, c, path, total * k / n);
                if (!q) return fail(q.error(), "planning the waypoints");
                m.points.push_back({.position = *q,
                                    .move = ec::motion::PathMove::Line,
                                    .speed = speed,
                                    .blend_radius = k == n ? 0.0 : kBlendShare * chord});
            }
            cli::out(
                "mode path: {} waypoints joined by lines of {:.1f} mm, corners blended within {:.1f} mm, at "
                "{:.1f} mm/s (moveByPath)\n",
                n, chord * 1000.0, kBlendShare * chord * 1000.0, o.speed_mm_s);
            drawing.emplace(std::move(m));
            break;
        }
        case Mode::Lines: {
            const int n =
                std::max(3, static_cast<int>(std::lround((o.segments > 0 ? o.segments : kLineSegments) * c.turns)));
            ec::motion::Sequence lines;
            for (int k = 1; k <= n; ++k)
                lines.then(ec::motion::MoveLineToPose{.target = pose_at(c, total * k / n), .speed = speed});
            cli::out(
                "mode lines: {} straight lines to poses, stopping at each corner, at {:.1f} mm/s "
                "(moveByLineCoord)\n",
                n, o.speed_mm_s);
            drawing.emplace(std::move(lines));
            break;
        }
        case Mode::Trajectory: {
            ec::motion::JointTrajectory upload = timed->sample(kUploadPeriod);
            cli::out("mode trajectory: {} joint points {:.0f} ms apart, uploaded, then run on the controller's clock\n",
                     upload.size(), kUploadPeriod * 1000.0);
            drawing.emplace(ec::motion::PlayTrajectory{.path = std::move(upload)});
            break;
        }
        case Mode::Follow:
        case Mode::PoseStream:
        case Mode::Velocity: {
            const ec::motion::ServoOptions servo{};
            const Joints accel = ec::motion::peak_acceleration(fine);
            for (std::size_t j = 0; j < fastest.size(); ++j) {
                const double accel_limit = servo.max_velocity[j] / seconds(servo.accelerate_in);
                if (fastest[j] > servo.max_velocity[j] || accel[j] > accel_limit) {
                    cli::err(
                        "too fast for a stream: J{} would move at {:.1f} deg/s and {:.1f} deg/s^2; streams allow "
                        "{:.1f} and {:.1f}: lower --speed or lengthen --ramp\n",
                        j + 1, ec::rad_to_deg(fastest[j]), ec::rad_to_deg(accel[j]),
                        ec::rad_to_deg(servo.max_velocity[j]), ec::rad_to_deg(accel_limit));
                    return cli::kFailed;
                }
            }
            if (o.mode == Mode::Follow) {
                cli::out(
                    "mode follow: {} joint points streamed by the PC, one every {:.0f} ms (transparent "
                    "transmission, lookahead {} ms)\n",
                    fine.size(), kPeriod * 1000.0, servo.lookahead.count());
                drawing.emplace(ec::motion::FollowTrajectory{.path = fine});
            } else if (o.mode == Mode::PoseStream) {
                cli::out(
                    "mode pose-stream: the circle's poses pushed every {:.0f} ms, each solved by the controller "
                    "and streamed (transparent transmission)\n",
                    kPeriod * 1000.0);
            } else {
                cli::out(
                    "mode velocity: joint velocities pushed every {:.0f} ms (moveBySpeedj), the planned ones "
                    "plus {:g}/s times the position error\n",
                    kPeriod * 1000.0, o.gain);
            }
            break;
        }
    }

    if (!o.yes) {
        if (!o.save.empty() && !save(o.save, o, c, *timed, std::nullopt)) {
            cli::err("could not write {}\n", o.save);
            return cli::kFailed;
        }
        cli::out("nothing moved: add --yes to run it\n");
        return cli::kOk;
    }

    // The approach: a straight line to the first point.
    if (gap > kArrived || ec::max_abs_diff(here, start) > ec::deg_to_rad(0.2)) {
        cli::out("approach: a straight line of {:.1f} mm at {:.1f} mm/s\n", gap * 1000.0, o.approach_speed_mm_s);
        const auto approached =
            watch(robot, robot.run(ec::motion::MoveLine{.target = start, .speed = o.approach_speed_mm_s / 1000.0}),
                  nullptr, interrupted);
        if (!approached) return fail(approached.error(), "approach");
    }

    // The circle.
    Deviation deviation(c);
    const auto started = Clock::now();
    std::expected<void, Error> result;
    if (drawing) {
        result = watch(robot, robot.run(std::move(*drawing)), &deviation, interrupted);
    } else if (o.mode == Mode::PoseStream) {
        result = stream_poses(robot, c, timed->profile(), deviation, interrupted);
    } else {
        result = stream_velocities(robot, *timed, o.gain, deviation, interrupted);
    }
    const double took = seconds(Clock::now() - started);
    if (!o.save.empty() && !save(o.save, o, c, *timed, started)) cli::err("could not write {}\n", o.save);
    deviation.report();
    if (!result) return fail(result.error(), std::format("circle ({})", to_string(o.mode)));
    cli::out("circle ({}): done in {:.1f} s\n", to_string(o.mode), took);
    return cli::kOk;
}

}  // namespace circle
