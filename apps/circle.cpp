// ec_move circle: plans a circle of the TCP in joint space with the controller's
// inverse kinematics (computations only: nothing moves), prints the plan and
// points to check by hand, and with --yes draws it: a straight approach to its
// first point, then the circle, both played by the controller's timestamp
// service; or, with --stream, the circle's poses pushed live, each solved by
// the controller and sent as joints. While it draws, every monitoring frame is
// compared with the circle, and the largest deviations are reported.

#include "circle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <fstream>
#include <numbers>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
#include "ec/log/clock.hpp"
#include "ec/motion/planning.hpp"
#include "ec/path.hpp"
#include "ec/pose.hpp"

namespace circle {
namespace {

using Clock = std::chrono::steady_clock;
using ec::Joints;
using ec::motion::Error;
using ec::motion::JointTrajectory;

constexpr double kStep = 0.003;        // [m] between planned poses
constexpr double kPeriod = 0.008;      // [s] the control loop's, and the stream's
constexpr double kStreamSettle = 0.5;  // [s] a stream goes on after the circle, for the arm to settle
constexpr double kArrived = 0.0005;    // [m] closer than this to the first point: no approach
constexpr double kAgree = 0.001;       // [m] forward kinematics against the reported TCP
constexpr double kAgreeDeg = 0.5;      // [deg]

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

std::string mm(const Eigen::Vector3d& p) {
    return std::format("({:.1f}, {:.1f}, {:.1f}) mm", p.x() * 1000.0, p.y() * 1000.0, p.z() * 1000.0);
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
        const Eigen::Isometry3d T_base_tcp = ec::isometry(s.tcp_position_base, s.tcp_rotation_base);
        const Eigen::Vector3d d = T_base_tcp.translation() - circle_.center;
        const double along = d.dot(normal_);
        radial_ = std::max(radial_, std::abs((d - along * normal_).norm() - circle_.radius));
        axial_ = std::max(axial_, std::abs(along));
        turn_ = std::max(turn_, ec::angle_between(T_base_tcp.linear(), circle_.orientation.toRotationMatrix()));
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
    Eigen::Vector3d normal_;
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
        if (auto result = job.wait_for(std::chrono::milliseconds{0})) return *result;
        if (const auto s = robot.next_state(seq, std::chrono::milliseconds{20})) {
            seq = s->seq;
            if (deviation != nullptr) deviation->add(s->state);
        }
        if (interrupted()) {
            cli::err("stopping\n");
            robot.stop();
        }
    }
}

// --stream: the circle's poses as time goes, each solved near the last solution
// and pushed as joints, every period.
std::expected<void, Error> stream(ec::Robot& robot, const ec::Circle& c, const ec::motion::SpeedProfile& profile,
                                  Deviation& deviation, const std::function<bool()>& interrupted) {
    auto session = robot.servo();
    if (!session) return std::unexpected(session.error());
    const auto now = robot.state();
    if (!now) return std::unexpected(Error{Error::Kind::Refused, "no state", {}});
    Joints last = now->state.joint_position;
    const auto t0 = Clock::now();
    auto next = t0;
    std::uint64_t seq = 0;
    while (session->running()) {
        const double t = seconds(Clock::now() - t0);
        if (t > profile.duration() + kStreamSettle) break;
        auto q = robot.ik(ec::pose_at(c, profile.distance(t)), last);
        if (!q) {
            robot.stop();  // ends the stream at once: there is no next target
            const auto closed = session->close();
            static_cast<void>(closed);  // stopped: the error that matters is the pose's
            return std::unexpected(std::move(q.error()));
        }
        last = *q;
        session->target(last);
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

// The plan as CSV: a row per trajectory point, the planned TCP pose and the joints.
bool save(const std::string& file, const ec::Circle& c, const ec::motion::SpeedProfile& profile,
          const JointTrajectory& trajectory, std::optional<Clock::time_point> started) {
    std::ofstream f(file);
    f << std::format(
        "# ec_move circle: centre {}, radius {:.1f} mm, normal ({:.3f}, {:.3f}, {:.3f}), {:g} turns, "
        "{:.1f} mm/s\n",
        mm(c.center), c.radius * 1000.0, c.normal.x(), c.normal.y(), c.normal.z(), c.turns,
        profile.cruise_speed() * 1000.0);
    f << "# started at steady clock "
      << (started ? std::to_string(ec::log::to_ns(*started)) + " ns" : std::string("- (not run)"))
      << ", the time base of ec_record and ec_move --record files\n";
    f << "t_s,x_mm,y_mm,z_mm,rx_rad,ry_rad,rz_rad,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg\n";
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        const double t = trajectory.time[k];
        const auto p = ec::to_mm_rad(ec::pose_at(c, profile.distance(t)));
        const Joints q = ec::rad_to_deg(trajectory.position[k]);
        f << std::format("{:.3f},{:.4f},{:.4f},{:.4f},{:.6f},{:.6f},{:.6f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n",
                         t, p[0], p[1], p[2], p[3], p[4], p[5], q[0], q[1], q[2], q[3], q[4], q[5]);
    }
    return static_cast<bool>(f);
}

}  // namespace

int run(ec::Robot& robot, const Options& o, const std::function<bool()>& interrupted) {
    const auto now = robot.state();
    if (!now) {
        cli::err("no state from the monitoring port\n");
        return cli::kFailed;
    }
    const Joints here = now->state.joint_position;
    const Eigen::Isometry3d T_base_tcp = ec::isometry(now->state.tcp_position_base, now->state.tcp_rotation_base);

    // The circle is planned with the controller's kinematics but checked against
    // the TCP it reports: the two must be the same tool.
    const auto fk = robot.fk(here);
    if (!fk) return fail(fk.error(), "forward kinematics");
    const double fk_off = (fk->translation() - T_base_tcp.translation()).norm();
    const double fk_turn = ec::angle_between(fk->linear(), T_base_tcp.linear());
    if (fk_off > kAgree || fk_turn > ec::deg_to_rad(kAgreeDeg)) {
        cli::err(
            "the controller's forward kinematics put the TCP {:.2f} mm and {:.2f} deg from the TCP it reports: "
            "a tool or user frame differs; nothing planned\n",
            fk_off * 1000.0, ec::rad_to_deg(fk_turn));
        return cli::kFailed;
    }

    ec::Circle c;
    c.center = o.center_mm ? Eigen::Vector3d(*o.center_mm * 0.001) : Eigen::Vector3d(T_base_tcp.translation());
    c.radius = o.radius_mm / 1000.0;
    c.normal = o.normal;
    c.start = o.start_toward ? *o.start_toward : Eigen::Vector3d(T_base_tcp.translation() - c.center);
    c.orientation = Eigen::Quaterniond(T_base_tcp.linear());
    c.turns = o.turns;
    if (const auto ok = validate(c); !ok) {
        cli::err("not a circle: {}\n", ok.error());
        return cli::kUsage;
    }
    const double total = length(c);
    const ec::CircleAxes axes = ec::axes(c);
    const Eigen::Isometry3d first = ec::pose_at(c, 0.0);
    const double gap = (T_base_tcp.translation() - first.translation()).norm();
    const Eigen::Vector3d rxyz = ec::rxyz_from_rotation(first.linear());

    cli::out("circle: centre {}, radius {:.1f} mm, {:g} turn{} counter-clockwise about ({:.3f}, {:.3f}, {:.3f})\n",
             mm(c.center), c.radius * 1000.0, c.turns, c.turns == 1.0 ? "" : "s", axes.n.x(), axes.n.y(), axes.n.z());
    cli::out("tool orientation kept at rx, ry, rz = ({:.2f}, {:.2f}, {:.2f}) deg (the TCP's now)\n",
             ec::rad_to_deg(rxyz.x()), ec::rad_to_deg(rxyz.y()), ec::rad_to_deg(rxyz.z()));
    cli::out("points to check: start (0 deg) {}, 90 deg {}, 180 deg {}, 270 deg {}\n", mm(first.translation()),
             mm(ec::pose_at(c, c.radius * std::numbers::pi / 2.0).translation()),
             mm(ec::pose_at(c, c.radius * std::numbers::pi).translation()),
             mm(ec::pose_at(c, c.radius * std::numbers::pi * 1.5).translation()));
    cli::out("the TCP is at {}, {:.1f} mm from the start\n", mm(T_base_tcp.translation()), gap * 1000.0);
    if (gap > o.max_approach_mm / 1000.0) {
        cli::err("the start is more than {:.0f} mm away: move closer first, or raise --max-approach\n",
                 o.max_approach_mm);
        return cli::kFailed;
    }

    // Plan: the approach (a straight line), then the circle, each solved by the controller.
    const ec::motion::InverseKinematics ik = [&robot](const Eigen::Isometry3d& T, const Joints& near) {
        return robot.ik(T, near);
    };
    std::optional<JointTrajectory> approach;
    Joints start = here;
    if (gap > kArrived) {
        const ec::Line line{.from = T_base_tcp, .to = first};
        const auto profile = ec::motion::SpeedProfile::make(gap, o.approach_speed_mm_s / 1000.0, o.ramp_s);
        if (!profile) return fail(Error{Error::Kind::Invalid, profile.error(), {}}, "approach");
        auto planned = ec::motion::plan(ec::motion::sample(line, *profile, kStep), here, ik);
        if (!planned) return fail(planned.error(), "planning the approach");
        start = planned->position.back();
        cli::out("approach: a straight line of {:.1f} mm at {:.1f} mm/s, {:.1f} s\n", gap * 1000.0,
                 profile->cruise_speed() * 1000.0, profile->duration());
        approach = std::move(*planned);
    }
    const auto profile = ec::motion::SpeedProfile::make(total, o.speed_mm_s / 1000.0, o.ramp_s);
    if (!profile) return fail(Error{Error::Kind::Invalid, profile.error(), {}}, "timing");
    const auto poses = ec::motion::sample(c, *profile, kStep);
    cli::out("solving {} poses with the controller's inverse kinematics...\n", poses.size());
    const auto trajectory = ec::motion::plan(poses, start, ik);
    if (!trajectory) return fail(trajectory.error(), "planning");
    const Joints fastest = ec::motion::peak_velocity(*trajectory);
    const auto worst = static_cast<std::size_t>(std::max_element(fastest.begin(), fastest.end()) - fastest.begin());
    const auto margins = ec::motion::singularity_margins(trajectory->position);
    cli::out("timing: {:.1f} mm/s{}, {:.1f} s ramps, {:.1f} s in all\n", profile->cruise_speed() * 1000.0,
             profile->cruise_speed() * 1000.0 < o.speed_mm_s - 1e-6 ? " (lowered: the circle is short)" : "", o.ramp_s,
             profile->duration());
    cli::out("joints at the start {}; the fastest is J{} at {:.1f} deg/s\n", degrees(start), worst + 1,
             ec::rad_to_deg(fastest[worst]));
    cli::out("closest to a singularity: J5 {:.1f} deg from 0 or 180 (wrist), J3 {:.1f} deg (elbow)\n",
             ec::rad_to_deg(margins.wrist), ec::rad_to_deg(margins.elbow));
    if (o.stream)
        cli::out("drawn by: the poses pushed every {:.0f} ms, each solved by the controller and streamed as joints\n",
                 kPeriod * 1000.0);
    else
        cli::out("drawn by: {} joint points {:.0f} ms apart, uploaded to the controller's timestamp service\n",
                 trajectory->size(), (trajectory->time[1] - trajectory->time[0]) * 1000.0);

    if (!o.yes) {
        if (!o.save.empty() && !save(o.save, c, *profile, *trajectory, std::nullopt)) {
            cli::err("could not write {}\n", o.save);
            return cli::kFailed;
        }
        cli::out("nothing moved: add --yes to run it\n");
        return cli::kOk;
    }

    if (approach) {
        cli::out("approach...\n");
        const auto approached = watch(robot, robot.play(std::move(*approach)), nullptr, interrupted);
        if (!approached) return fail(approached.error(), "approach");
    }
    Deviation deviation(c);
    const auto started = Clock::now();
    const auto result = o.stream ? stream(robot, c, *profile, deviation, interrupted)
                                 : watch(robot, robot.play(*trajectory), &deviation, interrupted);
    const double took = seconds(Clock::now() - started);
    if (!o.save.empty() && !save(o.save, c, *profile, *trajectory, started)) cli::err("could not write {}\n", o.save);
    deviation.report();
    if (!result) return fail(result.error(), "circle");
    cli::out("circle: done in {:.1f} s\n", took);
    return cli::kOk;
}

}  // namespace circle
