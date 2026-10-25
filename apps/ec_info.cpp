// ec_info: what the controller says about itself, read only: its versions,
// its model of the arm (the DH parameters as sent, tools, payloads, user
// frames), the safety settings, the start-up flags and what a motion still
// needs. For checking the robot's parameters against its manuals (the user
// manual's DH table: docs/ec66.md §3.1).
//
//   ec_info [--host H] [--command-port N]
//
// Every read is one query on the command connection, answered before the
// next; nothing is changed, so any key-switch mode will do. A read the
// controller refuses is shown with "!" and the rest goes on. Poses are shown
// as the pendant shows them: mm and degrees.
//
// Exit codes: 0 read (failed reads are marked); 1 no connection; 2 bad command line.

#include <array>
#include <cstdint>
#include <expected>
#include <format>
#include <string>
#include <string_view>

#include "cli.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"
#include "ec/workflows.hpp"

namespace {

// "  label  value", or "! label  why" for a read that failed.
template <typename T, typename Show>
void line(std::string_view label, const std::expected<T, ec::Error>& r, Show&& show) {
    if (r)
        cli::out("  {:<22} {}\n", label, show(*r));
    else
        cli::out("! {:<22} {}\n", label, describe(r.error()));
}

std::string mm_deg(const Eigen::Isometry3d& T) {
    const auto p = ec::to_mm_rad(T);
    return std::format("x {:.3f}  y {:.3f}  z {:.3f} mm | rx {:.3f}  ry {:.3f}  rz {:.3f} deg", p[0], p[1], p[2],
                       ec::rad_to_deg(p[3]), ec::rad_to_deg(p[4]), ec::rad_to_deg(p[5]));
}

std::string robot_type(int t) {
    switch (t) {
        case 62: return "62 (six-axis collaborative)";
        case 60: return "60 (vertical articulated)";
        case 41: return "41 (four-axis)";
        case 40: return "40 (palletizing)";
        case 43: return "43 (SCARA)";
        case 30: return "30 (delta)";
        default: return std::format("{} (not in the manual's list)", t);
    }
}

std::string limit(const ec::SafetyLimit& l, double scale, std::string_view unit) {
    return std::format("{:g} {} normal, {:g} {} reduced", l.normal * scale, unit, l.reduced * scale, unit);
}

void print(const ec::Robot& robot) {
    const auto as_is = [](const auto& v) { return std::format("{}", v); };
    const auto percent = [](double f) { return std::format("{:g} %", f * 100.0); };
    const auto yes_no = [](bool b) { return std::string(b ? "yes" : "no"); };

    cli::out("controller\n");
    line("version", robot.system().version(), as_is);
    line("robot type", robot.model().robot_type(), robot_type);
    line("robot subtype", robot.model().robot_subtype(), as_is);
    line("mode (key switch)", robot.system().mode(), as_is);
    line("speed slider", robot.motion().speed_scale(), percent);

    cli::out("joints\n");
    for (int j = 1; j <= 6; ++j) {
        const auto joint = robot.joint(j);
        line(std::format("J{} position", j), joint.position(),
             [](double q) { return std::format("{:.3f} deg", ec::rad_to_deg(q)); });
        line(std::format("J{} firmware", j), joint.firmware(), as_is);
    }

    cli::out("DH parameters (getDH, as sent; the EC66 user manual: d1 96, a2 418, a3 398, d4 122, d5 98, d6 89 mm)\n");
    for (int i = 0; i < 12; ++i) line(std::format("index {} (d{})", i, i + 1), robot.model().dh(i), as_is);

    cli::out("tools (0..7)\n");
    line("REMOTE/PLAY tool", robot.model().active_tool(), as_is);
    for (int t = 0; t < 8; ++t) {
        line(std::format("tool {} TCP", t), robot.model().tool(t), mm_deg);
        line(std::format("tool {} payload", t), robot.model().payload(t), [](const ec::Payload& p) {
            const Eigen::Vector3d c = p.centre_of_mass * 1000.0;
            return std::format("{:g} kg at x {:.1f}  y {:.1f}  z {:.1f} mm", p.mass, c.x(), c.y(), c.z());
        });
    }

    cli::out("user frames (0..7)\n");
    line("current user frame", robot.model().active_user_frame(), as_is);
    for (int u = 0; u < 8; ++u) line(std::format("user frame {}", u), robot.model().user_frame(u), mm_deg);

    cli::out("safety\n");
    const auto safety = robot.safety();
    line("collision detection", safety.collision_detection(), yes_no);
    line("collision sensitivity", safety.collision_sensitivity(), percent);
    line("limits enabled", safety.limits_enabled(), yes_no);
    line("power", safety.power_limit(), [](const auto& l) { return limit(l, 1.0, "W"); });
    line("momentum", safety.momentum_limit(), [](const auto& l) { return limit(l, 1.0, "kg m/s"); });
    line("tool force", safety.tool_force_limit(), [](const auto& l) { return limit(l, 1.0, "N"); });
    line("elbow force", safety.elbow_force_limit(), [](const auto& l) { return limit(l, 1.0, "N"); });
    line("speed", safety.speed_limit(), [](const auto& l) { return limit(l, 100.0, "%"); });

    cli::out("start-up\n");
    const auto conditions = ec::read_conditions(robot, /*with_precise=*/true);
    line("conditions", conditions, [](const ec::Conditions& c) {
        return std::format("mode {}, state {}, {}, servos {}, emergency stop {}, collision {}, {}", c.mode, c.state,
                           c.synced ? "synchronised" : "not synchronised", c.servo_on ? "on" : "off",
                           c.emergency_stop ? "yes" : "no", c.collision ? "yes" : "no",
                           c.precise.value_or(false) ? "precise" : "not precise");
    });
    if (conditions) {
        const auto b = ec::readiness(*conditions);
        cli::out("  {:<22} {}\n", "before a motion",
                 b == ec::Blocker::None ? std::string("nothing is missing") : std::format("{}: {}", b, ec::remedy(b)));
    }
    line("last alarms", robot.system().alarms(), [](const std::vector<std::int64_t>& a) {
        std::string s;
        for (const auto n : a) s += std::format("{} ", n);
        return s.empty() ? std::string("none") : s;
    });
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t command_port = 8055;
    CLI::App app{"What the controller says about itself, read only: versions, model, tools, frames, safety, flags."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--command-port", command_port, "Command port")->capture_default_str()->check(CLI::Range(1, 65535));
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    const auto robot = ec::Robot::connect({.host = host, .command_port = command_port, .monitor = false});
    if (!robot) {
        cli::err("{}\n", describe(robot.error()));
        return cli::kFailed;
    }
    print(*robot);
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_info", [&] { return run(argc, argv); });
}
