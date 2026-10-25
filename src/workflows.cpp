#include "ec/workflows.hpp"

#include <format>
#include <thread>
#include <utility>

namespace ec {
namespace {

using namespace std::chrono_literals;

// The conditions, asked over the command connection, one call each.
std::expected<Conditions, Error> read_conditions(const Robot& robot, bool with_precise) {
    Conditions c;
    const auto mode = robot.system().mode();
    if (!mode) return std::unexpected(mode.error());
    c.mode = *mode;
    const auto estop = robot.safety().emergency_stop();
    if (!estop) return std::unexpected(estop.error());
    c.emergency_stop = *estop;
    const auto collision = robot.safety().collision();
    if (!collision) return std::unexpected(collision.error());
    c.collision = *collision;
    const auto state = robot.motion().state();
    if (!state) return std::unexpected(state.error());
    c.state = *state;
    const auto synced = robot.drives().synced();
    if (!synced) return std::unexpected(synced.error());
    c.synced = *synced;
    const auto servo = robot.drives().servo_on();
    if (!servo) return std::unexpected(servo.error());
    c.servo_on = *servo;
    if (with_precise) {
        const auto precise = robot.drives().precise();
        if (!precise) return std::unexpected(precise.error());
        c.precise = *precise;
    }
    return c;
}

std::string summary(const Conditions& c) {
    std::string s = std::format("mode {}, state {}, {}, servos {}{}{}", c.mode, c.state,
                                c.synced ? "synchronised" : "not synchronised", c.servo_on ? "on" : "off",
                                c.emergency_stop ? ", EMERGENCY STOP" : "", c.collision ? ", COLLISION" : "");
    if (c.precise) s += *c.precise ? ", precise" : ", not precise";
    return s;
}

// A step that sends one call; false (and the report's error) if it failed.
bool act(Report& r, const char* what, const std::expected<void, Error>& done) {
    if (done) {
        r.steps.push_back({.what = what, .outcome = "done"});
        return true;
    }
    r.steps.push_back({.what = what, .outcome = describe(done.error()), .ok = false});
    r.error = done.error();
    return false;
}

Report& stopped_at(Report& r, Blocker b, Error::Kind kind) {
    r.blocker = b;
    r.error = Error{kind, std::format("{}: {}", b, remedy(b))};
    r.steps.push_back({.what = std::string(to_string(b)), .outcome = std::string(remedy(b)), .ok = false});
    return r;
}

// Asks servo_on until it reports `on`, every 100 ms, up to `within`.
bool servo_reports(Report& r, const Robot& robot, bool on, std::chrono::milliseconds within) {
    const auto deadline = std::chrono::steady_clock::now() + within;
    for (;;) {
        std::this_thread::sleep_for(100ms);
        const auto now_on = robot.drives().servo_on();
        if (!now_on) {
            r.steps.push_back({.what = "getServoStatus", .outcome = describe(now_on.error()), .ok = false});
            r.error = now_on.error();
            return false;
        }
        if (*now_on == on) {
            r.steps.push_back(
                {.what = "getServoStatus", .outcome = on ? "the servos report on" : "the servos report off"});
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            r.error = Error{Error::Kind::Timeout, std::format("the servos did not report {} within {} ms",
                                                              on ? "on" : "off", within.count())};
            r.steps.push_back({.what = "getServoStatus", .outcome = r.error->message, .ok = false});
            return false;
        }
    }
}

}  // namespace

std::string describe(const Report& report) {
    std::string out;
    for (const Step& s : report.steps) {
        out += std::format("{} {}: {}\n", s.ok ? " " : "!", s.what, s.outcome);
        for (const auto& line : s.details) out += std::format("      {}\n", line);
    }
    if (report.ok())
        out += "done\n";
    else
        out += std::format("stopped: {}{}\n", describe(*report.error),
                           report.operator_needed() ? " (the operator's step)" : "");
    return out;
}

Report bring_up(Robot& robot, const BringUpOptions& o) {
    Report r;
    std::optional<Blocker> remedied;  // what the last step was to remove
    for (int round = 0; round < 10; ++round) {
        const auto c = read_conditions(robot, o.require_precise);
        if (!c) {
            r.steps.push_back({.what = "the conditions", .outcome = describe(c.error()), .ok = false});
            r.error = c.error();
            return r;
        }
        r.steps.push_back({.what = "the conditions", .outcome = summary(*c)});
        const Blocker b = readiness(*c);
        if (b == Blocker::None) {
            const auto slider = robot.motion().speed_scale();
            r.steps.push_back({.what = "the speed slider",
                               .outcome = slider ? std::format("{:g} %", *slider * 100.0) : describe(slider.error()),
                               .ok = slider.has_value()});
            return r;
        }
        if (remedied == b) {
            r.blocker = b;
            r.error = Error{b == Blocker::Alarm || b == Blocker::Collision ? Error::Kind::Fault : Error::Kind::Command,
                            std::format("still {} after its remedy ({})", b, remedy(b))};
            return r;
        }
        switch (b) {
            case Blocker::NotRemote:
            case Blocker::NotPrecise: return stopped_at(r, b, Error::Kind::Command);
            case Blocker::EmergencyStop: return stopped_at(r, b, Error::Kind::Fault);
            case Blocker::NotStopped: return stopped_at(r, b, Error::Kind::Busy);
            case Blocker::Collision:
                if (!act(r, "resetCollisionState", robot.safety().reset_collision())) return r;
                if (!act(r, "clearAlarm", robot.drives().clear_alarm())) return r;
                std::this_thread::sleep_for(500ms);
                break;
            case Blocker::Alarm:
                if (!act(r, "clearAlarm", robot.drives().clear_alarm())) return r;
                std::this_thread::sleep_for(500ms);
                break;
            case Blocker::NotSynced:
                if (!act(r, "syncMotorStatus", robot.drives().synchronise())) return r;
                std::this_thread::sleep_for(500ms);
                break;
            case Blocker::ServoOff:
                std::this_thread::sleep_for(500ms);  // the manual's pause before the servo
                if (!act(r, "set_servo_status 1", robot.drives().switch_servo(true))) return r;
                if (!servo_reports(r, robot, true, o.servo_within)) return r;
                break;
            case Blocker::None: break;
        }
        remedied = b;
    }
    r.error = Error{Error::Kind::Command, "bring-up did not finish in 10 steps"};
    return r;
}

Report shut_down(Robot& robot, const ShutDownOptions& o) {
    Report r;
    robot.motion().stop();
    r.steps.push_back({.what = "stop", .outcome = "the open motion's stop, then stop"});
    const auto rest = robot.motion().wait_until_stopped(o.stop_within);
    if (!rest) {
        r.error = Error{Error::Kind::Timeout,
                        std::format("the arm did not come to rest within {} ms", o.stop_within.count())};
        r.steps.push_back({.what = "at rest", .outcome = r.error->message, .ok = false});
        return r;
    }
    if (*rest)
        r.steps.push_back({.what = "at rest", .outcome = "state stop"});
    else  // in alarm or stopped by a fault: the arm stands; the servos still go off
        r.steps.push_back({.what = "at rest", .outcome = describe(rest->error()), .ok = false});
    const auto on = robot.drives().servo_on();
    if (!on) {
        r.error = on.error();
        r.steps.push_back({.what = "getServoStatus", .outcome = describe(on.error()), .ok = false});
        return r;
    }
    if (!*on) {
        r.steps.push_back({.what = "getServoStatus", .outcome = "the servos are off already"});
        return r;
    }
    if (!act(r, "set_servo_status 0", robot.drives().switch_servo(false))) return r;
    servo_reports(r, robot, false, o.servo_within);
    return r;
}

Report recover(Robot& robot, const BringUpOptions& o) {
    Report r;
    if (const auto alarms = robot.system().alarms()) {
        Step step{.what = "getAlarmNum", .outcome = "the last alarms"};
        for (const auto n : *alarms) step.details.push_back(std::to_string(n));
        r.steps.push_back(std::move(step));
    } else {
        r.steps.push_back({.what = "getAlarmNum", .outcome = describe(alarms.error()), .ok = false});
    }
    if (auto lines = robot.system().log({.level = ec::log::Level::Error, .lines = 20})) {
        r.steps.push_back({.what = "the controller's log",
                           .outcome = std::format("the last {} error lines", lines->size()),
                           .details = std::move(*lines)});
    } else {
        r.steps.push_back({.what = "the controller's log", .outcome = describe(lines.error()), .ok = false});
    }
    Report up = bring_up(robot, o);
    for (auto& step : up.steps) r.steps.push_back(std::move(step));
    r.error = std::move(up.error);
    r.blocker = up.blocker;
    return r;
}

}  // namespace ec
