#include "ec/motion/follow.hpp"

#include <format>
#include <string>
#include <utility>

namespace ec::motion {
namespace {

double seconds(std::chrono::nanoseconds d) { return std::chrono::duration<double>(d).count(); }

}  // namespace

FollowTrajectoryActivity::FollowTrajectoryActivity(FollowTrajectory spec)
    : spec_(std::move(spec)), link_("follow_trajectory", spec_.servo) {}

std::expected<void, Error> FollowTrajectoryActivity::check(const Cycle& c) const {
    const auto invalid = [](std::string message) {
        return std::unexpected(Error{Error::Kind::Invalid, "follow_trajectory: " + std::move(message), {}});
    };
    const ServoOptions& servo = spec_.servo;
    if (auto ok = validate(spec_.path, {.max_velocity = servo.max_velocity, .joint_limit = servo.joint_limit}); !ok)
        return invalid(ok.error());
    if (servo.accelerate_in.count() <= 0) return invalid("accelerate_in must be positive");
    const Joints peak = peak_acceleration(spec_.path);
    for (std::size_t j = 0; j < kJointCount; ++j) {
        const double limit = servo.max_velocity[j] / seconds(servo.accelerate_in);
        if (peak[j] > limit * (1.0 + 1e-9))
            return invalid(std::format("joint {} would accelerate at {:.1f} deg/s^2, above the stream's {:.1f}", j + 1,
                                       rad_to_deg(peak[j]), rad_to_deg(limit)));
    }
    if (!(spec_.tolerance > 0.0) || !(spec_.start_tolerance > 0.0)) return invalid("the tolerances must be positive");
    const double off = max_abs_diff(c.state.joint_position, spec_.path.position.front());
    if (off > spec_.start_tolerance)
        return invalid(std::format("the arm is {:.2f} deg from the first point; move there first", rad_to_deg(off)));
    return {};
}

std::expected<Phase, Error> FollowTrajectoryActivity::step(const Cycle& c, Outbox& out) {
    if (!checked_) {
        if (auto ok = check(c); !ok) return std::unexpected(ok.error());
        checked_ = true;
    }
    if (!link_.is_open()) {
        const auto opened = link_.open(c, out);
        if (!opened) return std::unexpected(opened.error());
        if (!*opened) return Phase::Running;
        link_.shaper().reset(spec_.path.position.front());  // at rest at the first point, which is sent first
    }
    if (auto ok = link_.check_replies(c); !ok) return std::unexpected(ok.error());
    const double t = static_cast<double>(sent_) * seconds(c.period);
    const Joints setpoint = link_.shaper().follow(position_at(spec_.path, t));
    if (auto ok = link_.check_tracking(c); !ok) return std::unexpected(ok.error());
    link_.put(out, setpoint);
    ++sent_;

    if (t < spec_.path.duration()) return Phase::Running;
    if (!finished_) finished_ = c.now;
    const auto since = c.now - *finished_;
    const double off = max_abs_diff(c.state.joint_position, spec_.path.position.back());
    if (since >= spec_.servo.lookahead + 2 * c.period && off <= spec_.tolerance && c.conditions.has(Condition::AtRest))
        return Phase::Done;
    if (since > spec_.servo.lookahead + spec_.settle)
        return std::unexpected(
            Error{Error::Kind::Tracking,
                  std::format("follow_trajectory: the arm is still {:.2f} deg from the last point", rad_to_deg(off)),
                  {}});
    return Phase::Running;
}

}  // namespace ec::motion
