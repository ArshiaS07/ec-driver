#include "ec/motion/point_to_point.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <utility>

#include "ec/command/movement.hpp"
#include "ec/command/reply.hpp"

namespace ec::motion {
namespace {

// The arm counts as stopped short once it rests elsewhere for this many cycles.
constexpr int kShortCycles = 3;

bool is_fraction(double v) noexcept { return std::isfinite(v) && v > 0.0 && v <= 1.0; }

std::optional<std::string> check_fraction(std::string_view what, const std::optional<double>& v) {
    if (v && !is_fraction(*v)) return std::format("{} must be in (0, 1], not {}", what, *v);
    return std::nullopt;
}

std::optional<int> percent(const std::optional<double>& fraction) {
    if (!fraction) return std::nullopt;
    return std::max(1, static_cast<int>(std::lround(*fraction * 100.0)));
}

std::optional<std::string> check_joints(const Joints& q) {
    for (std::size_t i = 0; i < q.size(); ++i)
        if (!std::isfinite(q[i]) || std::abs(q[i]) > kEc66JointLimit)
            return std::format("joint {} target {} rad is outside +-360 deg", i + 1, q[i]);
    return std::nullopt;
}

// Where the arm is short of joint positions, or nullopt once within `tolerance`.
PointToPoint::Remaining joints_remaining(const Joints& target, double tolerance) {
    return [target, tolerance](const feedback::State& s) -> std::optional<std::string> {
        const double off = max_abs_diff(s.joint_position, target);
        if (off <= tolerance) return std::nullopt;
        return std::format("{:.2f} deg off on its worst joint", rad_to_deg(off));
    };
}

std::optional<std::string> check_linear(const std::optional<double>& acc, const std::optional<double>& dec,
                                        double speed) {
    if (!std::isfinite(speed) || speed < 0.001 || speed > 3.0)
        return std::format("speed must be in [0.001, 3] m/s, not {}", speed);
    if (auto e = check_fraction("acceleration", acc)) return e;
    return check_fraction("deceleration", dec);
}

}  // namespace

PointToPoint::PointToPoint(std::string_view name, command::Request request, Remaining remaining,
                           std::chrono::milliseconds timeout)
    : name_(name), request_(std::move(request)), remaining_(std::move(remaining)), timeout_(timeout) {}

PointToPoint PointToPoint::invalid(std::string_view name, std::string why) {
    PointToPoint p(name, command::Request{}, {}, std::chrono::milliseconds{0});
    p.invalid_ = std::move(why);
    return p;
}

std::expected<Phase, Error> PointToPoint::step(const Cycle& c, Outbox& out) {
    switch (stage_) {
        case Stage::Send:
            if (invalid_)
                return std::unexpected(Error{Error::Kind::Invalid, std::format("{}: {}", name_, *invalid_), {}});
            out.send(request_);
            sent_at_ = c.now;
            stage_ = Stage::AwaitReply;
            return Phase::Running;

        case Stage::AwaitReply: {
            const Reply* reply = nullptr;
            for (const Reply& r : c.replies)
                if (r.method == request_.method) reply = &r;
            if (reply == nullptr) {
                if (c.now - sent_at_ > timeout_)
                    return std::unexpected(Error{Error::Kind::Timeout, std::format("{}: no reply", name_), {}});
                return Phase::Running;
            }
            if (!reply->result) return std::unexpected(command_failed(request_.method, reply->result.error()));
            if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*reply->result); !ok)
                return std::unexpected(command_failed(request_.method, ok.error()));
            stage_ = Stage::Moving;
            return Phase::Running;
        }

        case Stage::Moving: {
            if (c.state.robot_state == RobotState::Running || !c.conditions.has(Condition::AtRest)) moved_ = true;
            const auto left = remaining_(c.state);
            const bool resting = c.conditions.has(Condition::Standstill);
            if (resting && !left) return Phase::Done;
            if (resting && moved_ && ++short_cycles_ >= kShortCycles)
                return std::unexpected(
                    Error{Error::Kind::Command, std::format("{}: the arm stopped {}", name_, *left), {}});
            if (!resting) short_cycles_ = 0;
            if (c.now - sent_at_ > timeout_)
                return std::unexpected(
                    Error{Error::Kind::Timeout,
                          std::format("{}: not at the target after {} s{}", name_,
                                      std::chrono::duration_cast<std::chrono::seconds>(timeout_).count(),
                                      left ? ", " + *left : std::string{}),
                          {}});
            return Phase::Running;
        }
    }
    return Phase::Running;
}

bool PointToPoint::close(const Cycle& c, Outbox& out, End why) {
    if (why == End::Done || stage_ == Stage::Send) return true;  // arrived, or never sent
    if (!stop_sent_at_) {
        out.send(command::Stop{});
        stop_sent_at_ = c.now;
        return false;
    }
    return c.now > *stop_sent_at_ && c.conditions.has(Condition::AtRest);
}

PointToPoint make_activity(const MoveJoint& m) {
    constexpr std::string_view name = "move_joint";
    if (auto e = check_joints(m.target)) return PointToPoint::invalid(name, *e);
    if (!is_fraction(m.speed))
        return PointToPoint::invalid(name, std::format("speed must be in (0, 1], not {}", m.speed));
    if (auto e = check_fraction("acceleration", m.acceleration)) return PointToPoint::invalid(name, *e);
    if (auto e = check_fraction("deceleration", m.deceleration)) return PointToPoint::invalid(name, *e);
    const command::MoveByJoint cmd{.target = rad_to_deg(m.target),
                                   .speed_pct = std::max(0.01, m.speed * 100.0),
                                   .acc_pct = percent(m.acceleration),
                                   .dec_pct = percent(m.deceleration),
                                   .until = std::nullopt};
    return {name, command::to_request(cmd), joints_remaining(m.target, m.tolerance), m.timeout};
}

PointToPoint make_activity(const MoveLine& m) {
    constexpr std::string_view name = "move_line";
    if (auto e = check_joints(m.target)) return PointToPoint::invalid(name, *e);
    if (auto e = check_linear(m.acceleration, m.deceleration, m.speed)) return PointToPoint::invalid(name, *e);
    const command::MoveByLine cmd{.target = rad_to_deg(m.target),
                                  .speed_type = command::SpeedType::AbsoluteLinear,
                                  .speed = m.speed * 1000.0,
                                  .acc_pct = percent(m.acceleration),
                                  .dec_pct = percent(m.deceleration),
                                  .until = std::nullopt};
    return {name, command::to_request(cmd), joints_remaining(m.target, m.tolerance), m.timeout};
}

PointToPoint make_activity(const MoveLineToPose& m) {
    constexpr std::string_view name = "move_line_to_pose";
    if (auto e = check_linear(m.acceleration, m.deceleration, m.speed)) return PointToPoint::invalid(name, *e);
    const command::MoveByLineCoord cmd{.target = to_mm_rad(m.target),
                                       .speed_type = command::SpeedType::AbsoluteLinear,
                                       .speed = m.speed * 1000.0,
                                       .acc_pct = percent(m.acceleration),
                                       .dec_pct = percent(m.deceleration),
                                       .user_frame = command::PoseMmRad{},  // the base frame
                                       .until = std::nullopt};
    auto remaining = [target = m.target, pos_tol = m.position_tolerance,
                      rot_tol = m.rotation_tolerance](const feedback::State& s) -> std::optional<std::string> {
        const double off = distance(s.tcp_position_base, target.position);
        const double turn = rotation_between(s.tcp_rotation_base, target.rotation);
        if (off <= pos_tol && turn <= rot_tol) return std::nullopt;
        return std::format("{:.1f} mm and {:.2f} deg off", off * 1000.0, rad_to_deg(turn));
    };
    return {name, command::to_request(cmd), std::move(remaining), m.timeout};
}

PointToPoint make_activity(const MoveArc& m) {
    constexpr std::string_view name = "move_arc";
    if (auto e = check_joints(m.middle)) return PointToPoint::invalid(name, "middle: " + *e);
    if (auto e = check_joints(m.target)) return PointToPoint::invalid(name, *e);
    if (auto e = check_linear(m.acceleration, m.deceleration, m.speed)) return PointToPoint::invalid(name, *e);
    const command::MoveByArc cmd{.middle = rad_to_deg(m.middle),
                                 .target = rad_to_deg(m.target),
                                 .speed_type = command::SpeedType::AbsoluteLinear,
                                 .speed = m.speed * 1000.0,
                                 .acc_pct = percent(m.acceleration),
                                 .dec_pct = percent(m.deceleration),
                                 .until = std::nullopt};
    return {name, command::to_request(cmd), joints_remaining(m.target, m.tolerance), m.timeout};
}

}  // namespace ec::motion
