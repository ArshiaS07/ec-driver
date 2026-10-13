#include "ec/motion/waypoints.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <utility>

#include "ec/command/movement.hpp"
#include "ec/command/reply.hpp"

namespace ec::motion {
namespace {

constexpr auto kReplyTimeout = std::chrono::seconds{2};

std::optional<std::string> check(const MovePath& spec) {
    if (spec.points.empty()) return "no waypoints";
    if (spec.points_per_cycle < 1) return "points_per_cycle must be at least 1";
    if (!(spec.tolerance > 0.0)) return "the tolerance must be positive";
    for (std::size_t k = 0; k < spec.points.size(); ++k) {
        const Waypoint& w = spec.points[k];
        for (std::size_t j = 0; j < kJointCount; ++j)
            if (!std::isfinite(w.position[j]) || std::abs(w.position[j]) > kEc66JointLimit)
                return std::format("waypoint {}: joint {} at {} rad is outside +-360 deg", k, j + 1, w.position[j]);
        if (w.move == PathMove::Line && !(w.speed >= 0.001 && w.speed <= 3.0))
            return std::format("waypoint {}: a line's speed must be in [0.001, 3] m/s, not {}", k, w.speed);
        if (w.move == PathMove::Joint && !(w.speed >= 0.01 && w.speed <= 1.0))
            return std::format("waypoint {}: a joint move's speed must be in [0.01, 1], not {}", k, w.speed);
        if (!(w.blend_radius >= 0.0 && w.blend_radius <= 2.608))
            return std::format("waypoint {}: the blend radius must be in [0, 2.608] m, not {}", k, w.blend_radius);
    }
    if (spec.points.back().blend_radius != 0.0) return "the last waypoint's blend radius must be 0";
    return std::nullopt;
}

command::AddPathPoint to_command(const Waypoint& w) {
    command::AddPathPoint p{};
    p.point = rad_to_deg(w.position);
    p.blend_radius_mm = w.blend_radius * 1000.0;
    if (w.move == PathMove::Joint) {
        p.move_type = command::PathMoveType::Joint;
        p.speed = std::max(1.0, w.speed * 100.0);
    } else {
        p.move_type = command::PathMoveType::Line;
        p.speed_type = command::SpeedType::AbsoluteLinear;
        p.speed = w.speed * 1000.0;
    }
    return p;
}

}  // namespace

std::optional<std::expected<void, Error>> MovePathActivity::await_ok(const Cycle& c, std::string_view method) const {
    for (const Reply& r : c.replies) {
        if (r.method != method) continue;
        if (!r.result) return std::unexpected(command_failed(method, r.result.error()));
        if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*r.result); !ok)
            return std::unexpected(command_failed(method, ok.error()));
        return std::expected<void, Error>{};
    }
    if (c.now - sent_at_ > kReplyTimeout)
        return std::unexpected(Error{Error::Kind::Timeout, std::format("move_path: no reply to {}", method), {}});
    return std::nullopt;
}

std::expected<Phase, Error> MovePathActivity::step(const Cycle& c, Outbox& out) {
    switch (stage_) {
        case Stage::Check:
            if (auto bad = check(spec_)) return std::unexpected(Error{Error::Kind::Invalid, "move_path: " + *bad, {}});
            out.send(command::ClearPathPoint{});
            sent_at_ = c.now;
            stage_ = Stage::AwaitClear;
            return Phase::Running;

        case Stage::AwaitClear: {
            const auto r = await_ok(c, command::ClearPathPoint::method);
            if (!r) return Phase::Running;
            if (!*r) return std::unexpected(r->error());
            stage_ = Stage::Upload;
            return upload(c, out);
        }

        case Stage::Upload: return upload(c, out);

        case Stage::AwaitRun: {
            const Reply* reply = find_reply<command::MoveByPath>(c.replies);
            if (reply == nullptr) {
                if (c.now - sent_at_ > kReplyTimeout)
                    return std::unexpected(Error{Error::Kind::Timeout, "move_path: no reply to moveByPath", {}});
                return Phase::Running;
            }
            const auto count = decode<command::MoveByPath>(*reply);
            if (!count) return std::unexpected(command_failed(command::MoveByPath::method, count.error()));
            if (static_cast<std::size_t>(*count) != spec_.points.size())
                return std::unexpected(Error{
                    Error::Kind::Command,
                    std::format("move_path: the controller runs {} waypoints, not {}", *count, spec_.points.size()),
                    {}});
            stage_ = Stage::Moving;
            return Phase::Running;
        }

        case Stage::Moving: return watch(c);
    }
    return Phase::Running;
}

std::expected<Phase, Error> MovePathActivity::upload(const Cycle& c, Outbox& out) {
    std::size_t replies = 0;
    for (const Reply& reply : c.replies) {
        if (reply.method != command::AddPathPoint::method) continue;
        ++replies;
        if (!reply.result) return std::unexpected(command_failed(reply.method, reply.result.error()));
        if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*reply.result); !ok)
            return std::unexpected(command_failed(reply.method, ok.error()));
    }
    if (replies < awaited_) {
        if (c.now - sent_at_ > kReplyTimeout)
            return std::unexpected(Error{Error::Kind::Timeout, "move_path: addPathPoint replies missing", {}});
        return Phase::Running;
    }
    awaited_ = 0;
    if (uploaded_ == spec_.points.size()) {
        out.send(command::MoveByPath{});
        sent_at_ = c.now;
        run_sent_ = true;
        stage_ = Stage::AwaitRun;
        return Phase::Running;
    }
    const auto batch = std::min(spec_.points.size() - uploaded_, static_cast<std::size_t>(spec_.points_per_cycle));
    for (std::size_t k = uploaded_; k < uploaded_ + batch; ++k) out.send(to_command(spec_.points[k]));
    uploaded_ += batch;
    awaited_ = batch;
    sent_at_ = c.now;
    return Phase::Running;
}

std::expected<Phase, Error> MovePathActivity::watch(const Cycle& c) {
    if (c.state.robot_state == RobotState::Running || !c.conditions.has(Condition::AtRest)) moved_ = true;
    const double off = max_abs_diff(c.state.joint_position, spec_.points.back().position);
    const bool resting = c.conditions.has(Condition::Standstill);
    if (resting && off <= spec_.tolerance) return Phase::Done;
    if (resting && moved_) {
        if (!resting_since_) resting_since_ = c.now;
        if (c.now - *resting_since_ >= spec_.stopped_for)
            return std::unexpected(
                Error{Error::Kind::Command,
                      std::format("move_path: the arm stopped {:.2f} deg from the last waypoint", rad_to_deg(off)),
                      {}});
    } else {
        resting_since_.reset();
    }
    if (c.now - sent_at_ > spec_.timeout)
        return std::unexpected(
            Error{Error::Kind::Timeout,
                  std::format("move_path: not at the last waypoint after {} s",
                              std::chrono::duration_cast<std::chrono::seconds>(spec_.timeout).count()),
                  {}});
    return Phase::Running;
}

bool MovePathActivity::close(const Cycle& c, Outbox& out, End why) {
    if (why == End::Done || !run_sent_) return true;  // arrived, or nothing set the arm moving
    if (!stop_sent_at_) {
        out.send(command::Stop{});
        stop_sent_at_ = c.now;
        return false;
    }
    return c.now > *stop_sent_at_ && c.conditions.has(Condition::AtRest);
}

}  // namespace ec::motion
