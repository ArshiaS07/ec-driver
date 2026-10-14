#include "ec/motion/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/trajectory.hpp"

namespace ec::motion {
namespace {

constexpr auto kReplyTimeout = std::chrono::seconds{2};
// The pauses around an upload, as in the manual's examples (2.2.9.10, 2.2.9.11).
constexpr auto kBeforeUpload = std::chrono::milliseconds{200};
constexpr auto kBeforeSeal = std::chrono::milliseconds{1'000};
constexpr int kShortCycles = 3;

}  // namespace

std::expected<void, std::string> validate(const JointTrajectory& t, const TrajectoryLimits& limits) {
    if (t.time.size() != t.position.size())
        return std::unexpected(std::format("{} times but {} positions", t.time.size(), t.position.size()));
    if (t.size() < 3) return std::unexpected(std::format("{} points; at least 3 are needed", t.size()));
    if (t.time.front() != 0.0) return std::unexpected(std::format("the first time is {} s, not 0", t.time.front()));
    for (std::size_t k = 0; k < t.size(); ++k) {
        for (std::size_t j = 0; j < kJointCount; ++j) {
            const double q = t.position[k][j];
            if (!std::isfinite(q) || std::abs(q) > limits.joint_limit)
                return std::unexpected(std::format("point {}: joint {} at {} rad is outside the limit", k, j + 1, q));
        }
        if (k == 0) continue;
        const double dt = t.time[k] - t.time[k - 1];
        if (!(dt > 0.0)) return std::unexpected(std::format("point {}: time {} s does not increase", k, t.time[k]));
        for (std::size_t j = 0; j < kJointCount; ++j) {
            const double v = std::abs(t.position[k][j] - t.position[k - 1][j]) / dt;
            if (v > limits.max_velocity[j])
                return std::unexpected(std::format("point {}: joint {} would move at {:.3f} rad/s, above {:.3f}", k,
                                                   j + 1, v, limits.max_velocity[j]));
        }
    }
    const Joints a = peak_acceleration(t);
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (a[j] > limits.max_acceleration[j] * (1.0 + 1e-9))
            return std::unexpected(std::format("joint {} would accelerate at {:.3f} rad/s^2, above {:.3f}", j + 1, a[j],
                                               limits.max_acceleration[j]));
    return {};
}

Joints peak_velocity(const JointTrajectory& t) noexcept {
    Joints peak{};
    for (std::size_t k = 1; k < t.size(); ++k) {
        const double dt = t.time[k] - t.time[k - 1];
        for (std::size_t j = 0; j < kJointCount; ++j)
            peak[j] = std::max(peak[j], std::abs(t.position[k][j] - t.position[k - 1][j]) / dt);
    }
    return peak;
}

Joints peak_acceleration(const JointTrajectory& t) noexcept {
    Joints peak{};
    const std::size_t n = t.size();
    if (n < 2) return peak;
    // The speed of segment k, from point k to k + 1; at rest before the first and after the last.
    const auto speed = [&t](std::size_t k, std::size_t j) {
        return (t.position[k + 1][j] - t.position[k][j]) / (t.time[k + 1] - t.time[k]);
    };
    for (std::size_t j = 0; j < kJointCount; ++j) {
        peak[j] = std::max(std::abs(speed(0, j)) / (t.time[1] - t.time[0]),
                           std::abs(speed(n - 2, j)) / (t.time[n - 1] - t.time[n - 2]));
        for (std::size_t k = 1; k + 1 < n; ++k) {
            const double dt = (t.time[k + 1] - t.time[k - 1]) / 2.0;
            peak[j] = std::max(peak[j], std::abs(speed(k, j) - speed(k - 1, j)) / dt);
        }
    }
    return peak;
}

Joints position_at(const JointTrajectory& t, double s) noexcept {
    if (t.size() == 0) return {};
    if (!(s > t.time.front())) return t.position.front();
    if (s >= t.time.back()) return t.position.back();
    const auto it = std::upper_bound(t.time.begin(), t.time.end(), s);  // the first point after s
    const auto k = static_cast<std::size_t>(it - t.time.begin());
    const double u = (s - t.time[k - 1]) / (t.time[k] - t.time[k - 1]);
    Joints q{};
    for (std::size_t j = 0; j < kJointCount; ++j)
        q[j] = t.position[k - 1][j] + (t.position[k][j] - t.position[k - 1][j]) * u;
    return q;
}

std::optional<std::expected<void, Error>> PlayTrajectoryActivity::await(const Cycle& c, std::string_view method) const {
    for (const Reply& r : c.replies) {
        if (r.method != method) continue;
        if (!r.result) return std::unexpected(command_failed(method, r.result.error()));
        if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*r.result); !ok)
            return std::unexpected(command_failed(method, ok.error()));
        return std::expected<void, Error>{};
    }
    if (c.now - sent_at_ > kReplyTimeout)
        return std::unexpected(Error{Error::Kind::Timeout, std::format("play_trajectory: no reply to {}", method), {}});
    return std::nullopt;
}

std::expected<Phase, Error> PlayTrajectoryActivity::step(const Cycle& c, Outbox& out) {
    const auto fail = [](Error::Kind kind, std::string message) {
        return std::unexpected(Error{kind, "play_trajectory: " + std::move(message), {}});
    };
    switch (stage_) {
        case Stage::Check: {
            if (const auto ok = validate(spec_.path, spec_.limits); !ok) return fail(Error::Kind::Invalid, ok.error());
            if (!(spec_.speed > 0.0 && spec_.speed <= 1.0))
                return fail(Error::Kind::Invalid, std::format("speed must be in (0, 1], not {}", spec_.speed));
            if (spec_.points_per_cycle < 1) return fail(Error::Kind::Invalid, "points_per_cycle must be at least 1");
            stationary_ = std::ranges::all_of(spec_.path.position, [this](const Joints& q) {
                return max_abs_diff(q, spec_.path.position.front()) <= spec_.tolerance;
            });
            const double off = max_abs_diff(c.state.joint_position, spec_.path.position.front());
            if (off > spec_.start_tolerance)
                return fail(
                    Error::Kind::Invalid,
                    std::format("the arm is {:.2f} deg from the first point; move there first", rad_to_deg(off)));
            out.send(command::GetSpeed{});
            sent_at_ = c.now;
            stage_ = Stage::AwaitSpeed;
            return Phase::Running;
        }

        case Stage::AwaitSpeed: {
            const Reply* reply = find_reply<command::GetSpeed>(c.replies);
            if (reply == nullptr) {
                if (c.now - sent_at_ > kReplyTimeout) return fail(Error::Kind::Timeout, "no reply to getSpeed");
                return Phase::Running;
            }
            const auto pct = decode<command::GetSpeed>(*reply);
            if (!pct) return std::unexpected(command_failed(command::GetSpeed::method, pct.error()));
            setting_ = std::clamp(*pct / 100.0, 0.0005, 1.0);
            out.send(command::StartPushPos{.point_count = static_cast<int>(spec_.path.size()),
                                           .point_type = command::PointType::Joint,
                                           .reference = rad_to_deg(spec_.path.position.front()),
                                           .frame = {},
                                           .replies = true});
            sent_at_ = c.now;
            stage_ = Stage::AwaitBegin;
            return Phase::Running;
        }

        case Stage::AwaitBegin: {
            const auto r = await(c, command::StartPushPos::method);
            if (!r) return Phase::Running;
            if (!*r) return std::unexpected(r->error());
            uploading_ = true;
            paused_since_ = c.now;
            stage_ = Stage::Upload;
            return Phase::Running;
        }

        case Stage::Upload: {
            // As the manual's examples (2.2.9.10, 11): a pause after start_push_pos,
            // then the points, each confirmed: the next batch goes once the
            // controller has taken the last.
            for (const Reply& reply : c.replies) {
                if (reply.method != command::PushPos::method) continue;
                if (!reply.result) return std::unexpected(command_failed(reply.method, reply.result.error()));
                if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*reply.result); !ok)
                    return std::unexpected(command_failed(reply.method, ok.error()));
            }
            if (c.now - paused_since_ < kBeforeUpload) return Phase::Running;
            if (uploaded_ == spec_.path.size()) {
                paused_since_ = c.now;
                stage_ = Stage::Settle;
                return Phase::Running;
            }
            const auto batch =
                std::min(spec_.path.size() - uploaded_, static_cast<std::size_t>(spec_.points_per_cycle));
            for (std::size_t k = uploaded_; k < uploaded_ + batch; ++k)
                out.send(command::PushPos{.time_s = spec_.path.time[k], .point = rad_to_deg(spec_.path.position[k])});
            uploaded_ += batch;
            return Phase::Running;
        }

        case Stage::Settle: {
            // And a pause before stop_push_pos, which confirms that every point was right.
            if (c.now - paused_since_ < kBeforeSeal) return Phase::Running;
            out.send(command::StopPushPos{});
            sent_at_ = c.now;
            stage_ = Stage::AwaitSeal;
            return Phase::Running;
        }

        case Stage::AwaitSeal: {
            const auto r = await(c, command::StopPushPos::method);
            if (!r) return Phase::Running;
            if (!*r) return std::unexpected(r->error());
            out.send(command::CheckTrajectory{});
            sent_at_ = c.now;
            stage_ = Stage::AwaitCheck;
            return Phase::Running;
        }

        case Stage::AwaitCheck: {
            const Reply* reply = find_reply<command::CheckTrajectory>(c.replies);
            if (reply == nullptr) {
                if (c.now - sent_at_ > kReplyTimeout) return fail(Error::Kind::Timeout, "no reply to check_trajectory");
                return Phase::Running;
            }
            const auto check = decode<command::CheckTrajectory>(*reply);
            if (!check) return std::unexpected(command_failed(command::CheckTrajectory::method, check.error()));
            if (*check != command::TrajectoryCheck::Valid)
                return fail(Error::Kind::Command,
                            std::format("the controller found the upload invalid ({})", static_cast<int>(*check)));
            out.send(command::StartTrajectory{.speed_pct = spec_.speed * 100.0});
            start_sent_ = true;
            sent_at_ = c.now;
            stage_ = Stage::AwaitStart;
            return Phase::Running;
        }

        case Stage::AwaitStart: {
            const auto r = await(c, command::StartTrajectory::method);
            if (!r) return Phase::Running;
            if (!*r) return std::unexpected(r->error());
            started_at_ = c.now;
            shortest_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(spec_.path.duration() / spec_.speed));
            limit_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(spec_.path.duration() / (spec_.speed * setting_))) +
                     spec_.margin;
            stage_ = Stage::Running;
            return Phase::Running;
        }

        case Stage::Running: {
            if (c.state.robot_state == RobotState::Running || !c.conditions.has(Condition::AtRest)) moved_ = true;
            const double off = max_abs_diff(c.state.joint_position, spec_.path.position.back());
            const bool resting = c.conditions.has(Condition::Standstill);
            // Arrived: at rest at the last point, after moving; a trajectory that stays at
            // its first point (within the tolerance) after its own time. A closed path
            // ends where it starts, so without the motion arrival would mean nothing.
            if (resting && off <= spec_.tolerance && (moved_ || (stationary_ && c.now - started_at_ >= shortest_)))
                return Phase::Done;
            if (resting && moved_ && ++short_cycles_ >= kShortCycles)
                return fail(Error::Kind::Command,
                            std::format("the arm stopped {:.2f} deg from the last point", rad_to_deg(off)));
            if (!resting) short_cycles_ = 0;
            if (c.now - started_at_ > limit_)
                return fail(Error::Kind::Timeout,
                            std::format("not finished after {:.1f} s",
                                        std::chrono::duration<double>(c.now - started_at_).count()));
            return Phase::Running;
        }
    }
    return Phase::Running;
}

bool PlayTrajectoryActivity::close(const Cycle& c, Outbox& out, End why) {
    for (;;) {
        switch (closing_) {
            case Closing::Begin:
                if (!uploading_) return true;  // nothing on the controller to clean up
                // Once start_trajectory went out the arm may move, even if its reply
                // has not come (or never will): stop it before clearing.
                if (start_sent_ && why != End::Done) {
                    out.send(command::StopTrajectory{});
                    out.send(command::Stop{});
                    stop_sent_at_ = c.now;
                    closing_ = Closing::AwaitRest;
                    return false;
                }
                closing_ = Closing::Flush;
                break;
            case Closing::AwaitRest:
                if (c.now <= stop_sent_at_ || !c.conditions.has(Condition::AtRest)) return false;
                closing_ = Closing::Flush;
                break;
            case Closing::Flush:
                out.send(command::FlushTrajectory{});
                sent_at_ = c.now;
                closing_ = Closing::AwaitFlush;
                return false;
            case Closing::AwaitFlush: {
                const auto r = await(c, command::FlushTrajectory::method);
                return r.has_value();  // flushed, or refused: either way nothing more can be done
            }
        }
    }
}

}  // namespace ec::motion
