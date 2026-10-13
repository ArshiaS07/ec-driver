#include "ec/motion/stream.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

#include "ec/command/movement.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/transparent.hpp"

namespace ec::motion {
namespace {

using std::chrono::duration_cast;
using std::chrono::milliseconds;

constexpr auto kReplyTimeout = std::chrono::seconds{2};

double seconds(std::chrono::nanoseconds d) { return std::chrono::duration<double>(d).count(); }

// The commands of `method` among this cycle's replies must all have succeeded.
std::expected<void, Error> all_ok(const Cycle& c, std::string_view method) {
    for (const Reply& r : c.replies) {
        if (r.method != method) continue;
        if (!r.result) return std::unexpected(command_failed(method, r.result.error()));
        if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*r.result); !ok)
            return std::unexpected(command_failed(method, ok.error()));
    }
    return {};
}

Error starved(std::string_view who, std::chrono::nanoseconds age) {
    return Error{Error::Kind::Starved,
                 std::format("{}: the newest target is {} ms old", who, duration_cast<milliseconds>(age).count()),
                 {}};
}

std::optional<std::string> check_target(const Joints& q, double limit) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!std::isfinite(q[j]) || std::abs(q[j]) > limit)
            return std::format("joint {} target {} rad is outside the joint limit", j + 1, q[j]);
    return std::nullopt;
}

int deg_s2(double rad_s2) { return std::max(1, static_cast<int>(std::lround(rad_to_deg(rad_s2)))); }

}  // namespace

// ------------------------------------------------------------------- ServoStream

ServoStream::ServoStream(const ServoOptions& options, std::shared_ptr<Channel<Joints>> channel)
    : options_(options), channel_(std::move(channel)) {}

std::expected<Phase, Error> ServoStream::step(const Cycle& c, Outbox& out) {
    const auto invalid = [](std::string message) {
        return std::unexpected(Error{Error::Kind::Invalid, "servo_stream: " + std::move(message), {}});
    };
    switch (stage_) {
        case Stage::Open: {
            const auto period = duration_cast<milliseconds>(c.period);
            if (period != c.period || period.count() < 2 || period.count() > 100)
                return invalid(std::format("the control period must be whole milliseconds in [2, 100], not {} ns",
                                           c.period.count()));
            const auto lookahead = options_.lookahead;
            if (lookahead.count() < 10 || lookahead.count() > 1000 || lookahead <= period)
                return invalid(std::format("lookahead must be in [10, 1000] ms and longer than the period, not {} ms",
                                           lookahead.count()));
            if (!(options_.smoothness >= 0.0 && options_.smoothness <= 1.0))
                return invalid(std::format("smoothness must be in [0, 1], not {}", options_.smoothness));
            if (options_.accelerate_in.count() <= 0) return invalid("accelerate_in must be positive");
            Joints accel{};
            for (std::size_t j = 0; j < kJointCount; ++j) {
                if (!(options_.max_velocity[j] > 0.0)) return invalid("every max_velocity must be positive");
                accel[j] = options_.max_velocity[j] / seconds(options_.accelerate_in);
            }
            shaper_.emplace(RateLimiter::Limits{options_.max_velocity, accel}, seconds(c.period));
            shaper_->reset(c.state.joint_position);
            recent_capacity_ = static_cast<std::size_t>((lookahead + milliseconds{200}) / period) + 1;
            recent_.assign(1, c.state.joint_position);
            command::TransparentTransmissionInit init{.lookahead_ms = static_cast<int>(lookahead.count()),
                                                      .period_ms = static_cast<int>(period.count()),
                                                      .smoothness = options_.smoothness,
                                                      .replies = std::nullopt};
            if (!options_.replies) init.replies = false;
            out.send(init);
            sent_at_ = c.now;
            stage_ = Stage::AwaitOpen;
            return Phase::Running;
        }

        case Stage::AwaitOpen: {
            const Reply* reply = find_reply<command::TransparentTransmissionInit>(c.replies);
            if (reply == nullptr) {
                if (c.now - sent_at_ > kReplyTimeout)
                    return std::unexpected(Error{Error::Kind::Timeout, "servo_stream: no reply to the init", {}});
                return Phase::Running;
            }
            if (const auto ok = decode<command::TransparentTransmissionInit>(*reply); !ok)
                return std::unexpected(command_failed(reply->method, ok.error()));
            stage_ = Stage::Streaming;
            channel_->mark_open();
            [[fallthrough]];
        }

        case Stage::Streaming: {
            if (!shaper_) return std::unexpected(Error{Error::Kind::Internal, "servo_stream: not opened", {}});
            RateLimiter& shaper = *shaper_;
            if (auto ok = check_replies(c); !ok) return std::unexpected(ok.error());
            if (channel_->close_requested()) return Phase::Done;
            Joints setpoint{};
            if (const auto target = channel_->newest()) {
                pushed_ = true;
                const auto age = c.now - target->at;
                if (age > options_.watchdog.target_timeout + options_.watchdog.hold_for)
                    return std::unexpected(starved("servo_stream", age));
                if (age > options_.watchdog.target_timeout) {
                    setpoint = shaper.brake();
                } else {
                    if (auto bad = check_target(target->value, options_.joint_limit)) return invalid(*bad);
                    setpoint = shaper.next(target->value);
                }
            } else {
                setpoint = shaper.brake();  // nothing pushed yet: stays where it started
            }
            if (auto ok = check_tracking(c); !ok) return std::unexpected(ok.error());
            put(c, out, setpoint);
            return Phase::Running;
        }
    }
    return Phase::Running;
}

std::expected<void, Error> ServoStream::check_replies(const Cycle& c) const {
    if (!options_.replies) return {};
    return all_ok(c, command::TtPutServoJointToBuf::method);
}

std::expected<void, Error> ServoStream::check_tracking(const Cycle& c) const {
    for (std::size_t j = 0; j < kJointCount; ++j) {
        double lo = recent_.front()[j];
        double hi = lo;
        for (const Joints& q : recent_) {
            lo = std::min(lo, q[j]);
            hi = std::max(hi, q[j]);
        }
        const double q = c.state.joint_position[j];
        const double off = q < lo ? lo - q : (q > hi ? q - hi : 0.0);
        if (off > options_.tracking_tolerance)
            return std::unexpected(
                Error{Error::Kind::Tracking,
                      std::format("servo_stream: joint {} is {:.2f} deg away from its recent setpoints", j + 1,
                                  rad_to_deg(off)),
                      {}});
    }
    return {};
}

void ServoStream::put(const Cycle& /*c*/, Outbox& out, const Joints& setpoint) {
    const command::TtPutServoJointToBuf point{.target = rad_to_deg(setpoint)};
    if (options_.replies)
        out.send(point);
    else
        out.send_without_reply(point);
    recent_.push_back(setpoint);
    while (recent_.size() > recent_capacity_) recent_.pop_front();
}

bool ServoStream::close(const Cycle& c, Outbox& out, End why) {
    for (;;) {
        switch (closing_) {
            case Closing::Begin:
                if (stage_ == Stage::Open) return true;  // nothing was sent
                if (stage_ == Stage::Streaming && (why == End::Done || why == End::Cancelled)) {
                    closing_ = Closing::Settle;
                    settled_since_ = {};
                    break;
                }
                out.send(command::Stop{});
                closing_ = Closing::Clear;
                break;

            case Closing::Settle: {
                if (!shaper_) {
                    closing_ = Closing::Clear;
                    break;
                }
                RateLimiter& shaper = *shaper_;
                // Keep the queue fed along the fastest smooth stop, then with the
                // final point until the arm has consumed the lookahead and rests.
                if (!all_ok(c, command::TtPutServoJointToBuf::method) && options_.replies) {
                    closing_ = Closing::Clear;  // the controller stopped accepting points: just clear
                    break;
                }
                put(c, out, shaper.brake());
                if (!shaper.at_rest()) return false;
                if (settled_since_ == Clock::time_point{}) settled_since_ = c.now;
                if (c.now - settled_since_ < options_.lookahead + 2 * c.period || !c.conditions.has(Condition::AtRest))
                    return false;
                closing_ = Closing::Clear;
                return false;  // clear in the next cycle, after these points
            }

            case Closing::Clear:
                out.send(command::TtClearServoJointBuf{});
                sent_at_ = c.now;
                closing_ = Closing::AwaitClear;
                return false;

            case Closing::AwaitClear: {
                const bool replied = find_reply<command::TtClearServoJointBuf>(c.replies) != nullptr;
                if (!replied && c.now - sent_at_ <= kReplyTimeout) return false;
                return c.conditions.has(Condition::AtRest);
            }
        }
    }
}

// ---------------------------------------------------------------- VelocityStream

std::expected<Phase, Error> VelocityStream::step(const Cycle& c, Outbox& out) {
    channel_->mark_open();
    if (auto ok = all_ok(c, command::MoveBySpeedj::method); !ok) return std::unexpected(ok.error());
    if (channel_->close_requested()) return Phase::Done;
    const auto target = channel_->newest();
    if (!target) return Phase::Running;  // nothing pushed yet: nothing sent
    pushed_ = true;
    Joints v{};
    const auto age = c.now - target->at;
    if (age > options_.watchdog.target_timeout + options_.watchdog.hold_for)
        return std::unexpected(starved("velocity_stream", age));
    if (age <= options_.watchdog.target_timeout) {
        v = target->value;
        double k = 1.0;  // scales the vector down, keeping its direction, to respect every joint's limit
        for (std::size_t j = 0; j < kJointCount; ++j) {
            if (!std::isfinite(v[j]))
                return std::unexpected(Error{Error::Kind::Invalid, "velocity_stream: a target is not finite", {}});
            if (std::abs(v[j]) > options_.max_velocity[j]) k = std::min(k, options_.max_velocity[j] / std::abs(v[j]));
        }
        v = scale(v, k);
    }
    out.send(command::MoveBySpeedj{.velocity_deg_s = rad_to_deg(v),
                                   .acc_deg_s2 = deg_s2(options_.acceleration),
                                   .duration_s = seconds(options_.command_duration)});
    return Phase::Running;
}

bool VelocityStream::close(const Cycle& c, Outbox& out, End why) {
    if (!pushed_) return true;
    if (!stop_sent_at_) {
        out.send(command::Stopj{.acc_deg_s2 = deg_s2(options_.acceleration)});
        if (why == End::Stopped || why == End::Faulted) out.send(command::Stop{});
        stop_sent_at_ = c.now;
        return false;
    }
    return c.now > *stop_sent_at_ && c.conditions.has(Condition::AtRest);
}

// --------------------------------------------------------------------- JogStream

std::expected<Phase, Error> JogStream::step(const Cycle& c, Outbox& out) {
    channel_->mark_open();
    if (auto ok = all_ok(c, command::Jog::method); !ok) return std::unexpected(ok.error());
    if (!(options_.speed > 0.0 && options_.speed <= 1.0))
        return std::unexpected(Error{
            Error::Kind::Invalid, std::format("jog_stream: speed must be in (0, 1], not {}", options_.speed), {}});
    if (channel_->close_requested()) return Phase::Done;
    std::optional<JogDirection> wanted;
    if (const auto target = channel_->newest(); target && c.now - target->at <= options_.watchdog.target_timeout)
        wanted = target->value;
    if (wanted) {
        if (wanted->axis < 0 || wanted->axis > 5)
            return std::unexpected(
                Error{Error::Kind::Invalid, std::format("jog_stream: axis {} is not 0..5", wanted->axis), {}});
        const int index = 2 * wanted->axis + (wanted->positive ? 1 : 0);
        if (jogging_ != index || c.now - last_sent_ >= options_.repeat) {
            out.send(command::Jog{.index = index, .speed_pct = std::max(0.05, options_.speed * 100.0)});
            jogging_ = index;
            last_sent_ = c.now;
        }
    } else if (jogging_) {
        out.send(command::Stop{});
        jogging_.reset();
    }
    return Phase::Running;
}

bool JogStream::close(const Cycle& c, Outbox& out, End /*why*/) {
    if (!stop_sent_at_) {
        out.send(command::Stop{});
        stop_sent_at_ = c.now;
        return false;
    }
    return c.now > *stop_sent_at_ && c.conditions.has(Condition::AtRest);
}

}  // namespace ec::motion
