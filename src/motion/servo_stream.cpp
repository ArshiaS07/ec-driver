#include "ec/motion/servo_stream.hpp"

#include <cmath>
#include <format>
#include <thread>
#include <utility>

#include "detail/connection.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/transparent.hpp"

namespace ec::motion {

std::expected<ServoStream, Error> ServoStream::open(Robot& robot, const ServoOptions& o) {
    using namespace std::chrono_literals;
    const auto lookahead = o.lookahead.count();
    const auto period = o.period.count();
    if (period < 2 || period > 100)
        return std::unexpected(
            Error{Error::Kind::Invalid, std::format("period must be in [2, 100] ms, not {} ms", period)});
    if (lookahead < 10 || lookahead > 1000 || lookahead <= period)
        return std::unexpected(
            Error{Error::Kind::Invalid,
                  std::format("lookahead must be in [10, 1000] ms and longer than the period, not {} ms", lookahead)});
    if (!(o.smoothness >= 0.0 && o.smoothness <= 1.0))
        return std::unexpected(
            Error{Error::Kind::Invalid, std::format("smoothness must be in [0, 1], not {}", o.smoothness)});
    auto c = robot.share();
    const auto id = c->open([raw = c.get()] {
        const auto cleared = raw->send(command::TtClearServoJointBuf{});
        static_cast<void>(cleared);
    });
    if (!id) return std::unexpected(id.error());
    // Until it is open, a failure only gives the registration back: nothing is sent to clean up.
    const auto fail = [&c, &id](Error e) {
        c->close(*id);
        return std::unexpected(std::move(e));
    };
    const auto state = c->send(command::GetTransparentTransmissionState{});
    if (!state) return fail(state.error());
    if (*state == command::Flag::On) {  // left open: cleared first, as the manual's example
        if (auto ok = c->send(command::TtClearServoJointBuf{}); !ok) return fail(ok.error());
        std::this_thread::sleep_for(500ms);
    }
    if (auto ok = c->send(command::TransparentTransmissionInit{.lookahead_ms = static_cast<int>(lookahead),
                                                               .period_ms = static_cast<int>(period),
                                                               .smoothness = o.smoothness,
                                                               .replies = true});
        !ok)
        return fail(ok.error());
    return ServoStream(c, *id);
}

std::expected<void, Error> ServoStream::add_point(const Joints& q) {
    if (!c_) return std::unexpected(Error{Error::Kind::Invalid, "not an open stream"});
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(q[j]) <= kEc66JointLimit))
            return std::unexpected(Error{Error::Kind::Invalid,
                                         std::format("joint {} target {} rad is beyond the joint limit", j + 1, q[j])});
    return c_->send(command::TtPutServoJointToBuf{.target = rad_to_deg(q)}).transform([](auto&&) {});
}

std::expected<void, Error> ServoStream::close() {
    if (!c_) return std::unexpected(Error{Error::Kind::Invalid, "not an open stream"});
    auto ok = c_->send(command::TtClearServoJointBuf{});
    c_->close(id_);
    c_.reset();
    if (!ok) return std::unexpected(ok.error());
    return {};
}

void ServoStream::stop() noexcept {
    try {
        if (!c_) return;
        const auto cleared = c_->send(command::TtClearServoJointBuf{});
        static_cast<void>(cleared);
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

ServoStream::ServoStream(ServoStream&& other) noexcept : c_(std::move(other.c_)), id_(std::exchange(other.id_, 0)) {}

ServoStream& ServoStream::operator=(ServoStream&& other) noexcept {
    if (this != &other) {
        close_quietly();
        c_ = std::move(other.c_);
        id_ = std::exchange(other.id_, 0);
    }
    return *this;
}

ServoStream::~ServoStream() { close_quietly(); }

void ServoStream::close_quietly() noexcept {
    try {
        if (!c_) return;
        const auto closed = close();
        static_cast<void>(closed);  // nothing more to do if it fails
    } catch (...) {                 // NOLINT(bugprone-empty-catch): noexcept; a failed close has no one to tell
        if (c_) c_->close(id_);
    }
}

}  // namespace ec::motion
