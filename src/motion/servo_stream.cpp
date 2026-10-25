#include "ec/motion/servo_stream.hpp"

#include <cmath>
#include <format>
#include <string>
#include <thread>
#include <utility>

#include "detail/connection.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/transparent.hpp"

namespace ec::motion {
namespace {

std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message)});
}

}  // namespace

ServoStream::ServoStream(Robot& robot, const ServoOptions& options) : session_(robot.session()), options_(options) {}

std::expected<void, Error> ServoStream::open() {
    using namespace std::chrono_literals;
    const auto lookahead = options_.lookahead.count();
    const auto period = options_.period.count();
    if (lookahead < 10 || lookahead > 1000)
        return invalid(std::format("lookahead must be in [10, 1000] ms, not {} ms", lookahead));
    if (period < 2 || period > 100) return invalid(std::format("period must be in [2, 100] ms, not {} ms", period));
    if (!(options_.smoothness >= 0.0 && options_.smoothness <= 1.0))
        return invalid(std::format("smoothness must be in [0, 1], not {}", options_.smoothness));

    const auto reached = detail::reach(session_, "open");
    if (!reached) return std::unexpected(reached.error());
    detail::Connection& c = **reached;
    const auto id = c.open([raw = &c] {
        const auto cleared = raw->send(sdk::TtClearServoJointBuf{});
        static_cast<void>(cleared);
    });
    if (!id) return std::unexpected(id.error());
    // Until it is open, a failure gives the registration back; nothing is sent to clean up.
    const auto fail = [&c, &id](Error e) {
        c.close(*id);
        return std::unexpected(std::move(e));
    };
    const auto state = c.send(sdk::GetTransparentTransmissionState{});
    if (!state) return fail(state.error());
    if (*state == sdk::Flag::On) {  // left open: cleared first, as the manual's example
        if (auto ok = c.send(sdk::TtClearServoJointBuf{}); !ok) return fail(ok.error());
        std::this_thread::sleep_for(500ms);
    }
    if (auto ok = c.send(sdk::TransparentTransmissionInit{.lookahead_ms = static_cast<int>(lookahead),
                                                          .period_ms = static_cast<int>(period),
                                                          .smoothness = options_.smoothness,
                                                          .replies = true});
        !ok)
        return fail(ok.error());
    id_ = *id;
    return {};
}

std::expected<void, Error> ServoStream::add_point(const Joints& q) {
    if (id_ == 0) return invalid("add_point: the stream is not open");
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(q[j]) <= kEc66JointLimit))
            return invalid(std::format("joint {} at {} rad is outside [-360, 360] deg", j + 1, q[j]));
    const auto c = detail::reach(session_, "tt_put_servo_joint_to_buf");
    if (!c) return std::unexpected(c.error());
    return (*c)->send(sdk::TtPutServoJointToBuf{.target = rad_to_deg(q)}).transform([](auto&&) {});
}

std::expected<void, Error> ServoStream::close() {
    if (id_ == 0) return invalid("close: the stream is not open");
    const auto c = detail::reach(session_, "tt_clear_servo_joint_buf");
    if (!c) {
        id_ = 0;
        return std::unexpected(c.error());
    }
    auto ok = (*c)->send(sdk::TtClearServoJointBuf{});
    (*c)->close(std::exchange(id_, 0));
    if (!ok) return std::unexpected(ok.error());
    return {};
}

void ServoStream::stop() noexcept {
    try {
        if (const auto c = session_.lock()) {
            const auto cleared = c->send(sdk::TtClearServoJointBuf{});
            static_cast<void>(cleared);
        }
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

ServoStream::ServoStream(ServoStream&& other) noexcept
    : session_(std::move(other.session_)), options_(other.options_), id_(std::exchange(other.id_, 0)) {}

ServoStream& ServoStream::operator=(ServoStream&& other) noexcept {
    if (this != &other) {
        close_quietly();
        session_ = std::move(other.session_);
        options_ = other.options_;
        id_ = std::exchange(other.id_, 0);
    }
    return *this;
}

ServoStream::~ServoStream() { close_quietly(); }

void ServoStream::close_quietly() noexcept {
    try {
        if (id_ == 0) return;
        const auto closed = close();
        static_cast<void>(closed);  // nothing more to do if it fails
    } catch (...) {                 // NOLINT(bugprone-empty-catch): noexcept; a failed close has no one to tell
        if (const auto c = session_.lock(); c && id_ != 0) c->close(id_);
        id_ = 0;
    }
}

}  // namespace ec::motion
