#include "ec/motion/move.hpp"

#include <utility>

#include "detail/connection.hpp"
#include "motion/checks.hpp"

namespace ec::motion {
namespace {

using checks::check_target;
using checks::invalid;
using checks::percent;

// A move of the TCP's speed and acceleration, in the controller's units.
struct Profile {
    double speed = 0.0;  // mm/s or deg/s, as speed_type says
    std::optional<int> acc_pct{};
    std::optional<int> dec_pct{};
};

std::expected<Profile, Error> profile(const PathMoveOptions& o) {
    const auto speed = checks::tcp_speed(o.speed_type, o.speed);
    if (!speed) return std::unexpected(speed.error());
    const auto acc = percent("acceleration", o.acceleration);
    if (!acc) return std::unexpected(acc.error());
    const auto dec = percent("deceleration", o.deceleration);
    if (!dec) return std::unexpected(dec.error());
    return Profile{.speed = *speed, .acc_pct = *acc, .dec_pct = *dec};
}

}  // namespace

Move::Move(Robot& robot) : session_(robot.session()) {}

std::expected<void, Error> Move::joint(const Joints& target, const JointMoveOptions& o) {
    if (auto ok = check_target("target", target); !ok) return ok;
    const auto speed = checks::joint_speed(o.speed, 0.0001);
    if (!speed) return std::unexpected(speed.error());
    const auto acc = percent("acceleration", o.acceleration);
    if (!acc) return std::unexpected(acc.error());
    const auto dec = percent("deceleration", o.deceleration);
    if (!dec) return std::unexpected(dec.error());
    return start(sdk::MoveByJoint{
        .target = rad_to_deg(target), .speed_pct = *speed, .acc_pct = *acc, .dec_pct = *dec, .until = o.until});
}

std::expected<void, Error> Move::line(const Joints& target, const PathMoveOptions& o) {
    if (auto ok = check_target("target", target); !ok) return ok;
    const auto p = profile(o);
    if (!p) return std::unexpected(p.error());
    return start(sdk::MoveByLine{.target = rad_to_deg(target),
                                 .speed_type = o.speed_type,
                                 .speed = p->speed,
                                 .acc_pct = p->acc_pct,
                                 .dec_pct = p->dec_pct,
                                 .until = o.until});
}

std::expected<void, Error> Move::arc(const Joints& middle, const Joints& target, const PathMoveOptions& o) {
    if (auto ok = check_target("middle", middle); !ok) return ok;
    if (auto ok = check_target("target", target); !ok) return ok;
    const auto p = profile(o);
    if (!p) return std::unexpected(p.error());
    return start(sdk::MoveByArc{.middle = rad_to_deg(middle),
                                .target = rad_to_deg(target),
                                .speed_type = o.speed_type,
                                .speed = p->speed,
                                .acc_pct = p->acc_pct,
                                .dec_pct = p->dec_pct,
                                .until = o.until});
}

std::expected<void, Error> Move::rotate(const Joints& target, const PathMoveOptions& o) {
    if (auto ok = check_target("target", target); !ok) return ok;
    const auto p = profile(o);
    if (!p) return std::unexpected(p.error());
    return start(sdk::MoveByRotate{.target = rad_to_deg(target),
                                   .speed_type = o.speed_type,
                                   .speed = p->speed,
                                   .acc_pct = p->acc_pct,
                                   .dec_pct = p->dec_pct,
                                   .until = o.until});
}

template <class C>
std::expected<void, Error> Move::start(const C& command) {
    const auto c = detail::reach(session_, C::method);
    if (!c) return std::unexpected(c.error());
    const auto id = (*c)->open([raw = c->get()] {
        const auto stopped = raw->send(sdk::Stop{});
        static_cast<void>(stopped);
    });
    if (!id) return std::unexpected(id.error());
    if (auto ok = (*c)->send(command); !ok) {
        (*c)->close(*id);  // not taken: nothing is moving
        return std::unexpected(ok.error());
    }
    id_ = *id;
    started_ = Clock::now();
    ran_ = false;
    return {};
}

std::optional<std::expected<void, Error>> Move::wait_for(std::chrono::nanoseconds timeout) {
    if (id_ == 0) return std::expected<void, Error>(invalid("no move to wait for"));
    const auto c = detail::reach(session_, "wait");
    if (!c) {
        id_ = 0;
        return std::expected<void, Error>(std::unexpected(c.error()));
    }
    auto ended = (*c)->wait_stopped(started_, ran_, timeout);
    if (!ended) return std::nullopt;
    (*c)->close(std::exchange(id_, 0));  // ended: no longer the open motion
    return ended;
}

std::expected<void, Error> Move::wait() {
    for (;;)
        if (auto done = wait_for(std::chrono::seconds{1})) return *done;
}

void Move::stop() noexcept {
    try {
        if (const auto c = session_.lock()) {
            const auto stopped = c->send(sdk::Stop{});
            static_cast<void>(stopped);
        }
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

void Move::end() noexcept {
    if (id_ == 0) return;
    stop();
    if (const auto c = session_.lock()) c->close(id_);
    id_ = 0;
}

Move::Move(Move&& other) noexcept
    : session_(std::move(other.session_)),
      id_(std::exchange(other.id_, 0)),
      started_(other.started_),
      ran_(other.ran_) {}

Move& Move::operator=(Move&& other) noexcept {
    if (this != &other) {
        end();
        session_ = std::move(other.session_);
        id_ = std::exchange(other.id_, 0);
        started_ = other.started_;
        ran_ = other.ran_;
    }
    return *this;
}

Move::~Move() { end(); }

}  // namespace ec::motion
