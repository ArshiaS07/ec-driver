#include "ec/motion/move.hpp"

#include <cmath>
#include <format>
#include <string>
#include <utility>

#include "detail/connection.hpp"

namespace ec::motion {
namespace {

std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message)});
}

// Whether every joint is within the manual's [-360, 360] deg.
std::expected<void, Error> check_target(std::string_view name, const Joints& q) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(q[j]) <= kEc66JointLimit))
            return invalid(std::format("{}: joint {} at {} rad is outside [-360, 360] deg", name, j + 1, q[j]));
    return {};
}

// A fraction as the manual's percentage, int [1, 100]; none stays the controller's default.
std::expected<std::optional<int>, Error> percent(std::string_view name, const std::optional<double>& fraction) {
    if (!fraction) return std::optional<int>{};
    if (!(*fraction >= 0.01 && *fraction <= 1.0))
        return invalid(std::format("{} must be in [0.01, 1], not {}", name, *fraction));
    return std::optional<int>(static_cast<int>(std::lround(*fraction * 100.0)));
}

// A move of the TCP's speed and acceleration, in the controller's units.
struct Profile {
    double speed = 0.0;  // mm/s or deg/s, as speed_type says
    std::optional<int> acc_pct{};
    std::optional<int> dec_pct{};
};

std::expected<Profile, Error> profile(const PathMoveOptions& o) {
    Profile p;
    switch (o.speed_type) {
        case sdk::SpeedType::Linear:
            if (!(o.speed >= 0.001 && o.speed <= 3.0))
                return invalid(std::format("speed must be in [0.001, 3] m/s, not {} m/s", o.speed));
            p.speed = o.speed * 1000.0;
            break;
        case sdk::SpeedType::Rotational:
            if (!(o.speed >= deg_to_rad(1.0) && o.speed <= deg_to_rad(300.0)))
                return invalid(std::format("speed must be in [1, 300] deg/s, not {} deg/s", rad_to_deg(o.speed)));
            p.speed = rad_to_deg(o.speed);
            break;
        case sdk::SpeedType::AbsoluteLinear: p.speed = o.speed * 1000.0; break;
        case sdk::SpeedType::AbsoluteRotational: p.speed = rad_to_deg(o.speed); break;
        default: return invalid("unknown speed type");
    }
    auto acc = percent("acceleration", o.acceleration);
    if (!acc) return std::unexpected(acc.error());
    auto dec = percent("deceleration", o.deceleration);
    if (!dec) return std::unexpected(dec.error());
    p.acc_pct = *acc;
    p.dec_pct = *dec;
    return p;
}

}  // namespace

Move::Move(Robot& robot) : c_(robot.share()) {}

std::expected<void, Error> Move::joint(const Joints& target, const JointMoveOptions& o) {
    if (auto ok = check_target("target", target); !ok) return ok;
    if (!(o.speed >= 0.0001 && o.speed <= 1.0))
        return invalid(std::format("speed must be in [0.0001, 1], not {}", o.speed));
    const auto acc = percent("acceleration", o.acceleration);
    if (!acc) return std::unexpected(acc.error());
    const auto dec = percent("deceleration", o.deceleration);
    if (!dec) return std::unexpected(dec.error());
    return start(sdk::MoveByJoint{.target = rad_to_deg(target),
                                  .speed_pct = o.speed * 100.0,
                                  .acc_pct = *acc,
                                  .dec_pct = *dec,
                                  .until = o.until});
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
    if (!c_) return invalid("a moved-from Move");
    const auto id = c_->open([raw = c_.get()] {
        const auto stopped = raw->send(sdk::Stop{});
        static_cast<void>(stopped);
    });
    if (!id) return std::unexpected(id.error());
    if (auto ok = c_->send(command); !ok) {
        c_->close(*id);  // not taken: nothing is moving
        return std::unexpected(ok.error());
    }
    id_ = *id;
    started_ = Clock::now();
    ran_ = false;
    return {};
}

std::optional<std::expected<void, Error>> Move::wait_for(std::chrono::nanoseconds timeout) {
    if (id_ == 0) return std::expected<void, Error>(invalid("no move to wait for"));
    auto ended = c_->wait_stopped(started_, ran_, timeout);
    if (!ended) return std::nullopt;
    c_->close(std::exchange(id_, 0));  // ended: no longer the open motion
    return ended;
}

std::expected<void, Error> Move::wait() {
    for (;;)
        if (auto done = wait_for(std::chrono::seconds{1})) return *done;
}

void Move::stop() noexcept {
    try {
        if (!c_) return;
        const auto stopped = c_->send(sdk::Stop{});
        static_cast<void>(stopped);
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

Move::Move(Move&& other) noexcept
    : c_(std::move(other.c_)), id_(std::exchange(other.id_, 0)), started_(other.started_), ran_(other.ran_) {}

Move& Move::operator=(Move&& other) noexcept {
    if (this != &other) {
        if (id_ != 0) c_->close(id_);
        c_ = std::move(other.c_);
        id_ = std::exchange(other.id_, 0);
        started_ = other.started_;
        ran_ = other.ran_;
    }
    return *this;
}

Move::~Move() {
    if (id_ != 0) c_->close(id_);
}

}  // namespace ec::motion
