#include "ec/motion/move.hpp"

#include <cmath>
#include <format>
#include <utility>

#include "detail/connection.hpp"
#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"

namespace ec::motion {

std::expected<Move, Error> Move::joints(Robot& robot, const Joints& target, const MoveOptions& o) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(target[j]) <= kEc66JointLimit))
            return std::unexpected(
                Error{Error::Kind::Invalid,
                      std::format("joint {} target {} rad is beyond the joint limit", j + 1, target[j])});
    if (!(o.speed > 0.0 && o.speed <= 1.0))
        return std::unexpected(Error{Error::Kind::Invalid, std::format("speed must be in (0, 1], not {}", o.speed)});
    if (o.acceleration && !(*o.acceleration > 0.0 && *o.acceleration <= 1.0))
        return std::unexpected(
            Error{Error::Kind::Invalid, std::format("acceleration must be in (0, 1], not {}", *o.acceleration)});
    auto c = robot.share();
    const auto id = c->open([raw = c.get()] {
        const auto stopped = raw->send(command::Stop{});
        static_cast<void>(stopped);
    });
    if (!id) return std::unexpected(id.error());
    Move move(c, *id, target, o.tolerance);  // closes the registration on any return below
    command::MoveByJoint cmd{.target = rad_to_deg(target), .speed_pct = o.speed * 100.0};
    if (o.acceleration) cmd.acc_pct = static_cast<int>(std::lround(*o.acceleration * 100.0));
    if (auto ok = c->send(cmd); !ok) return std::unexpected(ok.error());
    move.started_ = Clock::now();
    return move;
}

std::optional<std::expected<void, Error>> Move::wait_for(std::chrono::nanoseconds timeout) {
    if (result_) return result_;
    if (!c_) return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Invalid, "not a move"}));
    auto stopped = c_->wait_stopped(started_, ran_, timeout);
    if (!stopped) return std::nullopt;
    result_ = std::move(*stopped);
    if (*result_) {  // at rest: at the target?
        const auto q = c_->send(command::GetJointPos{});
        if (!q) {
            result_ = std::unexpected(q.error());
        } else if (const double off = max_abs_diff(deg_to_rad(*q), target_); off > tolerance_) {
            result_ = std::unexpected(Error{
                Error::Kind::Command, std::format("the arm stopped {:.2f} deg from the target", rad_to_deg(off))});
        }
    }
    c_->close(id_);  // ended: no longer the open motion
    return result_;
}

std::expected<void, Error> Move::wait() {
    for (;;)
        if (auto done = wait_for(std::chrono::seconds{1})) return *done;
}

void Move::stop() noexcept {
    try {
        if (!c_) return;
        const auto stopped = c_->send(command::Stop{});
        static_cast<void>(stopped);
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

Move::Move(std::shared_ptr<detail::Connection> c, std::uint64_t id, const Joints& target, double tolerance)
    : c_(std::move(c)), id_(id), target_(target), tolerance_(tolerance), started_(Clock::now()) {}

Move::Move(Move&& other) noexcept
    : c_(std::move(other.c_)),
      id_(std::exchange(other.id_, 0)),
      target_(other.target_),
      tolerance_(other.tolerance_),
      started_(other.started_),
      ran_(other.ran_),
      result_(std::move(other.result_)) {}

Move& Move::operator=(Move&& other) noexcept {
    if (this != &other) {
        if (c_) c_->close(id_);
        c_ = std::move(other.c_);
        id_ = std::exchange(other.id_, 0);
        target_ = other.target_;
        tolerance_ = other.tolerance_;
        started_ = other.started_;
        ran_ = other.ran_;
        result_ = std::move(other.result_);
    }
    return *this;
}

Move::~Move() {
    if (c_) c_->close(id_);
}

}  // namespace ec::motion
