#include "ec/motion/timestamped.hpp"

#include <format>
#include <thread>
#include <utility>

#include "detail/connection.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/trajectory.hpp"

namespace ec::motion {

std::expected<Timestamped, Error> Timestamped::upload(Robot& robot, const JointTrajectory& t,
                                                      const TimestampedOptions& o) {
    using namespace std::chrono_literals;
    if (auto ok = validate(t, o.limits); !ok) return std::unexpected(Error{Error::Kind::Invalid, ok.error()});
    auto c = robot.share();
    const auto id = c->open([raw = c.get()] {
        const auto stopped = raw->send(sdk::StopTrajectory{});
        static_cast<void>(stopped);
    });
    if (!id) return std::unexpected(id.error());
    Timestamped motion(c, *id);  // closes the registration on any return below

    const auto now = c->send(sdk::GetJointPos{});
    if (!now) return std::unexpected(now.error());
    if (const double off = max_abs_diff(deg_to_rad(*now), t.position.front()); off > o.start_tolerance)
        return std::unexpected(
            Error{Error::Kind::Invalid,
                  std::format("the arm is {:.2f} deg from the first point; move there first", rad_to_deg(off))});

    if (auto ok = c->send(sdk::StartPushPos{.point_count = static_cast<int>(t.size()),
                                            .point_type = sdk::PointType::Joint,
                                            .reference = rad_to_deg(t.position.front()),
                                            .frame = {},
                                            .replies = true});
        !ok)
        return std::unexpected(ok.error());
    std::this_thread::sleep_for(200ms);
    for (std::size_t k = 0; k < t.size(); ++k) {
        if (c->stop_requested.load()) return std::unexpected(Error{Error::Kind::Stopped, "during the upload"});
        if (auto ok = c->send(sdk::PushPos{.time_s = t.time[k], .point = rad_to_deg(t.position[k])}); !ok)
            return std::unexpected(Error{ok.error().kind, std::format("point {}: {}", k, ok.error().message)});
    }
    std::this_thread::sleep_for(1s);
    if (auto ok = c->send(sdk::StopPushPos{}); !ok) return std::unexpected(ok.error());
    const auto check = c->send(sdk::CheckTrajectory{});
    if (!check) return std::unexpected(check.error());
    if (*check != sdk::TrajectoryCheck::Valid)
        return std::unexpected(Error{
            Error::Kind::Command,
            std::format("check_trajectory: the controller found the upload invalid ({})", static_cast<int>(*check))});
    return motion;
}

std::expected<void, Error> Timestamped::start(double speed) {
    if (!c_ || result_) return std::unexpected(Error{Error::Kind::Invalid, "not an uploaded trajectory"});
    if (!(speed > 0.0 && speed <= 1.0))
        return std::unexpected(Error{Error::Kind::Invalid, std::format("speed must be in (0, 1], not {}", speed)});
    if (auto ok = c_->send(sdk::StartTrajectory{.speed_pct = speed * 100.0}); !ok) return std::unexpected(ok.error());
    started_ = true;
    started_at_ = Clock::now();
    return {};
}

std::expected<void, Error> Timestamped::pause() {
    if (!c_) return std::unexpected(Error{Error::Kind::Invalid, "not an uploaded trajectory"});
    return c_->send(sdk::PauseTrajectory{}).transform([](auto&&) {});
}

std::expected<void, Error> Timestamped::resume() {
    if (!c_) return std::unexpected(Error{Error::Kind::Invalid, "not an uploaded trajectory"});
    return c_->send(sdk::ResumeTrajectory{}).transform([](auto&&) {});
}

std::optional<std::expected<void, Error>> Timestamped::wait_for(std::chrono::nanoseconds timeout) {
    if (result_) return result_;
    if (!c_ || !started_)
        return std::expected<void, Error>(std::unexpected(Error{Error::Kind::Invalid, "not started"}));
    auto stopped = c_->wait_stopped(started_at_, ran_, timeout);
    if (!stopped) return std::nullopt;
    result_ = std::move(*stopped);
    c_->close(id_);  // ended: no longer the open motion
    return result_;
}

std::expected<void, Error> Timestamped::wait() {
    for (;;)
        if (auto done = wait_for(std::chrono::seconds{1})) return *done;
}

void Timestamped::stop() noexcept {
    try {
        if (!c_) return;
        const auto stopped = c_->send(sdk::StopTrajectory{});
        static_cast<void>(stopped);
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

Timestamped::Timestamped(Timestamped&& other) noexcept
    : c_(std::move(other.c_)),
      id_(std::exchange(other.id_, 0)),
      started_(other.started_),
      started_at_(other.started_at_),
      ran_(other.ran_),
      result_(std::move(other.result_)) {}

Timestamped& Timestamped::operator=(Timestamped&& other) noexcept {
    if (this != &other) {
        if (c_) c_->close(id_);
        c_ = std::move(other.c_);
        id_ = std::exchange(other.id_, 0);
        started_ = other.started_;
        started_at_ = other.started_at_;
        ran_ = other.ran_;
        result_ = std::move(other.result_);
    }
    return *this;
}

Timestamped::~Timestamped() {
    if (c_) c_->close(id_);
}

}  // namespace ec::motion
