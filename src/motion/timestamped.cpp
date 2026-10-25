#include "ec/motion/timestamped.hpp"

#include <format>
#include <string>
#include <thread>
#include <utility>

#include "detail/connection.hpp"
#include "ec/sdk/trajectory.hpp"

namespace ec::motion {
namespace {

std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message)});
}

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

}  // namespace

Timestamped::Timestamped(Robot& robot, JointTrajectory trajectory)
    : session_(robot.session()), trajectory_(std::move(trajectory)) {}

std::expected<void, Error> Timestamped::upload() {
    using namespace std::chrono_literals;
    const JointTrajectory& t = trajectory_;
    if (t.size() < 3) return invalid(std::format("{} points; the timestamp service takes at least 3", t.size()));
    if (t.front().stamp != Clock::duration::zero())
        return invalid(std::format("the first stamp is {} s, not 0", seconds(t.front().stamp)));
    for (std::size_t k = 1; k < t.size(); ++k)
        if (!(t[k].stamp > t[k - 1].stamp))
            return invalid(std::format("point {}: its stamp, {} s, does not increase", k, seconds(t[k].stamp)));
    const auto reached = detail::reach(session_, "upload");
    if (!reached) return std::unexpected(reached.error());
    detail::Connection& c = **reached;

    const auto id = c.open([raw = &c] {
        const auto stopped = raw->send(sdk::StopTrajectory{});
        static_cast<void>(stopped);
    });
    if (!id) return std::unexpected(id.error());
    // Until the upload is done, a failure gives the registration back; nothing is sent to clean up.
    const auto fail = [&c, &id](Error e) {
        c.close(*id);
        return std::unexpected(std::move(e));
    };
    if (auto ok = c.send(sdk::StartPushPos{.point_count = static_cast<int>(t.size()),
                                           .point_type = sdk::PointType::Joint,
                                           .reference = rad_to_deg(t.front().value),
                                           .frame = {},
                                           .replies = true});
        !ok)
        return fail(ok.error());
    std::this_thread::sleep_for(200ms);
    for (std::size_t k = 0; k < t.size(); ++k) {
        if (c.stop_requested.load()) return fail(Error{Error::Kind::Stopped, "during the upload"});
        if (auto ok = c.send(sdk::PushPos{.time_s = seconds(t[k].stamp), .point = rad_to_deg(t[k].value)}); !ok)
            return fail(Error{ok.error().kind, std::format("point {}: {}", k, ok.error().message)});
    }
    std::this_thread::sleep_for(1s);
    if (auto ok = c.send(sdk::StopPushPos{}); !ok) return fail(ok.error());
    const auto check = c.send(sdk::CheckTrajectory{});
    if (!check) return fail(check.error());
    if (*check != sdk::TrajectoryCheck::Valid)
        return fail(Error{
            Error::Kind::Command,
            std::format("check_trajectory: the controller found the upload invalid ({})", static_cast<int>(*check))});
    id_ = *id;
    return {};
}

std::expected<void, Error> Timestamped::start(double speed) {
    if (id_ == 0 || started_) return invalid("start: upload first");
    if (!(speed >= 0.001 && speed <= 1.0))
        return invalid(
            std::format("speed must be in [0.001, 1], not {}: above 1 the arm would move faster than "
                        "the trajectory was planned for",
                        speed));
    const auto c = detail::reach(session_, "start_trajectory");
    if (!c) return std::unexpected(c.error());
    if (auto ok = (*c)->send(sdk::StartTrajectory{.speed_pct = speed * 100.0}); !ok) return std::unexpected(ok.error());
    started_ = true;
    started_at_ = Clock::now();
    ran_ = false;
    return {};
}

std::expected<void, Error> Timestamped::pause() {
    if (id_ == 0) return invalid("pause: nothing uploaded");
    const auto c = detail::reach(session_, "pause_trajectory");
    if (!c) return std::unexpected(c.error());
    return (*c)->send(sdk::PauseTrajectory{}).transform([](auto&&) {});
}

std::expected<void, Error> Timestamped::resume() {
    if (id_ == 0) return invalid("resume: nothing uploaded");
    const auto c = detail::reach(session_, "resume_trajectory");
    if (!c) return std::unexpected(c.error());
    return (*c)->send(sdk::ResumeTrajectory{}).transform([](auto&&) {});
}

std::optional<std::expected<void, Error>> Timestamped::wait_for(std::chrono::nanoseconds timeout) {
    if (id_ == 0 || !started_) return std::expected<void, Error>(invalid("no run to wait for"));
    const auto c = detail::reach(session_, "wait");
    if (!c) {
        id_ = 0;
        started_ = false;
        return std::expected<void, Error>(std::unexpected(c.error()));
    }
    auto ended = (*c)->wait_stopped(started_at_, ran_, timeout);
    if (!ended) return std::nullopt;
    (*c)->close(std::exchange(id_, 0));  // ended: no longer the open motion
    started_ = false;
    return ended;
}

std::expected<void, Error> Timestamped::wait() {
    for (;;)
        if (auto done = wait_for(std::chrono::seconds{1})) return *done;
}

void Timestamped::stop() noexcept {
    try {
        if (const auto c = session_.lock()) {
            const auto stopped = c->send(sdk::StopTrajectory{});
            static_cast<void>(stopped);
        }
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

void Timestamped::end() noexcept {
    if (id_ == 0) return;
    if (started_) stop();  // uploaded only: nothing moves, nothing to stop
    if (const auto c = session_.lock()) c->close(id_);
    id_ = 0;
    started_ = false;
}

Timestamped::Timestamped(Timestamped&& other) noexcept
    : session_(std::move(other.session_)),
      trajectory_(std::move(other.trajectory_)),
      id_(std::exchange(other.id_, 0)),
      started_(std::exchange(other.started_, false)),
      started_at_(other.started_at_),
      ran_(other.ran_) {}

Timestamped& Timestamped::operator=(Timestamped&& other) noexcept {
    if (this != &other) {
        end();
        session_ = std::move(other.session_);
        trajectory_ = std::move(other.trajectory_);
        id_ = std::exchange(other.id_, 0);
        started_ = std::exchange(other.started_, false);
        started_at_ = other.started_at_;
        ran_ = other.ran_;
    }
    return *this;
}

Timestamped::~Timestamped() { end(); }

}  // namespace ec::motion
