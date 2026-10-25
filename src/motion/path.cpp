#include "ec/motion/path.hpp"

#include <format>
#include <utility>

#include "detail/connection.hpp"
#include "ec/sdk/param.hpp"
#include "motion/checks.hpp"

namespace ec::motion {
namespace {

using checks::invalid;

// One waypoint as addPathPoint takes it, or why it cannot be sent.
std::expected<sdk::AddPathPoint, Error> to_call(const Waypoint& w, std::size_t k, bool last) {
    const auto where = [k](const Error& e) {
        return std::unexpected(Error{e.kind, std::format("waypoint {}: {}", k, e.message)});
    };
    if (auto ok = checks::check_target("target", w.target); !ok) return where(ok.error());
    sdk::AddPathPoint call{.point = rad_to_deg(w.target), .move_type = w.move, .speed_type = w.speed_type};
    if (w.move == sdk::PathMoveType::Joint) {
        const auto speed = checks::joint_speed(w.speed, 0.01);
        if (!speed) return where(speed.error());
        call.speed = *speed;
    } else {
        const auto speed = checks::tcp_speed(w.speed_type, w.speed);
        if (!speed) return where(speed.error());
        call.speed = *speed;
    }
    const auto acc = checks::percent("acceleration", w.acceleration);
    if (!acc) return where(acc.error());
    const auto dec = checks::percent("deceleration", w.deceleration);
    if (!dec) return where(dec.error());
    call.acc_pct = *acc;
    call.dec_pct = *dec;
    if (!(w.blend_radius >= 0.0 && w.blend_radius <= 2.608))
        return where(
            Error{Error::Kind::Invalid, std::format("blend radius {} m is outside [0, 2.608]", w.blend_radius)});
    if (last && w.blend_radius != 0.0)
        return where(Error{Error::Kind::Invalid, "the last waypoint does not blend: its radius must be 0"});
    call.blend_radius_mm = w.blend_radius * 1000.0;
    call.until = w.until;
    return call;
}

}  // namespace

Path::Path(Robot& robot, std::vector<Waypoint> waypoints)
    : session_(robot.session()), waypoints_(std::move(waypoints)) {}

std::expected<void, Error> Path::run() {
    if (waypoints_.empty()) return invalid("a path needs at least one waypoint");
    std::vector<sdk::AddPathPoint> calls;
    calls.reserve(waypoints_.size());
    for (std::size_t k = 0; k < waypoints_.size(); ++k) {
        auto call = to_call(waypoints_[k], k, k + 1 == waypoints_.size());
        if (!call) return std::unexpected(call.error());
        calls.push_back(std::move(*call));
    }
    const auto reached = detail::reach(session_, "moveByPath");
    if (!reached) return std::unexpected(reached.error());
    detail::Connection& c = **reached;

    const auto id = c.open([raw = &c] {
        const auto stopped = raw->send(sdk::Stop{});
        static_cast<void>(stopped);
    });
    if (!id) return std::unexpected(id.error());
    // Until it runs, a failure gives the registration back; nothing moves.
    const auto fail = [&c, &id](Error e) {
        c.close(*id);
        return std::unexpected(std::move(e));
    };
    if (auto ok = c.send(sdk::ClearPathPoint{}); !ok) return fail(ok.error());
    for (std::size_t k = 0; k < calls.size(); ++k)
        if (auto ok = c.send(calls[k]); !ok)
            return fail(Error{ok.error().kind, std::format("waypoint {}: {}", k, ok.error().message)});
    const auto count = c.send(sdk::MoveByPath{});
    if (!count) return fail(count.error());
    if (*count != static_cast<int>(calls.size())) {
        const auto stopped = c.send(sdk::Stop{});  // it may have started: not the path that was loaded
        static_cast<void>(stopped);
        return fail(
            Error{Error::Kind::Command,
                  std::format("moveByPath: the controller holds {} waypoints, {} were added", *count, calls.size())});
    }
    id_ = *id;
    started_ = Clock::now();
    ran_ = false;
    return {};
}

std::expected<int, Error> Path::index() const { return detail::send(session_, sdk::GetPathPointIndex{}); }

std::optional<std::expected<void, Error>> Path::wait_for(std::chrono::nanoseconds timeout) {
    if (id_ == 0) return std::expected<void, Error>(invalid("no path to wait for"));
    const auto c = detail::reach(session_, "wait");
    if (!c) {
        id_ = 0;
        return std::expected<void, Error>(std::unexpected(c.error()));
    }
    auto ended = (*c)->wait_stopped(started_, ran_, timeout);
    if (!ended) return std::nullopt;
    (*c)->close(std::exchange(id_, 0));
    return ended;
}

std::expected<void, Error> Path::wait() {
    for (;;)
        if (auto done = wait_for(std::chrono::seconds{1})) return *done;
}

void Path::stop() noexcept {
    try {
        if (const auto c = session_.lock()) {
            const auto stopped = c->send(sdk::Stop{});
            static_cast<void>(stopped);
        }
    } catch (...) {  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

void Path::end() noexcept {
    if (id_ == 0) return;
    stop();
    if (const auto c = session_.lock()) c->close(id_);
    id_ = 0;
}

Path::Path(Path&& other) noexcept
    : session_(std::move(other.session_)),
      waypoints_(std::move(other.waypoints_)),
      id_(std::exchange(other.id_, 0)),
      started_(other.started_),
      ran_(other.ran_) {}

Path& Path::operator=(Path&& other) noexcept {
    if (this != &other) {
        end();
        session_ = std::move(other.session_);
        waypoints_ = std::move(other.waypoints_);
        id_ = std::exchange(other.id_, 0);
        started_ = other.started_;
        ran_ = other.ran_;
    }
    return *this;
}

Path::~Path() { end(); }

}  // namespace ec::motion
