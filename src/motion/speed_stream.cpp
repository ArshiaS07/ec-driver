#include "ec/motion/speed_stream.hpp"

#include <cmath>
#include <format>
#include <utility>

#include "detail/connection.hpp"
#include "ec/sdk/movement.hpp"
#include "motion/checks.hpp"

namespace ec::motion {
namespace {

using checks::invalid;

// A positive acceleration as the manual's whole number (int, > 0) in `unit`.
std::expected<int, Error> whole(const char* what, double value_in_unit, const char* unit) {
    const auto rounded = std::lround(value_in_unit);
    if (!(value_in_unit > 0.0) || rounded < 1 || rounded > 1'000'000)
        return invalid(std::format("{} must be at least 1 {} (a whole number), not {}", what, unit, value_in_unit));
    return static_cast<int>(rounded);
}

std::expected<double, Error> duration(std::chrono::milliseconds t) {
    if (t <= std::chrono::milliseconds::zero()) return invalid("valid_for must be positive");
    return std::chrono::duration<double>(t).count();
}

void send_ending(detail::Connection& c, const SpeedStream::Ending& e) {
    if (e.joints) {
        const auto stopped = c.send(sdk::Stopj{.acc_deg_s2 = e.acc});
        static_cast<void>(stopped);
    } else {
        const auto stopped = c.send(sdk::Stopl{.acc_mm_s2 = e.acc, .acc_rot_deg_s2 = e.acc_rot});
        static_cast<void>(stopped);
    }
}

}  // namespace

SpeedStream::SpeedStream(Robot& robot) : session_(robot.session()) {}

std::expected<void, Error> SpeedStream::begin(detail::Connection& c, const Ending& ending) {
    if (id_ != 0) {
        if (ending.joints != last_.joints) return invalid("one stream moves either the joints or the TCP");
        return {};
    }
    const auto id = c.open([raw = &c, ending] { send_ending(*raw, ending); });
    if (!id) return std::unexpected(id.error());
    id_ = *id;
    return {};
}

std::expected<void, Error> SpeedStream::joints(const Joints& velocity, const SpeedOptions& o) {
    const auto acc = whole("acceleration", rad_to_deg(o.acceleration), "deg/s^2");
    if (!acc) return std::unexpected(acc.error());
    const auto t = duration(o.valid_for);
    if (!t) return std::unexpected(t.error());
    const auto c = detail::reach(session_, "moveBySpeedj");
    if (!c) return std::unexpected(c.error());
    const Ending ending{.joints = true, .acc = *acc};
    const bool first = id_ == 0;
    if (auto ok = begin(**c, ending); !ok) return ok;
    if (auto ok =
            (*c)->send(sdk::MoveBySpeedj{.velocity_deg_s = rad_to_deg(velocity), .acc_deg_s2 = *acc, .duration_s = *t});
        !ok) {
        if (first) (*c)->close(std::exchange(id_, 0));  // not taken: nothing moves
        return std::unexpected(ok.error());
    }
    last_ = ending;
    return {};
}

std::expected<void, Error> SpeedStream::tcp(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular,
                                            const SpeedOptions& o) {
    const auto acc = whole("acceleration", o.acceleration * 1000.0, "mm/s^2");
    if (!acc) return std::unexpected(acc.error());
    std::optional<int> acc_rot;
    if (o.rotational_acceleration) {
        const auto r = whole("rotational acceleration", rad_to_deg(*o.rotational_acceleration), "deg/s^2");
        if (!r) return std::unexpected(r.error());
        acc_rot = *r;
    }
    const auto t = duration(o.valid_for);
    if (!t) return std::unexpected(t.error());
    const auto c = detail::reach(session_, "moveBySpeedl");
    if (!c) return std::unexpected(c.error());
    const Ending ending{.joints = false, .acc = *acc, .acc_rot = acc_rot};
    const bool first = id_ == 0;
    if (auto ok = begin(**c, ending); !ok) return ok;
    const std::array<double, 6> v{linear.x() * 1000.0,     linear.y() * 1000.0,     linear.z() * 1000.0,
                                  rad_to_deg(angular.x()), rad_to_deg(angular.y()), rad_to_deg(angular.z())};
    if (auto ok = (*c)->send(
            sdk::MoveBySpeedl{.velocity = v, .acc_mm_s2 = *acc, .acc_rot_deg_s2 = acc_rot, .duration_s = *t});
        !ok) {
        if (first) (*c)->close(std::exchange(id_, 0));
        return std::unexpected(ok.error());
    }
    last_ = ending;
    return {};
}

std::expected<void, Error> SpeedStream::stop() {
    if (id_ == 0) return invalid("stop: nothing was sent");
    const auto c = detail::reach(session_, last_.joints ? "stopj" : "stopl");
    if (!c) {
        id_ = 0;
        return std::unexpected(c.error());
    }
    std::expected<void, Error> result;
    if (last_.joints) {
        if (auto ok = (*c)->send(sdk::Stopj{.acc_deg_s2 = last_.acc}); !ok) result = std::unexpected(ok.error());
    } else {
        if (auto ok = (*c)->send(sdk::Stopl{.acc_mm_s2 = last_.acc, .acc_rot_deg_s2 = last_.acc_rot}); !ok)
            result = std::unexpected(ok.error());
    }
    (*c)->close(std::exchange(id_, 0));
    return result;
}

void SpeedStream::end() noexcept {
    try {
        if (id_ == 0) return;
        const auto stopped = stop();
        static_cast<void>(stopped);  // nothing more to do if it fails
    } catch (...) {                  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
    id_ = 0;
}

SpeedStream::SpeedStream(SpeedStream&& other) noexcept
    : session_(std::move(other.session_)), id_(std::exchange(other.id_, 0)), last_(other.last_) {}

SpeedStream& SpeedStream::operator=(SpeedStream&& other) noexcept {
    if (this != &other) {
        end();
        session_ = std::move(other.session_);
        id_ = std::exchange(other.id_, 0);
        last_ = other.last_;
    }
    return *this;
}

SpeedStream::~SpeedStream() { end(); }

}  // namespace ec::motion
