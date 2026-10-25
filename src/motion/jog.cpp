#include "ec/motion/jog.hpp"

#include <format>
#include <utility>

#include "detail/connection.hpp"
#include "ec/sdk/movement.hpp"
#include "motion/checks.hpp"

namespace ec::motion {

using checks::invalid;

Jog::Jog(Robot& robot, const JogOptions& options) : session_(robot.session()), options_(options) {}

std::expected<void, Error> Jog::step(int axis, Direction direction) {
    if (axis < 1 || axis > 6) return invalid(std::format("axis {}: 1..6", axis));
    if (!(options_.speed >= 0.0005 && options_.speed <= 1.0))
        return invalid(std::format("speed {} is outside [0.0005, 1] (0.05..100 %)", options_.speed));
    if (!sdk::is_valid(options_.frame)) return invalid("not a coordinate system");
    const auto c = detail::reach(session_, "jog");
    if (!c) return std::unexpected(c.error());

    if (id_ == 0) {  // the first step: register, select the frame
        const auto id = (*c)->open([raw = c->get()] {
            const auto stopped = raw->send(sdk::Stop{});
            static_cast<void>(stopped);
        });
        if (!id) return std::unexpected(id.error());
        if (auto ok = (*c)->send(sdk::SetCurrentCoord{.system = options_.frame}); !ok) {
            (*c)->close(*id);
            return std::unexpected(ok.error());
        }
        id_ = *id;
        ran_ = false;
    }
    // Two indices per axis, minus then plus [S 2.2.3.8].
    const int index = 2 * (axis - 1) + (direction == Direction::Plus ? 1 : 0);
    if (auto ok = (*c)->send(sdk::Jog{.index = index, .speed_pct = options_.speed * 100.0}); !ok)
        return std::unexpected(ok.error());
    last_step_ = Clock::now();
    return {};
}

std::optional<std::expected<void, Error>> Jog::wait_for(std::chrono::nanoseconds timeout) {
    if (id_ == 0) return std::expected<void, Error>(invalid("no jog to wait for"));
    const auto c = detail::reach(session_, "wait");
    if (!c) {
        id_ = 0;
        return std::expected<void, Error>(std::unexpected(c.error()));
    }
    auto ended = (*c)->wait_stopped(last_step_, ran_, timeout);
    if (!ended) return std::nullopt;
    (*c)->close(std::exchange(id_, 0));
    return ended;
}

std::expected<void, Error> Jog::stop() {
    const auto c = detail::reach(session_, "stop");
    if (!c) {
        id_ = 0;
        return std::unexpected(c.error());
    }
    auto ok = (*c)->send(sdk::Stop{});
    if (id_ != 0) (*c)->close(std::exchange(id_, 0));
    if (!ok) return std::unexpected(ok.error());
    return {};
}

void Jog::end() noexcept {
    try {
        if (id_ == 0) return;
        const auto stopped = stop();
        static_cast<void>(stopped);  // nothing more to do if it fails
    } catch (...) {                  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
    id_ = 0;
}

Jog::Jog(Jog&& other) noexcept
    : session_(std::move(other.session_)),
      options_(other.options_),
      id_(std::exchange(other.id_, 0)),
      last_step_(other.last_step_),
      ran_(other.ran_) {}

Jog& Jog::operator=(Jog&& other) noexcept {
    if (this != &other) {
        end();
        session_ = std::move(other.session_);
        options_ = other.options_;
        id_ = std::exchange(other.id_, 0);
        last_step_ = other.last_step_;
        ran_ = other.ran_;
    }
    return *this;
}

Jog::~Jog() { end(); }

}  // namespace ec::motion
