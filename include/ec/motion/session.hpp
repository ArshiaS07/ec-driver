#pragma once
/// @file
/// The application's handle on a streamed motion: push targets, then close.

#include <chrono>
#include <expected>
#include <memory>
#include <utility>

#include "ec/motion/control_loop.hpp"
#include "ec/motion/error.hpp"
#include "ec/motion/stream.hpp"

namespace ec::motion {

/// An open stream (docs/motion.md 5.5): target() from any thread, at any rate
/// (the newest wins, stamped on arrival); close() ends it gracefully and waits.
/// Move-only; destroying an open session closes it, waiting up to
/// `close_wait` (stop() on the robot ends it at once from anywhere).
template <typename T>
class Session {
public:
    Session(std::shared_ptr<Channel<T>> channel, Job job) : channel_(std::move(channel)), job_(std::move(job)) {}

    Session(Session&&) noexcept = default;
    Session& operator=(Session&& other) noexcept {
        if (this != &other) {
            finish();
            channel_ = std::move(other.channel_);
            job_ = std::move(other.job_);
        }
        return *this;
    }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    ~Session() { finish(); }

    /// The newest target.
    void target(const T& value) {
        if (channel_) channel_->push(value);
    }

    /// Ends the stream gracefully (decelerate, let the controller's queue
    /// drain, close the family) and waits: nothing if it ended well, else why
    /// it ended (e.g. Kind::Starved if the targets had stopped coming earlier).
    std::expected<void, Error> close() {
        if (!channel_) return std::unexpected(Error{Error::Kind::Refused, "not an open session", {}});
        channel_->request_close();
        auto result = job_.wait();
        channel_.reset();
        return result;
    }

    /// Whether the stream still runs (it may have ended by itself, e.g. starved).
    [[nodiscard]] bool running() const { return channel_ && !job_.done(); }
    [[nodiscard]] const Job& job() const noexcept { return job_; }

    std::chrono::milliseconds close_wait{5'000};

private:
    void finish() {
        if (!channel_) return;
        channel_->request_close();
        (void)job_.wait_for(close_wait);
        channel_.reset();
    }

    std::shared_ptr<Channel<T>> channel_;
    Job job_;
};

}  // namespace ec::motion
