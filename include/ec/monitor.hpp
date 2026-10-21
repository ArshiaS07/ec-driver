#pragma once
/// @file
/// ec::Monitor: the robot's monitoring path, port 8056. The controller pushes a
/// frame about every 8 ms; the monitor receives each on a thread of its own
/// (feedback::Receiver), decodes it into an ec::State and keeps the newest.
/// Reading it sends nothing to the controller and waits for no call: it is the
/// path beside the command connection (8055), on which the parts ask the
/// controller one call at a time. A Robot holds one (Robot::monitor()); it can
/// also run on its own.
///
///     ec::Monitor monitor("192.168.0.200", {});
///     if (auto s = monitor.next(0, 2s)) { ... }        // the first state
///     auto s = monitor.latest();                        // the newest, at once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "ec/feedback/receiver.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/latest.hpp"
#include "ec/state.hpp"

namespace ec {

/// See the file comment.
///
/// Threads: latest(), next() and status() may be called from any thread. The
/// callbacks run on the monitor's thread, one frame at a time, in order; they
/// must be quick (the next frame waits for them) and must not throw (the
/// receiver would stop for good; status() keeps why).
///
/// Neither copyable nor movable: its thread refers to it.
class Monitor {
public:
    struct Options {
        feedback::Receiver::Options receiver{};  ///< the port (8056), timeouts, reconnection
        /// Every frame as received, before it is decoded (frames that do not
        /// decode included): for recordings.
        std::function<void(const feedback::Sample&)> on_sample{};
        /// Every state, once it is the newest.
        std::function<void(const State&)> on_state{};
    };

    struct Status {
        feedback::Receiver::Status receiver{};  ///< connected, connections, framing, why the last one ended
        std::uint64_t states = 0;               ///< frames decoded into states
        std::uint64_t undecodable = 0;          ///< frames that did not decode, skipped
    };

    /// A monitor that receives nothing: latest() stays empty. What a Robot
    /// connected without the monitoring stream holds.
    Monitor() noexcept;
    /// Starts receiving from `host` and returns at once: connecting happens on
    /// the monitor's thread, and again whenever the stream is lost. Throws
    /// std::system_error only if the thread cannot be started.
    Monitor(std::string host, Options options);
    /// Stops the thread; no callback runs after it.
    ~Monitor();

    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;
    Monitor(Monitor&&) = delete;
    Monitor& operator=(Monitor&&) = delete;

    /// The newest state; nullopt before the first. Its State::age() tells how
    /// old it is: a lost stream is states that stop arriving (status() says why).
    [[nodiscard]] std::optional<State> latest() const;
    /// The newest state once its seq is greater than `after`, waiting up to
    /// `timeout`; at once if it already is. Pass 0 for the first state, or the
    /// seq of the last state seen for the next one (states in between are
    /// skipped). nullopt if none came in time.
    [[nodiscard]] std::optional<State> next(std::uint64_t after, std::chrono::nanoseconds timeout) const;
    [[nodiscard]] Status status() const;

private:
    void on_sample(const feedback::Sample& sample);  // on the receiver's thread

    Options options_;
    Latest<State> latest_;
    std::atomic<std::uint64_t> states_{0};  // written by the receiver's thread only
    std::atomic<std::uint64_t> undecodable_{0};
    std::unique_ptr<feedback::Receiver> receiver_;  // last: stopped first, while the rest still exists
};

}  // namespace ec
