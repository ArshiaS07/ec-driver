#pragma once
/// @file
/// Keeps the monitoring port (8056) connected on a thread of its own and hands
/// every frame to a callback.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>

#include "ec/feedback/framer.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/net/tcp_socket.hpp"

namespace ec::feedback {

/// A thread that reads the monitoring port for as long as the Receiver exists:
/// it connects, hands every frame to the callback as a Sample, and connects
/// again whenever the connection is lost or brings no frame for
/// Options::silence_timeout (a pulled cable can leave a connection open but
/// silent). It never sends anything to the controller.
///
///     ec::Latest<ec::feedback::Sample> latest;
///     ec::feedback::Receiver receiver("192.168.0.200", [&](const auto& s) { latest.publish(s); });
///
/// To consumers, a lost stream is samples that stop arriving: a sample's age is
/// the steady clock minus its arrival time. status() tells why.
///
/// Threads: the callback runs on the receiver's thread, one sample at a time,
/// in order. It must be quick, since the next frame waits for it, and it must
/// not throw: if it does, the receiver stops for good and Status::failure keeps
/// the message. status() and stop() may be called from any thread.
///
/// Neither copyable nor movable: the thread refers to the object.
class Receiver {
public:
    using Timeout = std::chrono::milliseconds;
    using Callback = std::function<void(const Sample&)>;

    struct Options {
        std::uint16_t port = 8056;
        Timeout connect_timeout{1000};  ///< for each connection attempt
        Timeout retry_delay{500};       ///< before each attempt after the first
        Timeout silence_timeout{1000};  ///< a connection without a frame for this long is replaced
    };

    struct Status {
        bool connected = false;
        std::uint64_t connections = 0;            ///< connections made so far
        std::uint64_t samples = 0;                ///< samples handed to the callback
        Framer::Stats framing;                    ///< over all connections
        std::optional<net::NetError> last_error;  ///< why the last connection or attempt ended
        std::string failure;                      ///< why the receiver stopped for good; empty while it runs
    };

    /// Starts the thread and returns at once: connecting happens on the thread.
    /// Throws std::system_error only if the thread cannot be started.
    Receiver(std::string host, Callback on_sample);
    Receiver(std::string host, const Options& options, Callback on_sample);

    /// Stops the thread (stop()).
    ~Receiver();

    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;
    Receiver(Receiver&&) = delete;
    Receiver& operator=(Receiver&&) = delete;

    /// Closes the connection and ends the thread; once it returns, the callback
    /// is not called again. Returns within about Options::connect_timeout (an
    /// attempt in progress is not interrupted). From the callback itself, it
    /// only asks the thread to end after the callback returns.
    void stop() noexcept;

    [[nodiscard]] Status status() const;

private:
    void run(const std::stop_token& stop);
    bool pause(const std::stop_token& stop, Timeout delay);  // false if stopped meanwhile

    std::string host_;
    Options options_;
    Callback on_sample_;
    mutable std::mutex mutex_;  // guards status_
    std::condition_variable_any wake_;
    Status status_;
    std::jthread thread_;  // last: starts once everything above exists
};

}  // namespace ec::feedback
