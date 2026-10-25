#pragma once
// Ctrl+C for the tools that move the arm: while a StopOnInterrupt lives,
// Ctrl+C (or SIGTERM) calls robot.motion().stop(). A signal handler may only set a
// flag; a thread of the guard's own looks at it every 10 ms and stops the
// robot from there.

#include <atomic>
#include <chrono>
#include <csignal>
#include <stop_token>
#include <thread>

#include "ec/robot.hpp"

namespace cli::detail {
inline volatile std::sig_atomic_t interrupted = 0;
}  // namespace cli::detail

extern "C" inline void ec_cli_on_interrupt(int /*signal*/) { cli::detail::interrupted = 1; }

namespace cli {

class StopOnInterrupt {
public:
    explicit StopOnInterrupt(ec::Robot& robot)
        : watcher_([this, &robot](const std::stop_token& stop) {
              while (!stop.stop_requested()) {
                  if (detail::interrupted != 0) {
                      detail::interrupted = 0;
                      hit_.store(true);
                      robot.motion().stop();
                  }
                  std::this_thread::sleep_for(std::chrono::milliseconds{10});
              }
          }) {
        std::signal(SIGINT, ec_cli_on_interrupt);
        std::signal(SIGTERM, ec_cli_on_interrupt);
    }
    ~StopOnInterrupt() {
        std::signal(SIGINT, SIG_DFL);
        std::signal(SIGTERM, SIG_DFL);
    }
    StopOnInterrupt(const StopOnInterrupt&) = delete;
    StopOnInterrupt& operator=(const StopOnInterrupt&) = delete;
    StopOnInterrupt(StopOnInterrupt&&) = delete;
    StopOnInterrupt& operator=(StopOnInterrupt&&) = delete;

    // Whether Ctrl+C was pressed while it lived.
    [[nodiscard]] bool interrupted() const noexcept { return hit_.load(); }

private:
    std::atomic<bool> hit_{false};
    std::jthread watcher_;  // last: stopped first
};

}  // namespace cli
