#pragma once
// Test helper: drives an activity cycle by cycle, without I/O. The test edits
// the robot state; the bench answers every command the activity sends (true by
// default, or what `answers` says for its method) in the next cycle, records
// what was sent, and advances time by one period per cycle.

#include <chrono>
#include <expected>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "ec/command/json.hpp"
#include "ec/motion/activity.hpp"
#include "ec/motion/conditions.hpp"

namespace ec::test {

using namespace std::chrono_literals;

// Remote, servos on, synced, stopped, at rest, at the home position.
inline feedback::State ready_state() {
    feedback::State s;
    s.robot_mode = RobotMode::Remote;
    s.robot_state = RobotState::Stop;
    s.brakes_released = true;
    s.motors_synced = true;
    return s;
}

struct Sent {
    std::string method;
    command::Json params;
};

class Bench {
public:
    using Answer = std::function<std::expected<command::Json, command::Error>(const command::Request&)>;

    feedback::State state = ready_state();
    std::chrono::nanoseconds period{8ms};
    std::chrono::nanoseconds state_age{8ms};
    motion::Thresholds thresholds{};
    std::map<std::string, Answer, std::less<>> answers;  // by method; the default answers true
    std::vector<Sent> sent;                              // everything sent, in order
    motion::Clock::time_point now{std::chrono::seconds{100}};

    template <typename A>
    std::expected<motion::Phase, motion::Error> step(A& activity) {
        return run([&](const motion::Cycle& c, motion::Outbox& out) { return activity.step(c, out); });
    }

    template <typename A>
    bool close(A& activity, motion::End why) {
        return run([&](const motion::Cycle& c, motion::Outbox& out) { return activity.close(c, out, why); });
    }

    // The cycle the next step or close will see.
    [[nodiscard]] motion::Cycle cycle() const {
        motion::Cycle c;
        c.now = now;
        c.period = period;
        c.state = state;
        c.state_seq = 1;
        c.state_age = state_age;
        c.conditions = motion::observe(state, state_age, thresholds);
        c.replies = replies_;
        return c;
    }

    // The methods sent so far, in order.
    [[nodiscard]] std::vector<std::string> methods() const {
        std::vector<std::string> out;
        out.reserve(sent.size());
        for (const auto& s : sent) out.push_back(s.method);
        return out;
    }
    [[nodiscard]] std::size_t count(std::string_view method) const {
        std::size_t n = 0;
        for (const auto& s : sent) n += s.method == method ? 1u : 0u;
        return n;
    }

    // Answers `method` with an error from now on.
    void fail(std::string method, command::Error::Kind kind = command::Error::Kind::Refused) {
        answers[std::move(method)] = [kind](const command::Request&) -> std::expected<command::Json, command::Error> {
            return std::unexpected(command::Error{kind, 0, "scripted failure"});
        };
    }

private:
    template <typename F>
    auto run(F&& f) {
        const motion::Cycle c = cycle();
        motion::Outbox out;
        auto result = f(c, out);
        replies_.clear();
        for (const auto& item : out.items()) {
            sent.push_back({std::string(item.request.method), item.request.params});
            if (!item.expects_reply) continue;
            const auto it = answers.find(item.request.method);
            replies_.push_back(motion::Reply{item.request.method,
                                             it != answers.end() ? it->second(item.request) : command::Json(true)});
        }
        now += period;
        return result;
    }

    std::vector<motion::Reply> replies_;
};

}  // namespace ec::test
