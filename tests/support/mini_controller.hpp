#pragma once
// Ports 8055, 8056 and 8058 of a scripted controller on 127.0.0.1, for the
// tests of Robot, its parts and the motion families: every call is recorded
// and answered from a script (true unless told otherwise); getRobotState
// follows a queue of states; get_joint_pos answers the joints set; frames go
// out every 8 ms; the log port records each request and answers the lines set.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ec/robot.hpp"
#include "ec/sdk/json.hpp"
#include "support/frame_builder.hpp"
#include "support/loopback_server.hpp"

namespace ec::test {

class MiniController {
public:
    struct Call {
        std::string method;
        sdk::Json params;
    };
    // An answer: the result, already JSON (e.g. "true", "[1,2]"), or nullopt for an error reply.
    using Answer = std::function<std::optional<std::string>(const sdk::Json& params)>;

    explicit MiniController(bool frames = true) {
        commands_thread_ = std::jthread([this](const std::stop_token& stop) { serve_commands(stop); });
        if (frames) frames_thread_ = std::jthread([this](const std::stop_token& stop) { send_frames(stop); });
        log_thread_ = std::jthread([this](const std::stop_token& stop) { serve_log(stop); });
    }
    ~MiniController() { stop_.store(true); }
    MiniController(const MiniController&) = delete;
    MiniController& operator=(const MiniController&) = delete;

    [[nodiscard]] RobotOptions options(bool monitor = true) const {
        using namespace std::chrono_literals;
        return RobotOptions{.host = "127.0.0.1",
                            .command_port = commands_.port(),
                            .monitor_port = frames_.port(),
                            .log_port = log_.port(),
                            .monitor = monitor,
                            .reply_timeout = 2s};
    }

    void answer(const std::string& method, Answer a) {
        const std::lock_guard lock(mutex_);
        answers_[method] = std::move(a);
    }
    void answer(const std::string& method, std::string result) {
        answer(method, [result = std::move(result)](const sdk::Json&) { return std::optional(result); });
    }
    // getRobotState answers these states in turn, then `then` for ever.
    void states(std::vector<int> queue, int then = 0) {
        const std::lock_guard lock(mutex_);
        states_.assign(queue.begin(), queue.end());
        state_after_ = then;
    }
    void joints_deg(const std::array<double, 6>& q) {
        const std::lock_guard lock(mutex_);
        joints_ = q;
    }

    // What the log port answers: these lines, each ended by "\n".
    void log_lines(std::vector<std::string> lines) {
        const std::lock_guard lock(mutex_);
        log_lines_ = std::move(lines);
    }
    // The log requests so far, each as its lines joined by '|', e.g. "Type=Info|50|exit".
    [[nodiscard]] std::vector<std::string> log_requests() const {
        const std::lock_guard lock(mutex_);
        return log_requests_;
    }

    [[nodiscard]] std::vector<Call> calls() const {
        const std::lock_guard lock(mutex_);
        return calls_;
    }
    [[nodiscard]] std::vector<std::string> methods() const {
        std::vector<std::string> out;
        for (const auto& c : calls()) out.push_back(c.method);
        return out;
    }
    [[nodiscard]] std::size_t count(const std::string& method) const {
        std::size_t n = 0;
        for (const auto& c : calls()) n += c.method == method ? 1u : 0u;
        return n;
    }
    // The parameters of the last call of `method`.
    [[nodiscard]] sdk::Json last(const std::string& method) const {
        const auto all = calls();
        for (auto it = all.rbegin(); it != all.rend(); ++it)
            if (it->method == method) return it->params;
        return {};
    }
    void forget() {
        const std::lock_guard lock(mutex_);
        calls_.clear();
    }

private:
    std::optional<std::string> respond(const std::string& method, const sdk::Json& params) {
        const std::lock_guard lock(mutex_);
        calls_.push_back({method, params});
        if (const auto it = answers_.find(method); it != answers_.end()) return it->second(params);
        if (method == "getRobotState") {
            if (states_.empty()) return std::to_string(state_after_);
            const int s = states_.front();
            states_.pop_front();
            return std::to_string(s);
        }
        if (method == "get_joint_pos") return sdk::Json(joints_).dump();
        if (method == "getRobotMode") return "2";
        if (method == "getMotorStatus" || method == "getServoStatus") return "true";
        if (method == "get_transparent_transmission_state") return "0";
        if (method == "check_trajectory") return "0";
        if (method == "inverseKinematic")  // the reference itself: the arm stands still
            return params.contains("referencePos") ? params["referencePos"].dump() : "[0,0,0,0,0,0]";
        if (method == "positiveKinematic") return "[400.0,-50.0,300.0,3.0,0.1,-0.2]";
        return "true";
    }

    void serve_commands(const std::stop_token& stop) {
        while (auto connection = accept_or_quit(commands_, stop)) {
            Peer& peer = *connection;
            for (;;) {
                const std::string line = peer.receive_line();
                if (line.empty()) break;
                const auto request = sdk::Json::parse(line);
                const std::string method = request["method"].get<std::string>();
                const sdk::Json params = request.contains("params") ? request["params"] : sdk::Json{};
                const auto result = respond(method, params);
                const sdk::Json reply = result
                                            ? sdk::Json{{"jsonrpc", "2.0"}, {"result", *result}, {"id", request["id"]}}
                                            : sdk::Json{{"jsonrpc", "2.0"},
                                                        {"error", {{"code", -32693}, {"message", "scripted failure"}}},
                                                        {"id", request["id"]}};
                peer.send(reply.dump() + "\n");
            }
        }
    }

    void send_frames(const std::stop_token& stop) {
        while (auto connection = accept_or_quit(frames_, stop)) {
            Peer& peer = *connection;
            const auto frame = make_frame(FrameFields{});
            const std::string bytes(reinterpret_cast<const char*>(frame.data()), frame.size());
            try {
                while (!stop.stop_requested() && !stop_.load()) {
                    peer.send(bytes);
                    std::this_thread::sleep_for(std::chrono::milliseconds{8});
                }
            } catch (const std::runtime_error&) {
                peer.close();  // the robot closed the monitor connection: wait for the next one
            }
        }
    }

    // The manual's exchange: the type, a count or "all", the reply, "exit".
    void serve_log(const std::stop_token& stop) {
        while (auto connection = accept_or_quit(log_, stop)) {
            Peer& peer = *connection;
            std::string request = peer.receive_line();
            request += "|" + peer.receive_line();
            std::string reply;
            {
                const std::lock_guard lock(mutex_);
                for (const auto& line : log_lines_) reply += line + "\n";
            }
            try {
                if (!reply.empty()) peer.send(reply);
            } catch (const std::runtime_error&) {
                continue;  // the client is gone
            }
            request += "|" + peer.receive_line();
            const std::lock_guard lock(mutex_);
            log_requests_.push_back(std::move(request));
        }
    }

    // Waits for the robot to connect, in slices so that a stop is seen; nullopt once stopped.
    std::optional<Peer> accept_or_quit(LoopbackServer& server, const std::stop_token& stop) {
        while (!stop.stop_requested() && !stop_.load()) {
            try {
                return server.accept(std::chrono::milliseconds{100});
            } catch (const std::runtime_error&) {
                continue;  // nothing yet: look again
            }
        }
        return std::nullopt;
    }

    LoopbackServer commands_;
    LoopbackServer frames_;
    LoopbackServer log_;
    mutable std::mutex mutex_;
    std::vector<Call> calls_;
    std::map<std::string, Answer> answers_;
    std::deque<int> states_;
    int state_after_ = 0;
    std::array<double, 6> joints_{0.0, -90.0, 90.0, -90.0, -90.0, 0.0};
    std::vector<std::string> log_lines_;
    std::vector<std::string> log_requests_;
    std::atomic<bool> stop_{false};
    std::jthread commands_thread_;
    std::jthread frames_thread_;
    std::jthread log_thread_;
};

}  // namespace ec::test
