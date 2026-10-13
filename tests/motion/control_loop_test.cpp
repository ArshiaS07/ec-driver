// The control loop with a real command connection (a loopback server that
// answers on its own thread) and a state fed every 8 ms: jobs end with their
// activity's result, stop and destruction stop the arm, a lost connection
// latches a fault, calls are gated by kind, and a slow cycle is an overrun.

#include "ec/motion/control_loop.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "support/frame_builder.hpp"
#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using ec::command::Json;
using ec::motion::Condition;
using ec::motion::ControlLoop;
using ec::motion::ControlOptions;
using ec::motion::Cycle;
using ec::motion::End;
using ec::motion::Error;
using ec::motion::Mode;
using ec::motion::Needs;
using ec::motion::Outbox;
using ec::motion::Phase;

// Answers every request line on the server side: "true", or what `answers`
// holds for its method. Records the methods it saw.
class Responder {
public:
    explicit Responder(ec::test::LoopbackServer& server) : server_(server) {}
    ~Responder() { stop(); }
    Responder(const Responder&) = delete;
    Responder& operator=(const Responder&) = delete;

    void start() {
        thread_ = std::jthread([this] { serve(); });
    }
    // Ends the connection, if it is still served, and waits for the thread.
    void stop() {
        {
            const std::lock_guard lock(mutex_);
            if (peer_) peer_->shutdown();  // wakes serve() if it is waiting for a request
        }
        if (thread_.joinable()) thread_.join();
    }
    void answer(const std::string& method, std::string result_json) {
        const std::lock_guard lock(mutex_);
        answers_[method] = std::move(result_json);
    }
    void drop_after(int requests) { drop_after_ = requests; }
    std::vector<std::string> seen() const {
        const std::lock_guard lock(mutex_);
        return seen_;
    }

private:
    void serve() {
        auto accepted = server_.accept(5s);
        {
            const std::lock_guard lock(mutex_);
            peer_.emplace(std::move(accepted));
        }
        ec::test::Peer& peer = *peer_;  // reset only by this thread; stop() only shuts it down
        for (int n = 1;; ++n) {
            const std::string line = peer.receive_line();
            if (line.empty()) break;  // the client closed, or stop()
            const Json request = Json::parse(line);
            std::string result = "true";
            {
                const std::lock_guard lock(mutex_);
                seen_.push_back(request["method"].get<std::string>());
                if (const auto it = answers_.find(seen_.back()); it != answers_.end()) result = it->second;
            }
            if (n == drop_after_) break;  // closes the connection without answering
            peer.send(Json{{"jsonrpc", "2.0"}, {"result", result}, {"id", request["id"]}}.dump() + "\n");
        }
        const std::lock_guard lock(mutex_);
        peer_.reset();  // closes the connection
    }

    ec::test::LoopbackServer& server_;
    std::optional<ec::test::Peer> peer_;  // the connection served; guarded by mutex_ except in serve()
    mutable std::mutex mutex_;
    std::map<std::string, std::string> answers_;
    std::vector<std::string> seen_;
    std::atomic<int> drop_after_{0};
    std::jthread thread_;
};

// Publishes a frame every 8 ms, built from `fields`, which the test may change.
class Feeder {
public:
    Feeder() : latest(std::make_shared<ec::Latest<ec::feedback::Sample>>()) {
        thread_ = std::jthread([this](const std::stop_token& stop) {
            while (!stop.stop_requested()) {
                ec::test::FrameFields f;
                {
                    const std::lock_guard lock(mutex_);
                    f = fields_;
                }
                latest->publish(ec::feedback::Sample{std::chrono::steady_clock::now(), ec::test::make_frame(f)});
                std::this_thread::sleep_for(8ms);
            }
        });
    }
    void set(const std::function<void(ec::test::FrameFields&)>& change) {
        const std::lock_guard lock(mutex_);
        change(fields_);
    }
    std::shared_ptr<ec::Latest<ec::feedback::Sample>> latest;

private:
    std::mutex mutex_;
    ec::test::FrameFields fields_;
    std::jthread thread_;
};

// Sends getRobotState every cycle until it has had `cycles` replies; or forever.
struct Pinger {
    int cycles = 3;  // 0: never done
    std::shared_ptr<std::atomic<int>> closes = std::make_shared<std::atomic<int>>(0);
    std::chrono::milliseconds sleep_in_step{0};
    int replies = 0;

    [[nodiscard]] Needs needs() const { return {{Condition::Fresh, Condition::ServoOn}, {Condition::Fresh}}; }
    std::expected<Phase, Error> step(const Cycle& c, Outbox& out) {
        if (sleep_in_step > 0ms) std::this_thread::sleep_for(sleep_in_step);
        for (const auto& r : c.replies) {
            if (!r.result) return std::unexpected(ec::motion::command_failed(r.method, r.result.error()));
            ++replies;
        }
        if (cycles != 0 && replies >= cycles) return Phase::Done;
        out.send(ec::command::GetRobotState{});
        return Phase::Running;
    }
    bool close(const Cycle&, Outbox&, End) {
        ++*closes;
        return true;
    }
};

ec::command::Client connect_to(const ec::test::LoopbackServer& server) {
    auto client = ec::command::Client::connect("127.0.0.1", {.port = server.port()});
    if (!client) throw std::runtime_error("connect failed: " + client.error().message);
    return std::move(*client);
}

struct Rig {
    ec::test::LoopbackServer server;
    Responder responder{server};
    Feeder feeder;
    std::unique_ptr<ControlLoop> loop;

    // The tests check what the loop does, not its timing: a state up to 500 ms
    // old still counts as fresh, so a busy machine cannot fail them.
    explicit Rig(ControlOptions options = {}) {
        options.thresholds.fresh_within = 500ms;
        auto client = connect_to(server);
        responder.start();
        loop = std::make_unique<ControlLoop>(
            std::move(client), [this] { return ec::command::Client::connect("127.0.0.1", {.port = server.port()}); },
            feeder.latest, options);
    }
};

TEST(ControlLoop, RunsAnActivityAndReportsItsResult) {
    Rig rig;
    const auto job = rig.loop->run(Pinger{});
    const auto result = job.wait_for(5s);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(*result);
    EXPECT_TRUE(job.done());
    const auto seen = rig.responder.seen();
    EXPECT_EQ(std::count(seen.begin(), seen.end(), "getRobotState"), 3);
    const auto status = rig.loop->status();
    EXPECT_EQ(status.mode, Mode::Idle);
    EXPECT_TRUE(status.connected);
    EXPECT_TRUE(status.conditions.has(Condition::Fresh));
    EXPECT_GT(status.cycles, 0u);
}

TEST(ControlLoop, RefusesAStartWithoutItsConditions) {
    Rig rig;
    rig.feeder.set([](auto& f) { f.servo = false; });
    std::this_thread::sleep_for(30ms);
    const auto result = rig.loop->run(Pinger{}).wait();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::Refused);
    EXPECT_EQ(result.error().missing, ec::motion::Conditions{Condition::ServoOn});
    EXPECT_EQ(rig.loop->status().mode, Mode::Idle);  // a refusal latches nothing
}

TEST(ControlLoop, StopEndsTheActivityAndCloseRuns) {
    Rig rig;
    Pinger forever{.cycles = 0};
    const auto closes = forever.closes;
    const auto job = rig.loop->run(forever);
    std::this_thread::sleep_for(50ms);
    EXPECT_FALSE(job.done());
    rig.loop->stop();
    const auto result = job.wait();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::Stopped);
    EXPECT_EQ(*closes, 1);
}

TEST(ControlLoop, CancelEndsOnlyItsJob) {
    Rig rig;
    const auto job = rig.loop->run(Pinger{.cycles = 0});
    std::this_thread::sleep_for(30ms);
    job.cancel();
    EXPECT_EQ(job.wait().error().kind, Error::Kind::Cancelled);
}

TEST(ControlLoop, ALostConnectionLatchesAFaultUntilAcknowledged) {
    Rig rig(ControlOptions{.reconnect_every = 100ms});
    rig.responder.drop_after(2);
    const auto result = rig.loop->run(Pinger{.cycles = 0}).wait();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::Disconnected);
    EXPECT_EQ(rig.loop->status().mode, Mode::Fault);

    rig.responder.stop();
    rig.responder.drop_after(0);
    rig.responder.start();  // the loop reconnects to the same server
    for (int i = 0; i < 200 && !rig.loop->status().connected; ++i) std::this_thread::sleep_for(10ms);
    ASSERT_TRUE(rig.loop->status().connected);
    EXPECT_EQ(rig.loop->run(Pinger{}).wait().error().kind, Error::Kind::Refused);  // still latched
    ASSERT_TRUE(rig.loop->acknowledge());
    EXPECT_TRUE(rig.loop->run(Pinger{}).wait());
}

TEST(ControlLoop, CallsQueriesAnyTimeAndSettingsOnlyWhenIdle) {
    Rig rig;
    rig.responder.answer("getRobotMode", "2");
    const auto mode = rig.loop->call(ec::command::to_request(ec::command::GetRobotMode{}), ec::command::Kind::Query);
    ASSERT_TRUE(mode) << describe(mode.error());
    EXPECT_EQ(*mode, Json(2));

    const auto motion = rig.loop->call(ec::command::to_request(ec::command::Stop{}), ec::command::Kind::Motion);
    ASSERT_FALSE(motion);
    EXPECT_EQ(motion.error().kind, Error::Kind::Refused);

    const auto job = rig.loop->run(Pinger{.cycles = 0});
    std::this_thread::sleep_for(30ms);
    const auto busy =
        rig.loop->call(ec::command::to_request(ec::command::SetSpeed{.value_pct = 50}), ec::command::Kind::Setting);
    ASSERT_FALSE(busy);
    EXPECT_EQ(busy.error().kind, Error::Kind::Refused);
    EXPECT_TRUE(rig.loop->call(ec::command::to_request(ec::command::GetRobotMode{}), ec::command::Kind::Query));
    job.cancel();
    EXPECT_EQ(job.wait().error().kind, Error::Kind::Cancelled);
    EXPECT_TRUE(
        rig.loop->call(ec::command::to_request(ec::command::SetSpeed{.value_pct = 50}), ec::command::Kind::Setting));
}

TEST(ControlLoop, DestructionStopsAnOpenActivity) {
    Rig rig;
    Pinger forever{.cycles = 0};
    const auto closes = forever.closes;
    const auto job = rig.loop->run(forever);
    std::this_thread::sleep_for(30ms);
    rig.loop.reset();
    ASSERT_TRUE(job.done());
    EXPECT_EQ(job.wait().error().kind, Error::Kind::Stopped);
    EXPECT_EQ(*closes, 1);
}

TEST(ControlLoop, ASlowCycleIsAnOverrun) {
    Rig rig;
    const auto result = rig.loop->run(Pinger{.cycles = 0, .sleep_in_step = 25ms}).wait();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::Overrun);
    EXPECT_GE(rig.loop->status().late_cycles, 3u);
}

// Frames published far faster than the cycle: some arrive between the cycle's
// start and its reading of the state. They are fresh, not from the future, so
// the activity, which needs fresh state all along, runs to its end.
TEST(ControlLoop, AFrameArrivingDuringTheCycleIsFresh) {
    ec::test::LoopbackServer server;
    Responder responder{server};
    auto client = connect_to(server);
    responder.start();
    auto latest = std::make_shared<ec::Latest<ec::feedback::Sample>>();
    const auto frame = ec::test::make_frame(ec::test::FrameFields{});
    std::jthread publisher([&latest, &frame](const std::stop_token& stop) {
        while (!stop.stop_requested()) {
            latest->publish(ec::feedback::Sample{std::chrono::steady_clock::now(), frame});
            std::this_thread::yield();  // as often as it can: a frame lands in almost every cycle's gap
        }
    });
    ControlLoop loop(std::move(client), {}, latest, ControlOptions{});  // the default 50 ms freshness
    const auto result = loop.run(Pinger{.cycles = 100}).wait();
    EXPECT_TRUE(result) << describe(result.error());
}

TEST(ControlLoop, CallsAreAnsweredBetweenCyclesNotAtThem) {
    Rig rig(ControlOptions{.period = std::chrono::milliseconds{50}});
    const auto started = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; ++i)
        ASSERT_TRUE(rig.loop->call(ec::command::to_request(ec::command::GetRobotMode{}), ec::command::Kind::Query));
    EXPECT_LT(std::chrono::steady_clock::now() - started, 250ms);  // a cycle each would take 500 ms
    const auto job = rig.loop->run(Pinger{.cycles = 0});           // gated as before while an activity runs
    std::this_thread::sleep_for(120ms);
    const auto busy =
        rig.loop->call(ec::command::to_request(ec::command::SetSpeed{.value_pct = 50}), ec::command::Kind::Setting);
    EXPECT_EQ(busy.error().kind, Error::Kind::Refused);
    job.cancel();
    EXPECT_EQ(job.wait().error().kind, Error::Kind::Cancelled);
}

// Never finishes closing.
struct Stubborn {
    [[nodiscard]] Needs needs() const { return {{Condition::Fresh}, {Condition::Fresh}}; }
    std::expected<Phase, Error> step(const Cycle&, Outbox&) { return Phase::Running; }
    bool close(const Cycle&, Outbox&, End) { return false; }
};

TEST(ControlLoop, ShutdownStopsTheArmWhenTheActivityWillNotClose) {
    ControlOptions options;
    options.shutdown_timeout = 100ms;
    options.executor.close_timeout = 10s;  // longer than the shutdown: the loop's own last resort counts
    Rig rig(options);
    const auto job = rig.loop->run(Stubborn{});
    std::this_thread::sleep_for(50ms);
    rig.loop.reset();
    const auto seen = rig.responder.seen();
    EXPECT_EQ(std::count(seen.begin(), seen.end(), "stop"), 1);
    ASSERT_TRUE(job.done());
    EXPECT_FALSE(job.wait());
}

TEST(ControlLoop, NextStatusWaitsForTheNextCycle) {
    Rig rig;
    const auto first = rig.loop->next_status(0, 1s);
    ASSERT_TRUE(first);
    const auto later = rig.loop->next_status(first->cycles, 1s);
    ASSERT_TRUE(later);
    EXPECT_GT(later->cycles, first->cycles);
}

}  // namespace
