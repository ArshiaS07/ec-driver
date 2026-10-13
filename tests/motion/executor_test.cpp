// The executor with a scripted activity: starting, refusing, the always-run
// close, latching and acknowledging faults, stop and cancel, and the close
// time limit. No I/O: every cycle is built by the test.

#include "ec/motion/executor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ec/command/param.hpp"

namespace {

using namespace std::chrono_literals;
using ec::motion::AnyActivity;
using ec::motion::Condition;
using ec::motion::Conditions;
using ec::motion::Cycle;
using ec::motion::End;
using ec::motion::Error;
using ec::motion::Executor;
using ec::motion::Mode;
using ec::motion::Needs;
using ec::motion::Outbox;
using ec::motion::Phase;

// What the test sees of its activity.
struct Log {
    int steps = 0;
    std::vector<End> closes;
};

// An activity whose step results and close duration the test chooses.
struct Scripted {
    std::shared_ptr<Log> log = std::make_shared<Log>();
    Needs needed{};
    std::vector<std::expected<Phase, Error>> results{Phase::Running};  // per step; the last one repeats
    int cycles_to_close = 1;                                           // close() returns true on this call; 0: never

    [[nodiscard]] Needs needs() const { return needed; }
    [[nodiscard]] std::string_view name() const { return "scripted"; }

    std::expected<Phase, Error> step(const Cycle&, Outbox& out) {
        out.send(ec::command::GetRobotState{});
        const auto i = static_cast<std::size_t>(log->steps++);
        return results[std::min(i, results.size() - 1)];
    }
    bool close(const Cycle&, Outbox&, End why) {
        log->closes.push_back(why);
        return cycles_to_close != 0 && static_cast<int>(log->closes.size()) >= cycles_to_close;
    }
};

Cycle at(std::chrono::milliseconds t, Conditions held = ec::motion::kCanStartMotion) {
    Cycle c;
    c.now = ec::motion::Clock::time_point{} + t;
    c.conditions = held;
    return c;
}

// Runs one cycle and returns the methods written to the outbox.
std::vector<std::string> run(Executor& ex, const Cycle& c) {
    Outbox out;
    ex.cycle(c, out);
    std::vector<std::string> methods;
    for (const auto& o : out.items()) methods.emplace_back(o.request.method);
    return methods;
}

TEST(Executor, RunsAnActivityToTheEndAndClosesIt) {
    Executor ex;
    Scripted a;
    a.results = {Phase::Running, Phase::Done};
    const auto log = a.log;
    ASSERT_TRUE(ex.start(7, std::move(a), at(0ms)));
    EXPECT_EQ(ex.mode(), Mode::Running);
    EXPECT_EQ(ex.current(), 7u);

    EXPECT_EQ(run(ex, at(0ms)), std::vector<std::string>{"getRobotState"});
    EXPECT_EQ(ex.mode(), Mode::Running);
    run(ex, at(8ms));  // Done, then closed in the same cycle
    EXPECT_EQ(ex.mode(), Mode::Idle);
    EXPECT_EQ(log->steps, 2);
    EXPECT_EQ(log->closes, std::vector<End>{End::Done});

    const auto finished = ex.take_finished();
    ASSERT_EQ(finished.size(), 1u);
    EXPECT_EQ(finished[0].ticket, 7u);
    EXPECT_TRUE(finished[0].result);
    EXPECT_FALSE(ex.current());
    EXPECT_TRUE(ex.take_finished().empty());
}

TEST(Executor, RefusesAStartWithoutItsConditionsAndLatchesNothing) {
    Executor ex;
    Scripted a;
    a.needed.to_start = {Condition::Remote, Condition::ServoOn};
    const auto r = ex.start(1, std::move(a), at(0ms, {Condition::Remote}));
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Refused);
    EXPECT_EQ(r.error().missing, Conditions{Condition::ServoOn});
    EXPECT_NE(r.error().message.find("servo on"), std::string::npos);
    EXPECT_EQ(ex.mode(), Mode::Idle);
    EXPECT_FALSE(ex.fault());
}

TEST(Executor, RefusesASecondActivityWhileOneIsOpen) {
    Executor ex;
    ASSERT_TRUE(ex.start(1, Scripted{}, at(0ms)));
    const auto r = ex.start(2, Scripted{}, at(0ms));
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Refused);
    EXPECT_EQ(ex.current(), 1u);
}

TEST(Executor, ALostConditionClosesTheActivityAndLatchesAFault) {
    Executor ex;
    Scripted a;
    a.needed.while_running = {Condition::ServoOn};
    const auto log = a.log;
    ASSERT_TRUE(ex.start(1, std::move(a), at(0ms)));
    run(ex, at(0ms));
    run(ex, at(8ms, {Condition::Fresh}));  // the servos went off
    EXPECT_EQ(log->closes, std::vector<End>{End::Faulted});
    EXPECT_EQ(log->steps, 1);  // not stepped once the condition was gone
    ASSERT_EQ(ex.mode(), Mode::Fault);
    EXPECT_EQ(ex.fault()->kind, Error::Kind::ConditionLost);
    EXPECT_EQ(ex.fault()->missing, Conditions{Condition::ServoOn});

    const auto finished = ex.take_finished();
    ASSERT_EQ(finished.size(), 1u);
    EXPECT_EQ(finished[0].result.error().kind, Error::Kind::ConditionLost);

    const auto refused = ex.start(2, Scripted{}, at(16ms));
    ASSERT_FALSE(refused);
    EXPECT_NE(refused.error().message.find("acknowledge"), std::string::npos);
    ASSERT_TRUE(ex.acknowledge());
    EXPECT_EQ(ex.mode(), Mode::Idle);
    EXPECT_FALSE(ex.fault());
    EXPECT_TRUE(ex.start(2, Scripted{}, at(16ms)));
}

TEST(Executor, AnErrorFromStepLatchesOnlyIfItsKindDoes) {
    {
        Executor ex;
        Scripted a;
        a.results = {std::unexpected(Error{Error::Kind::Tracking, "3 deg off", {}})};
        const auto log = a.log;
        ASSERT_TRUE(ex.start(1, std::move(a), at(0ms)));
        run(ex, at(0ms));
        EXPECT_EQ(log->closes, std::vector<End>{End::Faulted});
        EXPECT_EQ(ex.mode(), Mode::Fault);
        EXPECT_EQ(ex.fault()->message, "3 deg off");
    }
    {
        Executor ex;
        Scripted a;
        a.results = {std::unexpected(Error{Error::Kind::Invalid, "speed 2", {}})};
        const auto log = a.log;
        ASSERT_TRUE(ex.start(1, std::move(a), at(0ms)));
        run(ex, at(0ms));
        EXPECT_EQ(log->closes, std::vector<End>{End::Cancelled});
        EXPECT_EQ(ex.mode(), Mode::Idle);
        EXPECT_EQ(ex.take_finished()[0].result.error().kind, Error::Kind::Invalid);
    }
}

TEST(Executor, CancelClosesGracefullyAndOnlyForItsTicket) {
    Executor ex;
    Scripted a;
    a.cycles_to_close = 2;
    const auto log = a.log;
    ASSERT_TRUE(ex.start(5, std::move(a), at(0ms)));
    ex.cancel(4);  // another ticket: ignored
    run(ex, at(0ms));
    EXPECT_TRUE(log->closes.empty());
    ex.cancel(5);
    run(ex, at(8ms));
    EXPECT_EQ(ex.mode(), Mode::Closing);
    run(ex, at(16ms));
    EXPECT_EQ(log->closes, (std::vector<End>{End::Cancelled, End::Cancelled}));
    EXPECT_EQ(ex.mode(), Mode::Idle);
    EXPECT_EQ(ex.take_finished()[0].result.error().kind, Error::Kind::Cancelled);
}

TEST(Executor, StopClosesUrgentlyOrSendsAPlainStopWhenIdle) {
    Executor ex;
    EXPECT_TRUE(run(ex, at(0ms)).empty());
    ex.stop();
    EXPECT_EQ(run(ex, at(8ms)), std::vector<std::string>{"stop"});
    EXPECT_TRUE(run(ex, at(16ms)).empty());

    Scripted a;
    const auto log = a.log;
    ASSERT_TRUE(ex.start(1, std::move(a), at(16ms)));
    run(ex, at(16ms));
    ex.stop();
    run(ex, at(24ms));
    EXPECT_EQ(log->closes, std::vector<End>{End::Stopped});
    EXPECT_EQ(ex.mode(), Mode::Idle);  // a stop is not a fault
    EXPECT_EQ(ex.take_finished()[0].result.error().kind, Error::Kind::Stopped);
}

TEST(Executor, StopDuringAGracefulCloseMakesItUrgent) {
    Executor ex;
    Scripted a;
    a.cycles_to_close = 3;
    const auto log = a.log;
    ASSERT_TRUE(ex.start(1, std::move(a), at(0ms)));
    ex.cancel(1);
    run(ex, at(0ms));
    ex.stop();
    run(ex, at(8ms));
    run(ex, at(16ms));
    EXPECT_EQ(log->closes, (std::vector<End>{End::Cancelled, End::Stopped, End::Stopped}));
    EXPECT_EQ(ex.take_finished()[0].result.error().kind, Error::Kind::Stopped);
}

TEST(Executor, ACloseThatNeverFinishesIsDroppedWithAFault) {
    Executor ex(Executor::Options{.close_timeout = 100ms});
    Scripted a;
    a.cycles_to_close = 0;  // never closes
    ASSERT_TRUE(ex.start(1, std::move(a), at(0ms)));
    ex.cancel(1);
    run(ex, at(0ms));
    EXPECT_TRUE(run(ex, at(100ms)).empty());
    EXPECT_EQ(run(ex, at(101ms)), std::vector<std::string>{"stop"});  // the last resort
    ASSERT_EQ(ex.mode(), Mode::Fault);
    EXPECT_EQ(ex.fault()->kind, Error::Kind::Timeout);
    EXPECT_FALSE(ex.current());
}

TEST(Executor, AFailureFromTheControlThreadLatches) {
    Executor ex;
    ex.fail(Error{Error::Kind::Disconnected, "8055 closed", {}});
    run(ex, at(0ms));
    ASSERT_EQ(ex.mode(), Mode::Fault);
    EXPECT_EQ(ex.fault()->kind, Error::Kind::Disconnected);
    ASSERT_TRUE(ex.acknowledge());

    Scripted a;
    const auto log = a.log;
    ASSERT_TRUE(ex.start(1, std::move(a), at(8ms)));
    ex.fail(Error{Error::Kind::Overrun, "3 late cycles", {}});
    run(ex, at(8ms));
    EXPECT_EQ(log->closes, std::vector<End>{End::Faulted});
    EXPECT_EQ(ex.fault()->kind, Error::Kind::Overrun);
}

TEST(Executor, AcknowledgeIsRefusedWhileAnActivityIsOpen) {
    Executor ex;
    ASSERT_TRUE(ex.start(1, Scripted{}, at(0ms)));
    EXPECT_FALSE(ex.acknowledge());
    EXPECT_TRUE(Executor{}.acknowledge());  // nothing latched: nothing to do
}

}  // namespace
