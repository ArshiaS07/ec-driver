// ec::Robot against a scripted controller on 127.0.0.1 (tests/support/
// mini_controller.hpp): connecting, the monitor's newest state, typed calls,
// bring-up, and stop.

#include "ec/robot.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ec/sdk/param.hpp"
#include "support/mini_controller.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Error;
using ec::test::MiniController;

TEST(Robot, ConnectFailsWithoutTheCommandPort) {
    std::uint16_t closed = 0;
    {
        const ec::test::LoopbackServer gone;
        closed = gone.port();
    }
    const auto robot = ec::Robot::connect({.host = "127.0.0.1", .command_port = closed, .monitor = false});
    ASSERT_FALSE(robot);
    EXPECT_EQ(robot.error().kind, Error::Kind::Disconnected);
    EXPECT_NE(robot.error().message.find("command port"), std::string::npos);
}

TEST(Robot, ConnectFailsWithoutFramesWhenItMonitors) {
    MiniController controller(/*frames=*/false);
    auto o = controller.options();
    o.first_state_within = 300ms;
    const auto robot = ec::Robot::connect(o);
    ASSERT_FALSE(robot);
    EXPECT_EQ(robot.error().kind, Error::Kind::Disconnected);
    EXPECT_NE(robot.error().message.find("monitor port"), std::string::npos);
}

TEST(Robot, WithoutTheMonitorThereIsNoStateButCallsWork) {
    MiniController controller(/*frames=*/false);
    auto robot = ec::Robot::connect(controller.options(/*monitor=*/false));
    ASSERT_TRUE(robot) << describe(robot.error());
    EXPECT_FALSE(robot->monitor().latest());
    EXPECT_FALSE(robot->monitor().status().receiver.connected);
    const auto mode = robot->call(ec::sdk::GetRobotMode{});
    ASSERT_TRUE(mode) << describe(mode.error());
    EXPECT_EQ(*mode, ec::RobotMode::Remote);
}

TEST(Robot, TheMonitorHoldsTheNewestStateAndHandsOnEveryFrame) {
    MiniController controller;
    std::atomic<int> samples{0};
    std::atomic<int> states{0};
    auto o = controller.options();
    o.on_sample = [&samples](const ec::feedback::Sample&) { ++samples; };
    o.on_state = [&states](const ec::State& s) {
        if (s.seq > 0) ++states;
    };
    auto robot = ec::Robot::connect(o);
    ASSERT_TRUE(robot) << describe(robot.error());
    const auto first = robot->monitor().latest();
    ASSERT_TRUE(first);
    EXPECT_LT(first->age(), 1s);
    EXPECT_TRUE(first->drives.servo_on);  // the scripted frames: remote, servos on, synchronised, stop
    const auto next = robot->monitor().next(first->seq, 1s);
    ASSERT_TRUE(next);
    EXPECT_GT(next->seq, first->seq);
    EXPECT_GE(states.load(), 2);
    EXPECT_GE(samples.load(), states.load());

    ec::Robot moved = std::move(*robot);  // the monitor stays where it was
    EXPECT_TRUE(moved.monitor().next(next->seq, 1s));
}

TEST(Robot, CallsFromSeveralThreadsAreAnsweredOneByOne) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    std::atomic<int> answered{0};
    {
        std::vector<std::jthread> threads;
        threads.reserve(4);
        for (int t = 0; t < 4; ++t)
            threads.emplace_back([&] {
                for (int i = 0; i < 25; ++i)
                    if (robot->call(ec::sdk::GetRobotMode{})) ++answered;
            });
    }
    EXPECT_EQ(answered.load(), 100);
    EXPECT_EQ(controller.count("getRobotMode"), 100u);
}

TEST(Robot, BringUpSendsOnlyWhatTheStateNeeds) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    controller.forget();
    ASSERT_TRUE(robot->bring_up());
    EXPECT_EQ(controller.methods(), (std::vector<std::string>{"getRobotState", "getMotorStatus", "getServoStatus"}));

    controller.forget();
    controller.states({4});  // an alarm
    controller.answer("getMotorStatus", "false");
    int asked = 0;  // the servos are off until switched on, and report on at the second question after
    controller.answer("getServoStatus",
                      [&asked](const auto&) { return std::optional<std::string>(++asked >= 3 ? "true" : "false"); });
    ASSERT_TRUE(robot->bring_up());
    EXPECT_EQ(controller.methods(),
              (std::vector<std::string>{"getRobotState", "clearAlarm", "getMotorStatus", "syncMotorStatus",
                                        "getServoStatus", "set_servo_status", "getServoStatus", "getServoStatus"}));
    EXPECT_EQ(controller.last("set_servo_status")["status"], 1);

    controller.answer("getServoStatus", "false");
    controller.answer("set_servo_status", [](const auto&) { return std::optional<std::string>{}; });
    const auto failed = robot->bring_up();
    ASSERT_FALSE(failed);
    EXPECT_EQ(failed.error().kind, Error::Kind::Command);
    EXPECT_NE(failed.error().message.find("set_servo_status"), std::string::npos);
}

TEST(Robot, StopWithNothingOpenSendsStop) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    controller.forget();
    robot->motion().stop();
    EXPECT_EQ(controller.methods(), std::vector<std::string>{"stop"});
}

}  // namespace
