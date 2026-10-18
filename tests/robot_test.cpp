// ec::Robot against a scripted controller on 127.0.0.1 (tests/support/
// mini_controller.hpp): connecting, the monitoring stream's newest frame,
// typed calls, the controller's kinematics in SI, bring-up, and stop.

#include "ec/robot.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ec/pose.hpp"
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
    o.first_feedback_within = 300ms;
    const auto robot = ec::Robot::connect(o);
    ASSERT_FALSE(robot);
    EXPECT_EQ(robot.error().kind, Error::Kind::Disconnected);
    EXPECT_NE(robot.error().message.find("monitor port"), std::string::npos);
}

TEST(Robot, WithoutTheMonitorThereIsNoFeedbackButCallsWork) {
    MiniController controller(/*frames=*/false);
    auto robot = ec::Robot::connect(controller.options(/*monitor=*/false));
    ASSERT_TRUE(robot) << describe(robot.error());
    EXPECT_FALSE(robot->feedback());
    const auto mode = robot->call(ec::sdk::GetRobotMode{});
    ASSERT_TRUE(mode) << describe(mode.error());
    EXPECT_EQ(*mode, ec::RobotMode::Remote);
}

TEST(Robot, FeedbackIsTheNewestFrameDecodedAndEveryFrameIsHandedOn) {
    MiniController controller;
    std::atomic<int> handed{0};
    auto o = controller.options();
    o.on_feedback = [&handed](const ec::Feedback& f) {
        if (f.seq > 0 && f.sample.raw.size() == ec::feedback::kFrameSize) ++handed;
    };
    auto robot = ec::Robot::connect(o);
    ASSERT_TRUE(robot) << describe(robot.error());
    const auto first = robot->feedback();
    ASSERT_TRUE(first);
    EXPECT_LT(first->age(), 1s);
    const auto next = robot->next_feedback(first->seq, 1s);
    ASSERT_TRUE(next);
    EXPECT_GT(next->seq, first->seq);
    EXPECT_GE(handed.load(), 2);
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

TEST(Robot, KinematicsAreTheControllersInSiUnits) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    const Eigen::Isometry3d T_base_tcp = ec::from_mm_rad({400.0, -50.0, 300.0, 3.0, 0.1, -0.2});
    const ec::Joints near{0.1, -1.5, 1.5, -1.5, -1.5, 0.0};
    const auto q = robot->ik(T_base_tcp, near);
    ASSERT_TRUE(q) << describe(q.error());
    const auto ik = controller.last("inverseKinematic");
    for (std::size_t i = 0; i < 6; ++i)  // mm and radians
        EXPECT_NEAR(ik["targetPose"][i].get<double>(), (std::array{400.0, -50.0, 300.0, 3.0, 0.1, -0.2})[i], 1e-9);
    EXPECT_NEAR(ik["referencePos"][1].get<double>(), -85.9437, 1e-4);                  // degrees
    for (std::size_t j = 0; j < q->size(); ++j) EXPECT_NEAR((*q)[j], near[j], 1e-12);  // back in radians

    const auto T = robot->fk(near);
    ASSERT_TRUE(T) << describe(T.error());
    EXPECT_TRUE(T->translation().isApprox(Eigen::Vector3d(0.4, -0.05, 0.3), 1e-12));  // metres
    const auto joints = robot->joints();
    ASSERT_TRUE(joints);
    EXPECT_NEAR((*joints)[1], ec::deg_to_rad(-90.0), 1e-12);
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
    robot->stop();
    EXPECT_EQ(controller.methods(), std::vector<std::string>{"stop"});
}

}  // namespace
