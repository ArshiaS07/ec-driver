// The motion families (ADR 0018) against a scripted controller: each sends the
// manual's sequence, every call answered; one motion is open at a time; waits
// read getRobotState; Robot::stop() sends the open motion's stop, then stop.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "ec/motion/move.hpp"
#include "ec/motion/move_along.hpp"
#include "ec/motion/servo_stream.hpp"
#include "ec/motion/timestamped.hpp"
#include "ec/robot.hpp"
#include "support/mini_controller.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Error;
using ec::Joints;
using ec::test::MiniController;

constexpr int kStop = 0;
constexpr int kRunning = 3;

const Joints kHere{0.0, ec::deg_to_rad(-90.0), ec::deg_to_rad(90.0), ec::deg_to_rad(-90.0), ec::deg_to_rad(-90.0), 0.0};

ec::Robot connect(MiniController& controller) {
    auto robot = ec::Robot::connect(controller.options(/*monitor=*/false));
    if (!robot) throw std::runtime_error(describe(robot.error()));
    controller.forget();
    return std::move(*robot);
}

// A short joint trajectory from kHere, J1 out by 2 deg and back.
ec::motion::JointTrajectory there_and_back() {
    ec::motion::JointTrajectory t;
    for (int k = 0; k <= 10; ++k) {
        Joints q = kHere;
        q[0] = ec::deg_to_rad(0.2 * std::min(k, 10 - k));
        t.time.push_back(0.1 * k);
        t.position.push_back(q);
    }
    return t;
}

// ------------------------------------------------------------------------ Move

TEST(Move, SendsDegreesAndPercentAndWaitsUntilTheArmRestsAtTheTarget) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kRunning, kStop});
    auto move = ec::motion::Move::joints(robot, kHere, {.speed = 0.25});
    ASSERT_TRUE(move) << describe(move.error());
    const auto params = controller.last("moveByJoint");
    EXPECT_NEAR(params["targetPos"][1].get<double>(), -90.0, 1e-9);
    EXPECT_NEAR(params["speed"].get<double>(), 25.0, 1e-9);
    EXPECT_TRUE(move->wait());
    EXPECT_EQ(controller.count("getRobotState"), 3u);
    EXPECT_EQ(controller.count("get_joint_pos"), 1u);  // the arrival
}

TEST(Move, RestingElsewhereIsAnError) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kStop});
    Joints target = kHere;
    target[5] = ec::deg_to_rad(10.0);  // the scripted arm stays at kHere
    auto move = ec::motion::Move::joints(robot, target);
    ASSERT_TRUE(move);
    const auto r = move->wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_NE(r.error().message.find("10.00 deg from the target"), std::string::npos) << r.error().message;
}

TEST(Move, AnEmergencyStopIsAFault) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, 2});
    auto move = ec::motion::Move::joints(robot, kHere);
    ASSERT_TRUE(move);
    const auto r = move->wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Fault);
    EXPECT_EQ(r.error().message, "emergency stop");
}

TEST(Move, ImpossibleArgumentsAreInvalidAndSendNothing) {
    MiniController controller;
    auto robot = connect(controller);
    Joints far = kHere;
    far[0] = ec::deg_to_rad(400.0);
    EXPECT_EQ(ec::motion::Move::joints(robot, far).error().kind, Error::Kind::Invalid);
    EXPECT_EQ(ec::motion::Move::joints(robot, kHere, {.speed = 0.0}).error().kind, Error::Kind::Invalid);
    EXPECT_EQ(ec::motion::Move::joints(robot, kHere, {.speed = 0.5, .acceleration = 2.0}).error().kind,
              Error::Kind::Invalid);
    EXPECT_TRUE(controller.calls().empty());
}

TEST(Move, OneMotionAtATimeAndOnlyReadingCallsWhileItIsOpen) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kRunning, kStop});
    auto move = ec::motion::Move::joints(robot, kHere);
    ASSERT_TRUE(move);
    EXPECT_EQ(ec::motion::Move::joints(robot, kHere).error().kind, Error::Kind::Busy);
    EXPECT_TRUE(robot.call(ec::command::GetRobotMode{}));                                             // a query
    EXPECT_EQ(robot.call(ec::command::SetSpeed{.value_pct = 50.0}).error().kind, Error::Kind::Busy);  // a setting
    EXPECT_TRUE(move->wait());
    EXPECT_TRUE(robot.call(ec::command::SetSpeed{.value_pct = 50.0}));  // ended: no longer open
}

TEST(Move, StopFromAnotherThreadEndsTheWait) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({}, kRunning);  // runs until stopped
    auto move = ec::motion::Move::joints(robot, kHere);
    ASSERT_TRUE(move);
    std::jthread stopper([&robot] {
        std::this_thread::sleep_for(100ms);
        robot.stop();
    });
    const auto r = move->wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Stopped);
    EXPECT_EQ(controller.count("stop"), 2u);  // the move's own stop, then stop
}

// ----------------------------------------------------------------- Timestamped

TEST(Timestamped, UploadsAsTheManualThenStartsAndWaits) {
    MiniController controller;
    auto robot = connect(controller);
    const auto t = there_and_back();
    auto run = ec::motion::Timestamped::upload(robot, t);
    ASSERT_TRUE(run) << describe(run.error());
    std::vector<std::string> expected{"get_joint_pos", "start_push_pos"};
    expected.insert(expected.end(), t.size(), "push_pos");
    expected.insert(expected.end(), {"stop_push_pos", "check_trajectory"});
    EXPECT_EQ(controller.methods(), expected);
    const auto start = controller.last("start_push_pos");
    EXPECT_EQ(start["path_lenth"], 11);
    EXPECT_EQ(start["ret_flag"], 1);
    EXPECT_EQ(start["pos_type"], 0);
    const auto last = controller.last("push_pos");
    EXPECT_NEAR(last["timestamp"].get<double>(), 1.0, 1e-12);
    EXPECT_NEAR(last["pos"][1].get<double>(), -90.0, 1e-9);

    controller.states({kRunning, kRunning, kStop});
    ASSERT_TRUE(run->start(0.5));
    EXPECT_NEAR(controller.last("start_trajectory")["speed_percent"].get<double>(), 50.0, 1e-9);
    EXPECT_TRUE(run->wait());
}

TEST(Timestamped, RefusesToUploadAwayFromTheFirstPoint) {
    MiniController controller;
    auto robot = connect(controller);
    controller.joints_deg({5.0, -90.0, 90.0, -90.0, -90.0, 0.0});
    const auto run = ec::motion::Timestamped::upload(robot, there_and_back());
    ASSERT_FALSE(run);
    EXPECT_EQ(run.error().kind, Error::Kind::Invalid);
    EXPECT_NE(run.error().message.find("move there first"), std::string::npos);
    EXPECT_EQ(controller.count("start_push_pos"), 0u);
}

TEST(Timestamped, AnInvalidUploadIsReportedAndNotStarted) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("check_trajectory", "-3");
    const auto run = ec::motion::Timestamped::upload(robot, there_and_back());
    ASSERT_FALSE(run);
    EXPECT_EQ(run.error().kind, Error::Kind::Command);
    EXPECT_NE(run.error().message.find("(-3)"), std::string::npos) << run.error().message;
    EXPECT_EQ(controller.count("start_trajectory"), 0u);
    EXPECT_EQ(controller.count("flush_trajectory"), 0u);  // no clean-up sequence of ours
}

TEST(Timestamped, ARefusedPointNamesIt) {
    MiniController controller;
    auto robot = connect(controller);
    int pushes = 0;
    controller.answer("push_pos", [&pushes](const auto&) {
        return ++pushes == 4 ? std::optional<std::string>{"false"} : std::optional<std::string>{"true"};
    });
    const auto run = ec::motion::Timestamped::upload(robot, there_and_back());
    ASSERT_FALSE(run);
    EXPECT_NE(run.error().message.find("point 3: push_pos"), std::string::npos) << run.error().message;
    EXPECT_EQ(controller.count("push_pos"), 4u);
}

TEST(Timestamped, StopWhileItRunsSendsStopTrajectoryThenStop) {
    MiniController controller;
    auto robot = connect(controller);
    auto run = ec::motion::Timestamped::upload(robot, there_and_back());
    ASSERT_TRUE(run);
    controller.states({}, kRunning);
    ASSERT_TRUE(run->start());
    std::jthread stopper([&robot] {
        std::this_thread::sleep_for(100ms);
        robot.stop();
    });
    const auto r = run->wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Stopped);
    const auto m = controller.methods();
    const auto at = std::find(m.begin(), m.end(), "stop_trajectory");
    ASSERT_NE(at, m.end());
    EXPECT_NE(std::find(at, m.end(), "stop"), m.end());
}

// ----------------------------------------------------------------- ServoStream

TEST(ServoStream, OpensAsTheManualTakesPointsAndCloses) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("get_transparent_transmission_state", "1");  // left open
    {
        auto stream = ec::motion::ServoStream::open(robot, {.lookahead = 40ms, .period = 8ms, .smoothness = 0.2});
        ASSERT_TRUE(stream) << describe(stream.error());
        EXPECT_EQ(controller.methods(),
                  (std::vector<std::string>{"get_transparent_transmission_state", "tt_clear_servo_joint_buf",
                                            "transparent_transmission_init"}));
        const auto init = controller.last("transparent_transmission_init");
        EXPECT_EQ(init["lookahead"], 40);
        EXPECT_EQ(init["t"], 8);
        EXPECT_NEAR(init["smoothness"].get<double>(), 0.2, 1e-12);
        ASSERT_TRUE(stream->add_point(kHere));
        EXPECT_NEAR(controller.last("tt_put_servo_joint_to_buf")["targetPos"][2].get<double>(), 90.0, 1e-9);
        Joints far = kHere;
        far[2] = 7.0;
        EXPECT_EQ(stream->add_point(far).error().kind, Error::Kind::Invalid);
        ASSERT_TRUE(stream->close());
        EXPECT_EQ(controller.count("tt_clear_servo_joint_buf"), 2u);
        EXPECT_TRUE(ec::motion::Move::joints(robot, kHere));  // closed: another motion may open
    }
}

TEST(ServoStream, OptionsOutOfRangeAreInvalid) {
    MiniController controller;
    auto robot = connect(controller);
    EXPECT_EQ(ec::motion::ServoStream::open(robot, {.lookahead = 8ms, .period = 8ms}).error().kind,
              Error::Kind::Invalid);
    EXPECT_EQ(ec::motion::ServoStream::open(robot, {.period = 1ms}).error().kind, Error::Kind::Invalid);
    EXPECT_TRUE(controller.calls().empty());
}

// ------------------------------------------------------------------ move_along

TEST(MoveAlong, PlansMovesToTheStartUploadsAndRuns) {
    MiniController controller;
    auto robot = connect(controller);
    // The scripted inverse kinematics answers the reference: the arm stays at kHere.
    const auto here = ec::from_mm_rad({400.0, -50.0, 300.0, 3.0, 0.1, -0.2});
    Eigen::Isometry3d nearby = here;
    nearby.translation().x() += 0.001;
    const ec::PoseTrajectory poses{{0s, here}, {1s, nearby}};
    controller.states({kRunning, kStop, kRunning, kStop});
    const auto r = ec::motion::move_along(robot, poses);
    ASSERT_TRUE(r) << describe(r.error());
    const auto m = controller.methods();
    const auto order = [&m](const std::string& a, const std::string& b) {
        return std::find(m.begin(), m.end(), a) < std::find(m.begin(), m.end(), b);
    };
    EXPECT_TRUE(order("inverseKinematic", "moveByJoint"));
    EXPECT_TRUE(order("moveByJoint", "start_push_pos"));
    EXPECT_TRUE(order("check_trajectory", "start_trajectory"));
}

}  // namespace
