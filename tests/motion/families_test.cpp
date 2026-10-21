// The motion families against a scripted controller: each sends its
// SDK calls as the manual, in the controller's units, every call answered; one
// motion is open at a time; waits read getRobotState; Robot::stop() sends the
// open motion's stop, then stop.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ec/motion/move.hpp"
#include "ec/motion/servo_stream.hpp"
#include "ec/motion/timestamped.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/sdk/param.hpp"
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

// A short joint trajectory from kHere, J1 out by 1 deg and back over a second.
ec::JointTrajectory there_and_back() {
    ec::JointTrajectory t;
    for (int k = 0; k <= 10; ++k) {
        Joints q = kHere;
        q[0] = ec::deg_to_rad(0.2 * std::min(k, 10 - k));
        t.push_back({k * 100ms, q});
    }
    return t;
}

// ------------------------------------------------------------------------ Move

TEST(Move, SendsDegreesAndPercentAndWaitsAsTheManualsWaitStop) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kRunning, kStop});
    ec::motion::Move move(robot);
    EXPECT_TRUE(controller.calls().empty());  // constructed: nothing sent
    ASSERT_TRUE(move.joint(kHere, {.speed = 0.25, .acceleration = 0.5, .deceleration = 0.3}));
    const auto params = controller.last("moveByJoint");
    EXPECT_NEAR(params["targetPos"][1].get<double>(), -90.0, 1e-9);
    EXPECT_NEAR(params["speed"].get<double>(), 25.0, 1e-9);
    EXPECT_EQ(params["acc"], 50);
    EXPECT_EQ(params["dec"], 30);
    EXPECT_TRUE(move.wait());
    EXPECT_EQ(controller.methods(),
              (std::vector<std::string>{"moveByJoint", "getRobotState", "getRobotState", "getRobotState"}));
}

TEST(Move, LineArcAndRotateSendTheControllersUnits) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::Move move(robot);
    Joints there = kHere;
    there[0] = ec::deg_to_rad(10.0);

    controller.states({kRunning, kStop});
    ASSERT_TRUE(
        move.line(there, {.speed = 0.2,
                          .until = ec::sdk::StopCondition{
                              .source = ec::sdk::StopCondition::Source::DigitalInput, .address = 7, .value = true}}));
    auto p = controller.last("moveByLine");
    EXPECT_NEAR(p["targetPos"][0].get<double>(), 10.0, 1e-9);
    EXPECT_EQ(p["speed_type"], 0);
    EXPECT_NEAR(p["speed"].get<double>(), 200.0, 1e-9);  // mm/s
    EXPECT_EQ(p["cond_type"], 0);
    EXPECT_EQ(p["cond_num"], 7);
    EXPECT_EQ(p["cond_value"], 1);
    ASSERT_TRUE(move.wait());

    controller.states({kRunning, kStop});
    ASSERT_TRUE(move.arc(there, kHere, {.speed_type = ec::sdk::SpeedType::Rotational, .speed = ec::deg_to_rad(30.0)}));
    p = controller.last("moveByArc");
    EXPECT_NEAR(p["midPos"][0].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(p["targetPos"][0].get<double>(), 0.0, 1e-9);
    EXPECT_EQ(p["speed_type"], 1);
    EXPECT_NEAR(p["speed"].get<double>(), 30.0, 1e-9);  // deg/s
    ASSERT_TRUE(move.wait());

    controller.states({kRunning, kStop});
    ASSERT_TRUE(move.rotate(
        there,
        {.speed_type = ec::sdk::SpeedType::AbsoluteRotational, .speed = ec::deg_to_rad(20.0), .acceleration = 0.4}));
    p = controller.last("moveByRotate");
    EXPECT_EQ(p["speed_type"], 3);
    EXPECT_NEAR(p["speed"].get<double>(), 20.0, 1e-9);
    EXPECT_EQ(p["acc"], 40);
    EXPECT_FALSE(p.contains("dec"));  // the controller's default
    ASSERT_TRUE(move.wait());
}

TEST(Move, AnEmergencyStopIsAFault) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, 2});
    ec::motion::Move move(robot);
    ASSERT_TRUE(move.joint(kHere));
    const auto r = move.wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Fault);
    EXPECT_EQ(r.error().message, "emergency stop");
}

TEST(Move, ArgumentsOutsideTheManualsRangesAreInvalidAndSendNothing) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::Move move(robot);
    Joints far = kHere;
    far[0] = ec::deg_to_rad(400.0);
    const auto kind = [](const std::expected<void, Error>& r) {
        return r ? std::optional<Error::Kind>{} : r.error().kind;
    };
    EXPECT_EQ(kind(move.joint(far)), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.joint(kHere, {.speed = 0.0})), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.joint(kHere, {.speed = 0.5, .acceleration = 2.0})), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.line(kHere, {.speed = 5.0})), Error::Kind::Invalid);  // above 3000 mm/s
    EXPECT_EQ(kind(move.rotate(kHere, {.speed_type = ec::sdk::SpeedType::Rotational, .speed = ec::deg_to_rad(400.0)})),
              Error::Kind::Invalid);
    EXPECT_EQ(kind(move.arc(far, kHere)), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.wait()), Error::Kind::Invalid);  // no move to wait for
    EXPECT_TRUE(controller.calls().empty());
}

TEST(Move, OneMotionAtATimeAndOnlyReadingCallsWhileItIsOpen) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kRunning, kStop});
    ec::motion::Move move(robot);
    ASSERT_TRUE(move.joint(kHere));
    EXPECT_EQ(move.joint(kHere).error().kind, Error::Kind::Busy);  // its own move, until a wait has seen it end
    ec::motion::Move other(robot);
    EXPECT_EQ(other.joint(kHere).error().kind, Error::Kind::Busy);
    EXPECT_TRUE(robot.call(ec::sdk::GetRobotMode{}));                                             // a query
    EXPECT_EQ(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}).error().kind, Error::Kind::Busy);  // a setting
    EXPECT_TRUE(move.wait());
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));  // ended: no longer open
    EXPECT_EQ(controller.count("moveByJoint"), 1u);
}

TEST(Move, ARefusedMoveLeavesNothingOpen) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("moveByJoint", "false");
    ec::motion::Move move(robot);
    const auto r = move.joint(kHere);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_TRUE(r.error().message.starts_with("moveByJoint: controller returned false")) << r.error().message;
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));
}

TEST(Move, StopFromAnotherThreadEndsTheWait) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({}, kRunning);  // runs until stopped
    ec::motion::Move move(robot);
    ASSERT_TRUE(move.joint(kHere));
    std::jthread stopper([&robot] {
        std::this_thread::sleep_for(100ms);
        robot.stop();
    });
    const auto r = move.wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Stopped);
    EXPECT_EQ(controller.count("stop"), 2u);  // the move's own stop, then stop
}

// ----------------------------------------------------------------- Timestamped

TEST(Timestamped, UploadsAsTheManualThenStartsAndWaits) {
    MiniController controller;
    auto robot = connect(controller);
    const auto t = there_and_back();
    ec::motion::Timestamped run(robot, t);
    EXPECT_TRUE(controller.calls().empty());  // constructed: nothing sent
    ASSERT_TRUE(run.upload());
    std::vector<std::string> expected{"start_push_pos"};
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
    ASSERT_TRUE(run.start(0.5));
    EXPECT_NEAR(controller.last("start_trajectory")["speed_percent"].get<double>(), 50.0, 1e-9);
    EXPECT_TRUE(run.wait());
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));  // ended: no longer open
}

TEST(Timestamped, ArgumentsOutsideTheManualsRangesAreInvalidAndSendNothing) {
    MiniController controller;
    auto robot = connect(controller);
    const auto upload = [&robot](ec::JointTrajectory t) {
        ec::motion::Timestamped run(robot, std::move(t));
        const auto r = run.upload();
        return r ? std::string{} : r.error().message;
    };
    auto t = there_and_back();
    EXPECT_EQ(upload({t[0], t[1]}), "2 points; the timestamp service takes at least 3");
    auto late = t;
    late[0].stamp = 50ms;
    EXPECT_EQ(upload(late), "the first stamp is 0.05 s, not 0");
    auto back = t;
    back[3].stamp = back[2].stamp;
    EXPECT_EQ(upload(back), "point 3: its stamp, 0.2 s, does not increase");
    ec::motion::Timestamped run(robot, t);
    EXPECT_EQ(run.start().error().kind, Error::Kind::Invalid);  // not uploaded
    EXPECT_TRUE(controller.calls().empty());
    ASSERT_TRUE(run.upload());
    EXPECT_EQ(run.start(0.0005).error().kind, Error::Kind::Invalid);  // speed_percent below 0.1
    EXPECT_EQ(controller.count("start_trajectory"), 0u);
}

TEST(Timestamped, AnInvalidUploadIsReportedAndNotStarted) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("check_trajectory", "-3");
    ec::motion::Timestamped run(robot, there_and_back());
    const auto r = run.upload();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_NE(r.error().message.find("(-3)"), std::string::npos) << r.error().message;
    EXPECT_EQ(run.start().error().kind, Error::Kind::Invalid);
    EXPECT_EQ(controller.count("start_trajectory"), 0u);
    EXPECT_EQ(controller.count("flush_trajectory"), 0u);            // no clean-up sequence of ours
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));  // nothing open
}

TEST(Timestamped, ARefusedPointNamesIt) {
    MiniController controller;
    auto robot = connect(controller);
    int pushes = 0;
    controller.answer("push_pos", [&pushes](const auto&) {
        return ++pushes == 4 ? std::optional<std::string>{"false"} : std::optional<std::string>{"true"};
    });
    ec::motion::Timestamped run(robot, there_and_back());
    const auto r = run.upload();
    ASSERT_FALSE(r);
    EXPECT_NE(r.error().message.find("point 3: push_pos"), std::string::npos) << r.error().message;
    EXPECT_EQ(controller.count("push_pos"), 4u);
}

TEST(Timestamped, StopWhileItRunsSendsStopTrajectoryThenStop) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::Timestamped run(robot, there_and_back());
    ASSERT_TRUE(run.upload());
    controller.states({}, kRunning);
    ASSERT_TRUE(run.start());
    std::jthread stopper([&robot] {
        std::this_thread::sleep_for(100ms);
        robot.stop();
    });
    const auto r = run.wait();
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
    ec::motion::ServoStream stream(robot, {.lookahead = 40ms, .period = 8ms, .smoothness = 0.2});
    EXPECT_TRUE(controller.calls().empty());  // constructed: nothing sent
    ASSERT_TRUE(stream.open());
    EXPECT_EQ(controller.methods(),
              (std::vector<std::string>{"get_transparent_transmission_state", "tt_clear_servo_joint_buf",
                                        "transparent_transmission_init"}));
    const auto init = controller.last("transparent_transmission_init");
    EXPECT_EQ(init["lookahead"], 40);
    EXPECT_EQ(init["t"], 8);
    EXPECT_NEAR(init["smoothness"].get<double>(), 0.2, 1e-12);
    ASSERT_TRUE(stream.add_point(kHere));
    EXPECT_NEAR(controller.last("tt_put_servo_joint_to_buf")["targetPos"][2].get<double>(), 90.0, 1e-9);
    Joints far = kHere;
    far[2] = 7.0;
    EXPECT_EQ(stream.add_point(far).error().kind, Error::Kind::Invalid);
    ASSERT_TRUE(stream.close());
    EXPECT_EQ(controller.count("tt_clear_servo_joint_buf"), 2u);
    ec::motion::Move move(robot);
    EXPECT_TRUE(move.joint(kHere));  // closed: another motion may open
}

TEST(ServoStream, OptionsOutsideTheManualsRangesAreInvalidAndSendNothing) {
    MiniController controller;
    auto robot = connect(controller);
    for (const auto& options : {ec::motion::ServoOptions{.lookahead = 5ms}, ec::motion::ServoOptions{.period = 1ms},
                                ec::motion::ServoOptions{.smoothness = 1.5}}) {
        ec::motion::ServoStream stream(robot, options);
        EXPECT_EQ(stream.open().error().kind, Error::Kind::Invalid);
        EXPECT_EQ(stream.add_point(kHere).error().kind, Error::Kind::Invalid);  // not open
    }
    EXPECT_TRUE(controller.calls().empty());
}

TEST(ServoStream, DestroyedOpenItClosesAndTheRobotIsFree) {
    MiniController controller;
    auto robot = connect(controller);
    {
        ec::motion::ServoStream stream(robot);
        ASSERT_TRUE(stream.open());
    }
    EXPECT_EQ(controller.count("tt_clear_servo_joint_buf"), 1u);  // as close()
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));
}

}  // namespace
