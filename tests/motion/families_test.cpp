// The motion families against a scripted controller: each sends its
// SDK calls as the manual, in the controller's units, every call answered; one
// motion is open at a time; waits read getRobotState; motion().stop() sends the
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

#include "ec/motion/jog.hpp"
#include "ec/motion/move.hpp"
#include "ec/motion/path.hpp"
#include "ec/motion/servo_stream.hpp"
#include "ec/motion/speed_stream.hpp"
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
    Joints too_far = kHere;
    too_far[0] = ec::deg_to_rad(400.0);
    const auto kind = [](const std::expected<void, Error>& r) {
        return r ? std::optional<Error::Kind>{} : r.error().kind;
    };
    EXPECT_EQ(kind(move.joint(too_far)), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.joint(kHere, {.speed = 0.0})), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.joint(kHere, {.speed = 0.5, .acceleration = 2.0})), Error::Kind::Invalid);
    EXPECT_EQ(kind(move.line(kHere, {.speed = 5.0})), Error::Kind::Invalid);  // above 3000 mm/s
    EXPECT_EQ(kind(move.rotate(kHere, {.speed_type = ec::sdk::SpeedType::Rotational, .speed = ec::deg_to_rad(400.0)})),
              Error::Kind::Invalid);
    EXPECT_EQ(kind(move.arc(too_far, kHere)), Error::Kind::Invalid);
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
        robot.motion().stop();
    });
    const auto r = move.wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Stopped);
    // The wait ends when it sees the request; stop() sends its calls after that, on its own thread.
    stopper.join();
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
        robot.motion().stop();
    });
    const auto r = run.wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Stopped);
    stopper.join();  // as in Move.StopFromAnotherThreadEndsTheWait: the calls come after the wait ends
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
    Joints too_far = kHere;
    too_far[2] = 7.0;
    EXPECT_EQ(stream.add_point(too_far).error().kind, Error::Kind::Invalid);
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

// ------------------------------------------------------------------- lifetimes

TEST(Lifetimes, DestroyingAnOpenMoveStopsTheArm) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({}, kRunning);
    {
        ec::motion::Move move(robot);
        ASSERT_TRUE(move.joint(kHere));
    }
    EXPECT_EQ(controller.count("stop"), 1u);
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));  // nothing open any more
}

TEST(Lifetimes, AMoveSeenToEndSendsNothingMore) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kStop});
    {
        ec::motion::Move move(robot);
        ASSERT_TRUE(move.joint(kHere));
        EXPECT_TRUE(move.wait());
    }
    EXPECT_EQ(controller.count("stop"), 0u);
}

TEST(Lifetimes, DestroyingARunningTrajectoryStopsItAnUploadedOneSendsNothing) {
    MiniController controller;
    auto robot = connect(controller);
    {
        ec::motion::Timestamped uploaded(robot, there_and_back());
        ASSERT_TRUE(uploaded.upload());
    }
    EXPECT_EQ(controller.count("stop_trajectory"), 0u);
    controller.states({}, kRunning);
    {
        ec::motion::Timestamped running(robot, there_and_back());
        ASSERT_TRUE(running.upload());
        ASSERT_TRUE(running.start());
    }
    EXPECT_EQ(controller.count("stop_trajectory"), 1u);
}

TEST(Lifetimes, DestroyingTheRobotStopsItsOpenMotionAndOutlivedFamiliesSaySo) {
    MiniController controller;
    std::optional<ec::motion::Move> move;
    {
        auto robot = connect(controller);
        controller.states({}, kRunning);
        move.emplace(robot);
        ASSERT_TRUE(move->joint(kHere));
    }
    EXPECT_EQ(controller.count("stop"), 2u);  // the move's own stop, then stop
    const auto waited = move->wait();
    ASSERT_FALSE(waited);
    EXPECT_EQ(waited.error().kind, Error::Kind::Disconnected);
    EXPECT_EQ(move->joint(kHere).error().kind, Error::Kind::Disconnected);
}

// ------------------------------------------------------- waits on the stream

ec::Robot connect_monitoring(MiniController& controller) {
    auto robot = ec::Robot::connect(controller.options());
    if (!robot) throw std::runtime_error(describe(robot.error()));
    controller.forget();
    return std::move(*robot);
}

TEST(Waits, ReadTheMonitoringPathWhileItStreams) {
    MiniController controller;
    auto robot = connect_monitoring(controller);
    ec::test::FrameFields running;
    running.robot_state = kRunning;
    controller.frames(running);
    ec::motion::Move move(robot);
    ASSERT_TRUE(move.joint(kHere));
    std::jthread arrives([&controller] {
        std::this_thread::sleep_for(150ms);
        controller.frames(ec::test::FrameFields{});  // at rest
    });
    EXPECT_TRUE(move.wait());
    EXPECT_EQ(controller.count("getRobotState"), 0u);  // nothing asked on the command connection
}

TEST(Waits, AnEmergencyStopOnTheStreamIsAFault) {
    MiniController controller;
    auto robot = connect_monitoring(controller);
    ec::test::FrameFields pressed;
    pressed.estop = true;
    ec::motion::Move move(robot);
    ASSERT_TRUE(move.joint(kHere));
    controller.frames(pressed);
    const auto r = move.wait();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Fault);
    EXPECT_EQ(r.error().message, "emergency stop");
}

// ------------------------------------------------------------------ Controller

TEST(Controller, StateSpeedSliderHandGuidingAndTheWaitForRest) {
    MiniController controller;
    auto robot = connect(controller);
    const auto motion = robot.motion();
    controller.answer("getSpeed", "50.0");
    controller.answer("get_drag_state", "false");
    controller.states({kRunning, kRunning, kStop});

    EXPECT_EQ(motion.state(), ec::RobotState::Running);
    const auto rest = motion.wait_until_stopped(5s);
    ASSERT_TRUE(rest);
    EXPECT_TRUE(*rest);
    EXPECT_DOUBLE_EQ(motion.speed_scale().value(), 0.5);
    EXPECT_TRUE(motion.set_speed_scale(0.3));
    EXPECT_DOUBLE_EQ(controller.last("setSpeed")["value"].get<double>(), 30.0);
    EXPECT_EQ(motion.set_speed_scale(0.0001).error().kind, Error::Kind::Invalid);
    EXPECT_EQ(motion.set_speed_scale(1.5).error().kind, Error::Kind::Invalid);
    EXPECT_EQ(motion.hand_guiding(), false);
    EXPECT_TRUE(motion.set_hand_guiding(true));
    EXPECT_EQ(controller.last("drag_teach_switch")["switch"], 1);
}

TEST(Controller, TheWaitForRestIsNotEndedByAStop) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({kRunning, kRunning, kStop});
    robot.motion().stop();
    const auto rest = robot.motion().wait_until_stopped(5s);  // the arm braking to rest
    ASSERT_TRUE(rest);
    EXPECT_TRUE(*rest);
    EXPECT_EQ(controller.count("getRobotState"), 3u);
}

// ------------------------------------------------------------------------ Path

TEST(Path, ClearsAddsEachWaypointRunsAndWaits) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("moveByPath", "2");
    controller.answer("getPathPointIndex", "1");
    controller.states({kRunning, kRunning, kStop});
    Joints there = kHere;
    there[0] = ec::deg_to_rad(10.0);
    ec::motion::Path path(robot, {{.target = there, .speed = 0.2, .blend_radius = 0.02},
                                  {.target = kHere, .move = ec::sdk::PathMoveType::Line, .speed = 0.1}});
    EXPECT_TRUE(controller.calls().empty());  // constructed: nothing sent
    ASSERT_TRUE(path.run());
    EXPECT_EQ(controller.methods(),
              (std::vector<std::string>{"clearPathPoint", "addPathPoint", "addPathPoint", "moveByPath"}));
    const auto first = controller.calls()[1].params;
    EXPECT_NEAR(first["wayPoint"][0].get<double>(), 10.0, 1e-9);  // deg
    EXPECT_EQ(first["moveType"], 0);
    EXPECT_NEAR(first["speed"].get<double>(), 20.0, 1e-9);            // % of the joints' maximum
    EXPECT_NEAR(first["circular_radius"].get<double>(), 20.0, 1e-9);  // mm
    const auto second = controller.calls()[2].params;
    EXPECT_EQ(second["moveType"], 1);
    EXPECT_NEAR(second["speed"].get<double>(), 100.0, 1e-9);  // mm/s
    EXPECT_EQ(path.index(), 1);
    EXPECT_TRUE(path.wait());
}

TEST(Path, WaypointsOutsideTheManualsRangesAreInvalidAndSendNothing) {
    MiniController controller;
    auto robot = connect(controller);
    const auto kind = [&robot](std::vector<ec::motion::Waypoint> w) {
        ec::motion::Path path(robot, std::move(w));
        const auto r = path.run();
        return r ? std::optional<Error::Kind>{} : r.error().kind;
    };
    EXPECT_EQ(kind({}), Error::Kind::Invalid);
    EXPECT_EQ(kind({{.target = kHere, .speed = 0.005}}), Error::Kind::Invalid);        // under 1 %
    EXPECT_EQ(kind({{.target = kHere, .blend_radius = 0.01}}), Error::Kind::Invalid);  // the last one blends
    EXPECT_EQ(kind({{.target = kHere, .move = ec::sdk::PathMoveType::Line, .speed = 4.0}}), Error::Kind::Invalid);
    EXPECT_TRUE(controller.calls().empty());
}

TEST(Path, AnotherCountFromTheControllerIsAnErrorAndStops) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("moveByPath", "1");
    ec::motion::Path path(robot, {{.target = kHere}, {.target = kHere}});
    const auto r = path.run();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_EQ(controller.count("stop"), 1u);
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));  // nothing left open
}

// ------------------------------------------------------------------------- Jog

TEST(Jog, SelectsTheFrameOnceThenRepeatsJogAndStops) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::Jog jog(robot, {.frame = ec::sdk::CoordinateSystem::Base, .speed = 0.1});
    ASSERT_TRUE(jog.step(3, ec::motion::Direction::Minus));  // -z
    ASSERT_TRUE(jog.step(3, ec::motion::Direction::Minus));
    ASSERT_TRUE(jog.step(1, ec::motion::Direction::Plus));  // +x
    EXPECT_EQ(controller.methods(), (std::vector<std::string>{"setCurrentCoord", "jog", "jog", "jog"}));
    EXPECT_EQ(controller.calls()[0].params["coord_mode"], 1);
    EXPECT_EQ(controller.calls()[1].params["index"], 4);
    EXPECT_EQ(controller.calls()[3].params["index"], 1);
    EXPECT_NEAR(controller.calls()[1].params["speed"].get<double>(), 10.0, 1e-9);
    EXPECT_EQ(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}).error().kind, Error::Kind::Busy);  // open
    ASSERT_TRUE(jog.stop());
    EXPECT_EQ(controller.count("stop"), 1u);
    EXPECT_TRUE(robot.call(ec::sdk::SetSpeed{.value_pct = 50.0}));  // ended
    EXPECT_EQ(jog.step(7, ec::motion::Direction::Plus).error().kind, Error::Kind::Invalid);
}

TEST(Jog, DestroyedOpenItStops) {
    MiniController controller;
    auto robot = connect(controller);
    {
        ec::motion::Jog jog(robot);
        ASSERT_TRUE(jog.step(6, ec::motion::Direction::Plus));
    }
    EXPECT_EQ(controller.count("stop"), 1u);
}

// ----------------------------------------------------------------- SpeedStream

TEST(SpeedStream, JointVelocitiesInTheControllersUnitsThenStopj) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::SpeedStream speed(robot);
    Joints qd{};
    qd[5] = ec::deg_to_rad(10.0);
    ASSERT_TRUE(speed.joints(qd, {.acceleration = ec::deg_to_rad(50.0), .valid_for = 200ms}));
    ASSERT_TRUE(speed.joints(qd, {.acceleration = ec::deg_to_rad(80.0), .valid_for = 200ms}));
    const auto sent = controller.last("moveBySpeedj");
    EXPECT_NEAR(sent["vj"][5].get<double>(), 10.0, 1e-9);
    EXPECT_EQ(sent["acc"], 80);
    EXPECT_NEAR(sent["t"].get<double>(), 0.2, 1e-12);
    EXPECT_EQ(speed.tcp(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()).error().kind, Error::Kind::Invalid);
    ASSERT_TRUE(speed.stop());
    EXPECT_EQ(controller.last("stopj")["acc"], 80);
    EXPECT_EQ(speed.stop().error().kind, Error::Kind::Invalid);  // nothing open
}

TEST(SpeedStream, TcpVelocitiesAndTheRobotsStopUsesTheFamilysOwn) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::SpeedStream speed(robot);
    ASSERT_TRUE(speed.tcp(Eigen::Vector3d(0.0, 0.0, -0.02), Eigen::Vector3d(0.0, 0.0, ec::deg_to_rad(5.0)),
                          {.acceleration = 0.1, .rotational_acceleration = ec::deg_to_rad(30.0)}));
    const auto sent = controller.last("moveBySpeedl");
    EXPECT_NEAR(sent["v"][2].get<double>(), -20.0, 1e-9);  // mm/s
    EXPECT_NEAR(sent["v"][5].get<double>(), 5.0, 1e-9);    // deg/s
    EXPECT_EQ(sent["acc"], 100);                           // mm/s^2
    EXPECT_EQ(sent["arot"], 30);
    robot.motion().stop();
    EXPECT_EQ(controller.methods().back(), "stop");
    EXPECT_EQ(controller.count("stopl"), 1u);  // the stream's own stop first
    EXPECT_EQ(speed.joints(Joints{}, {.acceleration = 0.0}).error().kind, Error::Kind::Invalid);
}

TEST(Timestamped, ASpeedAboveThePlannedOneIsInvalid) {
    MiniController controller;
    auto robot = connect(controller);
    ec::motion::Timestamped run(robot, there_and_back());
    ASSERT_TRUE(run.upload());
    EXPECT_EQ(run.start(1.5).error().kind, Error::Kind::Invalid);
    EXPECT_EQ(controller.count("start_trajectory"), 0u);
}

}  // namespace
