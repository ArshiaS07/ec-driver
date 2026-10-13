// The planned-motion activities on the bench: the commands they send (in the
// controller's units), how they recognise arrival and failure in the state,
// and what their close() does.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "ec/motion/bring_up.hpp"
#include "ec/motion/point_to_point.hpp"
#include "ec/motion/sequence.hpp"
#include "ec/motion/trajectory.hpp"
#include "support/bench.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Joints;
using ec::command::Json;
using ec::motion::End;
using ec::motion::Error;
using ec::motion::Phase;
using ec::test::Bench;

constexpr Joints kTarget{0.1, -0.2, 0.3, 0.0, 0.5, -0.6};

// The arm moving between where it was and `q`.
void moving(Bench& b, const Joints& q) {
    b.state.robot_state = ec::RobotState::Running;
    b.state.joint_position = q;
    b.state.joint_velocity = {0.05, 0, 0, 0, 0, 0};
}

// The arm at rest at `q`.
void resting(Bench& b, const Joints& q) {
    b.state.robot_state = ec::RobotState::Stop;
    b.state.joint_position = q;
    b.state.joint_velocity = {};
}

TEST(MoveJoint, SendsDegreesAndPercentAndEndsAtRestAtTheTarget) {
    Bench b;
    auto a = make_activity(ec::motion::MoveJoint{.target = kTarget, .speed = 0.2, .acceleration = 0.5});
    EXPECT_EQ(a.needs().to_start, ec::motion::kCanStartMotion);
    EXPECT_EQ(b.step(a), Phase::Running);
    ASSERT_EQ(b.methods(), std::vector<std::string>{"moveByJoint"});
    const Json& p = b.sent[0].params;
    EXPECT_NEAR(p["targetPos"][0].get<double>(), 5.7296, 1e-4);
    EXPECT_NEAR(p["targetPos"][5].get<double>(), -34.3775, 1e-4);
    EXPECT_DOUBLE_EQ(p["speed"].get<double>(), 20.0);
    EXPECT_EQ(p["acc"], 50);
    EXPECT_FALSE(p.contains("dec"));

    EXPECT_EQ(b.step(a), Phase::Running);  // reply: accepted
    moving(b, {0.05, -0.1, 0.15, 0, 0.25, -0.3});
    EXPECT_EQ(b.step(a), Phase::Running);
    resting(b, kTarget);
    EXPECT_EQ(b.step(a), Phase::Done);
    EXPECT_TRUE(b.close(a, End::Done));
    EXPECT_EQ(b.count("stop"), 0u);
}

TEST(MoveJoint, AlreadyThereIsDoneWithoutMoving) {
    Bench b;
    resting(b, kTarget);
    auto a = make_activity(ec::motion::MoveJoint{.target = kTarget});
    ASSERT_TRUE(b.step(a));
    EXPECT_EQ(b.step(a), Phase::Running);
    EXPECT_EQ(b.step(a), Phase::Done);
}

TEST(MoveJoint, ARefusedCommandIsACommandError) {
    Bench b;
    b.fail("moveByJoint");
    auto a = make_activity(ec::motion::MoveJoint{.target = kTarget});
    ASSERT_TRUE(b.step(a));
    const auto r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_NE(r.error().message.find("moveByJoint"), std::string::npos);
}

TEST(MoveJoint, StoppingShortOfTheTargetIsAnError) {
    Bench b;
    auto a = make_activity(ec::motion::MoveJoint{.target = kTarget});
    ASSERT_TRUE(b.step(a));
    ASSERT_TRUE(b.step(a));
    moving(b, {0.05, 0, 0, 0, 0, 0});
    ASSERT_TRUE(b.step(a));
    resting(b, {0.05, 0, 0, 0, 0, 0});
    EXPECT_EQ(b.step(a), Phase::Running);
    EXPECT_EQ(b.step(a), Phase::Running);
    const auto r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_NE(r.error().message.find("stopped"), std::string::npos);
}

TEST(MoveJoint, NotArrivingInTimeIsATimeout) {
    Bench b;
    auto a = make_activity(ec::motion::MoveJoint{.target = kTarget, .timeout = 100ms});
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 20 && r && *r == Phase::Running; ++i) r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Timeout);
}

TEST(MoveJoint, ImpossibleArgumentsAreInvalidAndSendNothing) {
    for (const auto& spec :
         {ec::motion::MoveJoint{.target = kTarget, .speed = 0.0},
          ec::motion::MoveJoint{.target = kTarget, .speed = 1.5}, ec::motion::MoveJoint{.target = {7.0, 0, 0, 0, 0, 0}},
          ec::motion::MoveJoint{.target = kTarget, .acceleration = 0.0}}) {
        Bench b;
        auto a = make_activity(spec);
        const auto r = b.step(a);
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().kind, Error::Kind::Invalid);
        EXPECT_TRUE(b.sent.empty());
        EXPECT_TRUE(b.close(a, End::Cancelled));
    }
}

TEST(MoveJoint, CancelStopsTheArmAndWaitsForRest) {
    Bench b;
    auto a = make_activity(ec::motion::MoveJoint{.target = kTarget});
    ASSERT_TRUE(b.step(a));
    ASSERT_TRUE(b.step(a));
    moving(b, {0.05, 0, 0, 0, 0, 0});
    ASSERT_TRUE(b.step(a));
    EXPECT_FALSE(b.close(a, End::Cancelled));
    EXPECT_EQ(b.count("stop"), 1u);
    EXPECT_FALSE(b.close(a, End::Cancelled));  // still moving
    resting(b, {0.06, 0, 0, 0, 0, 0});
    EXPECT_TRUE(b.close(a, End::Cancelled));
    EXPECT_EQ(b.count("stop"), 1u);
}

TEST(BringUp, ClearsTheAlarmSynchronisesAndSwitchesTheServosOnInOrder) {
    Bench b;
    b.state.robot_state = ec::RobotState::Alarm;
    b.state.brakes_released = false;
    b.state.motors_synced = false;
    ec::motion::BringUpActivity a(ec::motion::BringUp{});
    EXPECT_FALSE(a.needs().to_start.has(ec::motion::Condition::ServoOn));
    EXPECT_FALSE(a.needs().to_start.has(ec::motion::Condition::NoAlarm));

    EXPECT_EQ(b.step(a), Phase::Running);
    EXPECT_EQ(b.step(a), Phase::Running);  // accepted, but the alarm is still shown
    b.state.robot_state = ec::RobotState::Stop;
    EXPECT_EQ(b.step(a), Phase::Running);  // alarm gone: synchronise
    EXPECT_EQ(b.step(a), Phase::Running);
    b.state.motors_synced = true;
    EXPECT_EQ(b.step(a), Phase::Running);  // synchronised: servos on
    EXPECT_EQ(b.sent.back().params, Json::parse(R"({"status":1})"));
    EXPECT_EQ(b.step(a), Phase::Running);
    b.state.brakes_released = true;
    EXPECT_EQ(b.step(a), Phase::Done);
    EXPECT_EQ(b.methods(), (std::vector<std::string>{"clearAlarm", "syncMotorStatus", "set_servo_status"}));
}

TEST(BringUp, AStepWithoutEffectFailsAfterItsTimeout) {
    Bench b;
    b.state.motors_synced = false;
    ec::motion::BringUpActivity a(ec::motion::BringUp{.step_timeout = 50ms});
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 20 && r && *r == Phase::Running; ++i) r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_NE(r.error().message.find("syncMotorStatus"), std::string::npos);
    EXPECT_EQ(b.count("syncMotorStatus"), 1u);
}

TEST(BringUp, NothingToDoIsDoneAtOnce) {
    Bench b;
    ec::motion::BringUpActivity a(ec::motion::BringUp{});
    EXPECT_EQ(b.step(a), Phase::Done);
    EXPECT_TRUE(b.sent.empty());
}

TEST(Sequence, RunsItsStepsBackToBack) {
    Bench b;
    ec::motion::Sequence s(ec::motion::MoveJoint{.target = kTarget}, ec::motion::MoveJoint{.target = Joints{}});
    EXPECT_EQ(s.size(), 2u);
    EXPECT_EQ(b.step(s), Phase::Running);  // first move sent
    EXPECT_EQ(b.step(s), Phase::Running);
    moving(b, {0.05, 0, 0, 0, 0, 0});
    ASSERT_TRUE(b.step(s));
    resting(b, kTarget);
    EXPECT_EQ(b.step(s), Phase::Running);  // first done and closed; the second sent in the same cycle
    EXPECT_EQ(b.methods(), (std::vector<std::string>{"moveByJoint", "moveByJoint"}));
    ASSERT_TRUE(b.step(s));
    moving(b, {0.05, 0, 0, 0, 0, 0});
    ASSERT_TRUE(b.step(s));
    resting(b, Joints{});
    EXPECT_EQ(b.step(s), Phase::Done);
    EXPECT_TRUE(b.close(s, End::Done));
}

TEST(Sequence, AFailingStepFailsTheSequenceAndIsClosedByIt) {
    Bench b;
    ec::motion::Sequence s(ec::motion::MoveJoint{.target = kTarget}, ec::motion::MoveJoint{.target = kTarget});
    b.fail("moveByJoint");
    ASSERT_TRUE(b.step(s));
    const auto r = b.step(s);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_FALSE(b.close(s, End::Faulted));  // the failed move is stopped
    EXPECT_EQ(b.count("stop"), 1u);
    EXPECT_TRUE(b.close(s, End::Faulted));
}

TEST(Sequence, AnEmptySequenceIsDone) {
    Bench b;
    ec::motion::Sequence s;
    EXPECT_EQ(b.step(s), Phase::Done);
    EXPECT_TRUE(b.close(s, End::Done));
}

ec::motion::JointTrajectory ramp(int points, double dt = 0.1) {
    ec::motion::JointTrajectory t;
    for (int k = 0; k < points; ++k) {
        t.time.push_back(k * dt);
        t.position.push_back(Joints{0.01 * k, 0, 0, 0, 0, 0});
    }
    return t;
}

TEST(Trajectory, ValidationNamesTheFirstProblem) {
    EXPECT_TRUE(validate(ramp(3)));
    EXPECT_FALSE(validate(ramp(2)));
    auto late = ramp(4);
    late.time[0] = 0.1;
    EXPECT_FALSE(validate(late));
    auto stuck = ramp(4);
    stuck.time[2] = stuck.time[1];
    EXPECT_NE(validate(stuck).error().find("does not increase"), std::string::npos);
    auto fast = ramp(4, 0.001);  // 10 rad/s
    EXPECT_NE(validate(fast).error().find("rad/s"), std::string::npos);
    auto far = ramp(4);
    far.position[3][2] = 7.0;
    EXPECT_FALSE(validate(far));
    auto ragged = ramp(4);
    ragged.position.pop_back();
    EXPECT_FALSE(validate(ragged));
}

TEST(PlayTrajectory, UploadsChecksRunsAndFlushes) {
    Bench b;
    b.answers["getSpeed"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(50.0); };
    b.answers["check_trajectory"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(0); };
    const auto path = ramp(12);
    auto a = make_activity(ec::motion::PlayTrajectory{.path = path, .speed = 0.5});

    for (int i = 0; i < 20 && b.count("start_trajectory") == 0; ++i) ASSERT_EQ(b.step(a), Phase::Running);
    EXPECT_EQ(b.count("getSpeed"), 1u);
    EXPECT_EQ(b.count("start_push_pos"), 1u);
    EXPECT_EQ(b.count("push_pos"), 12u);
    EXPECT_EQ(b.count("stop_push_pos"), 1u);
    EXPECT_EQ(b.count("check_trajectory"), 1u);
    for (const auto& s : b.sent) {
        if (s.method == "start_push_pos") {
            EXPECT_EQ(s.params["path_lenth"], 12);
            EXPECT_EQ(s.params["ret_flag"], 1);
        }
        if (s.method == "start_trajectory") {
            EXPECT_DOUBLE_EQ(s.params["speed_percent"].get<double>(), 50.0);
        }
    }
    const auto& last_push = [&] {
        const ec::test::Sent* p = nullptr;
        for (const auto& s : b.sent)
            if (s.method == "push_pos") p = &s;
        return *p;
    }();
    EXPECT_NEAR(last_push.params["timestamp"].get<double>(), 1.1, 1e-12);
    EXPECT_NEAR(last_push.params["pos"][0].get<double>(), ec::rad_to_deg(0.11), 1e-9);

    EXPECT_EQ(b.step(a), Phase::Running);  // accepted: running
    moving(b, path.position[5]);
    EXPECT_EQ(b.step(a), Phase::Running);
    resting(b, path.position.back());
    EXPECT_EQ(b.step(a), Phase::Done);
    EXPECT_FALSE(b.close(a, End::Done));
    EXPECT_EQ(b.sent.back().method, "flush_trajectory");
    EXPECT_TRUE(b.close(a, End::Done));
}

TEST(PlayTrajectory, RefusesToStartAwayFromTheFirstPoint) {
    Bench b;
    resting(b, {0.2, 0, 0, 0, 0, 0});
    auto a = make_activity(ec::motion::PlayTrajectory{.path = ramp(5)});
    const auto r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Invalid);
    EXPECT_NE(r.error().message.find("move there first"), std::string::npos);
    EXPECT_TRUE(b.close(a, End::Cancelled));
    EXPECT_TRUE(b.sent.empty());
}

TEST(PlayTrajectory, AnUploadTheControllerRejectsIsFlushed) {
    Bench b;
    b.answers["getSpeed"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(100.0); };
    b.answers["check_trajectory"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(-3); };
    auto a = make_activity(ec::motion::PlayTrajectory{.path = ramp(4)});
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 20 && r; ++i) r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
    EXPECT_EQ(b.count("start_trajectory"), 0u);
    EXPECT_FALSE(b.close(a, End::Faulted));
    EXPECT_EQ(b.sent.back().method, "flush_trajectory");
    EXPECT_TRUE(b.close(a, End::Faulted));
}

// Out and back to where it started: a closed path, as a square or a circle is.
ec::motion::JointTrajectory there_and_back() {
    ec::motion::JointTrajectory t;
    for (int k = 0; k <= 10; ++k) {
        t.time.push_back(0.1 * k);
        t.position.push_back(Joints{0.002 * std::min(k, 10 - k), 0, 0, 0, 0, 0});
    }
    return t;
}

void answer_upload(Bench& b) {
    b.answers["getSpeed"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(100.0); };
    b.answers["check_trajectory"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(0); };
}

TEST(PlayTrajectory, AClosedPathIsDoneOnlyAfterTheArmMoved) {
    Bench b;
    answer_upload(b);
    auto a = make_activity(ec::motion::PlayTrajectory{.path = there_and_back(), .margin = 500ms});
    for (int i = 0; i < 20 && b.count("start_trajectory") == 0; ++i) ASSERT_TRUE(b.step(a));
    // The arm never moves: at rest at the first point, which is also the last.
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 400 && r && *r == Phase::Running; ++i) r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Timeout);
}

TEST(PlayTrajectory, ATrajectoryThatStaysPutIsDoneAfterItsTime) {
    Bench b;
    answer_upload(b);
    ec::motion::JointTrajectory still;
    still.time = {0.0, 0.1, 0.2};
    still.position = {Joints{}, Joints{}, Joints{}};
    auto a = make_activity(ec::motion::PlayTrajectory{.path = still});
    for (int i = 0; i < 20 && b.count("start_trajectory") == 0; ++i) ASSERT_TRUE(b.step(a));
    int cycles = 0;
    std::expected<Phase, Error> r = Phase::Running;
    while (r && *r == Phase::Running && cycles < 100) {
        r = b.step(a);
        ++cycles;
    }
    ASSERT_TRUE(r) << describe(r.error());
    EXPECT_EQ(*r, Phase::Done);
    EXPECT_NEAR(cycles * 8, 200, 16);  // its 0.2 s
}

TEST(PlayTrajectory, AStopRightAfterTheStartStopsTheArm) {
    Bench b;
    answer_upload(b);
    auto a = make_activity(ec::motion::PlayTrajectory{.path = ramp(6)});
    for (int i = 0; i < 20 && b.count("start_trajectory") == 0; ++i) ASSERT_TRUE(b.step(a));
    // start_trajectory went out and its reply has not been looked at: the arm may move.
    EXPECT_FALSE(b.close(a, End::Stopped));
    EXPECT_EQ(b.count("stop_trajectory"), 1u);
    EXPECT_EQ(b.count("stop"), 1u);
    EXPECT_FALSE(b.close(a, End::Stopped));  // at rest: flush
    EXPECT_EQ(b.sent.back().method, "flush_trajectory");
    EXPECT_TRUE(b.close(a, End::Stopped));
}

TEST(PlayTrajectory, CancelWhileRunningStopsItThenFlushes) {
    Bench b;
    b.answers["getSpeed"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(100.0); };
    b.answers["check_trajectory"] = [](const auto&) -> std::expected<Json, ec::command::Error> { return Json(0); };
    const auto path = ramp(6);
    auto a = make_activity(ec::motion::PlayTrajectory{.path = path});
    for (int i = 0; i < 20 && b.count("start_trajectory") == 0; ++i) ASSERT_TRUE(b.step(a));
    ASSERT_TRUE(b.step(a));
    moving(b, path.position[2]);
    ASSERT_TRUE(b.step(a));
    EXPECT_FALSE(b.close(a, End::Cancelled));
    EXPECT_EQ(b.count("stop_trajectory"), 1u);
    EXPECT_FALSE(b.close(a, End::Cancelled));  // still moving
    resting(b, path.position[3]);
    EXPECT_FALSE(b.close(a, End::Cancelled));  // at rest: flush
    EXPECT_EQ(b.sent.back().method, "flush_trajectory");
    EXPECT_TRUE(b.close(a, End::Cancelled));
}

}  // namespace
