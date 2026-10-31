// The workflows against a scripted controller on 127.0.0.1
// (tests/support/mini_controller.hpp): bring-up sends only what the
// conditions call for and stops, naming the step, where only the operator can
// help; shut-down stops and switches the servos off; recover explains first.

#include "ec/workflows.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "support/mini_controller.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Blocker;
using ec::Error;
using ec::test::MiniController;

// The calls read_conditions makes, in order.
std::vector<std::string> conditions() {
    return {"getRobotMode",  "get_estop_status", "getCollisionState",
            "getRobotState", "getMotorStatus",   "getServoStatus"};
}

ec::Robot connect(MiniController& controller) {
    auto robot = ec::Robot::connect(controller.options(/*monitor=*/false));
    if (!robot) throw std::runtime_error(describe(robot.error()));
    controller.forget();
    return std::move(*robot);
}

std::vector<std::string> concat(std::initializer_list<std::vector<std::string>> parts) {
    std::vector<std::string> out;
    for (const auto& p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}

TEST(BringUp, ReadyAlreadyItOnlyReads) {
    MiniController controller;
    auto robot = connect(controller);
    const auto r = ec::bring_up(robot);
    EXPECT_TRUE(r.ok()) << describe(r);
    EXPECT_EQ(controller.methods(), concat({conditions(), {"getSpeed"}}));
    EXPECT_EQ(r.steps.back().outcome, "100 %");  // the speed slider
}

TEST(BringUp, ClearsSynchronisesAndSwitchesOnAsTheConditionsCallFor) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({4});  // an alarm, then stop
    std::atomic<bool> synced{false};
    controller.answer("syncMotorStatus", [&synced](const auto&) {
        synced = true;
        return std::optional<std::string>("true");
    });
    controller.answer("getMotorStatus",
                      [&synced](const auto&) { return std::optional<std::string>(synced ? "true" : "false"); });
    std::atomic<bool> on{false};
    controller.answer("set_servo_status", [&on](const auto&) {
        on = true;
        return std::optional<std::string>("true");
    });
    controller.answer("getServoStatus",
                      [&on](const auto&) { return std::optional<std::string>(on ? "true" : "false"); });

    const auto r = ec::bring_up(robot);
    ASSERT_TRUE(r.ok()) << describe(r);
    EXPECT_EQ(controller.methods(), concat({conditions(),
                                            {"clearAlarm"},
                                            conditions(),
                                            {"syncMotorStatus"},
                                            conditions(),
                                            {"set_servo_status", "getServoStatus"},
                                            conditions(),
                                            {"getSpeed"}}));
    EXPECT_EQ(controller.last("set_servo_status")["status"], 1);
}

TEST(BringUp, ACollisionIsResetThenItsAlarmCleared) {
    MiniController controller;
    auto robot = connect(controller);
    std::atomic<bool> collision{true};
    controller.answer("getCollisionState",
                      [&collision](const auto&) { return std::optional<std::string>(collision ? "1" : "0"); });
    controller.answer("resetCollisionState", [&collision](const auto&) {
        collision = false;
        return std::optional<std::string>("true");
    });
    const auto r = ec::bring_up(robot);
    ASSERT_TRUE(r.ok()) << describe(r);
    EXPECT_EQ(controller.methods(),
              concat({conditions(), {"resetCollisionState", "clearAlarm"}, conditions(), {"getSpeed"}}));
}

TEST(BringUp, StopsWhereOnlyTheOperatorCanHelpAndSendsNothing) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("getRobotMode", "0");  // teach
    auto r = ec::bring_up(robot);
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.blocker, Blocker::NotRemote);
    EXPECT_TRUE(r.operator_needed());
    EXPECT_EQ(r.error->kind, Error::Kind::Command);
    EXPECT_EQ(controller.methods(), conditions());  // read only

    controller.forget();
    controller.answer("getRobotMode", "2");
    controller.answer("get_estop_status", "1");
    r = ec::bring_up(robot);
    EXPECT_EQ(r.blocker, Blocker::EmergencyStop);
    EXPECT_EQ(r.error->kind, Error::Kind::Fault);
    EXPECT_EQ(controller.methods(), conditions());
}

TEST(BringUp, AStepThatDoesNotTakeIsNamed) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({}, 4);  // the alarm stays
    const auto r = ec::bring_up(robot);
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.blocker, Blocker::Alarm);
    EXPECT_EQ(r.error->kind, Error::Kind::Fault);
    EXPECT_NE(r.error->message.find("still alarm"), std::string::npos) << r.error->message;
    EXPECT_EQ(controller.count("clearAlarm"), 1u);
}

TEST(BringUp, ServosThatDoNotComeOnAreATimeout) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("getServoStatus", "false");
    const auto r = ec::bring_up(robot, {.servo_within = 300ms});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error->kind, Error::Kind::Timeout);
    EXPECT_EQ(controller.count("set_servo_status"), 1u);
}

TEST(BringUp, PreciseOnlyWhenAsked) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("get_servo_precise_position_status", "0");
    EXPECT_TRUE(ec::bring_up(robot).ok());
    EXPECT_EQ(controller.count("get_servo_precise_position_status"), 0u);
    const auto r = ec::bring_up(robot, {.require_precise = true});
    EXPECT_EQ(r.blocker, Blocker::NotPrecise);
    EXPECT_TRUE(r.operator_needed());
}

TEST(ShutDown, StopsWaitsForRestAndSwitchesTheServosOff) {
    MiniController controller;
    auto robot = connect(controller);
    std::atomic<bool> on{true};
    controller.answer("set_servo_status", [&on](const auto&) {
        on = false;
        return std::optional<std::string>("true");
    });
    controller.answer("getServoStatus",
                      [&on](const auto&) { return std::optional<std::string>(on ? "true" : "false"); });
    const auto r = ec::shut_down(robot);
    ASSERT_TRUE(r.ok()) << describe(r);
    EXPECT_EQ(controller.methods(), (std::vector<std::string>{"stop", "getRobotState", "getServoStatus",
                                                              "set_servo_status", "getServoStatus"}));
    EXPECT_EQ(controller.last("set_servo_status")["status"], 0);
}

TEST(Recover, ExplainsWithTheAlarmsAndTheLogThenBringsUp) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("getAlarmNum", "[1030, 0, 0, 0, 0]");
    controller.log_lines({"Error: 0-1030-ca"});
    controller.states({4});
    const auto r = ec::recover(robot);
    ASSERT_TRUE(r.ok()) << describe(r);
    EXPECT_EQ(r.steps[0].what, "getAlarmNum");
    EXPECT_EQ(r.steps[0].details.front(), "1030");
    EXPECT_EQ(r.steps[1].details, (std::vector<std::string>{"Error: 0-1030-ca"}));
    EXPECT_EQ(controller.count("clearAlarm"), 1u);
    const auto text = describe(r);
    EXPECT_NE(text.find("0-1030-ca"), std::string::npos);
    EXPECT_NE(text.find("done"), std::string::npos);
}

TEST(Session, StartsUpAndEndsWithTheServosOffWhicheverWayItEnds) {
    MiniController controller;
    auto robot = connect(controller);
    std::atomic<bool> on{true};
    controller.answer("set_servo_status", [&on](const ec::sdk::Json& p) {
        on = p["status"] == 1;
        return std::optional<std::string>("true");
    });
    controller.answer("getServoStatus",
                      [&on](const auto&) { return std::optional<std::string>(on ? "true" : "false"); });
    {
        auto session = ec::Session::start(robot);
        ASSERT_TRUE(session) << describe(session.error());
        EXPECT_TRUE(session->active());
        EXPECT_TRUE(session->started().ok());
        controller.forget();
    }  // left without end(): the destructor shuts down
    EXPECT_EQ(controller.methods(), (std::vector<std::string>{"stop", "getRobotState", "getServoStatus",
                                                              "set_servo_status", "getServoStatus"}));
    EXPECT_FALSE(on.load());

    on = true;
    auto session = ec::Session::start(robot);
    ASSERT_TRUE(session);
    const auto ended = session->end();
    EXPECT_TRUE(ended.ok()) << describe(ended);
    EXPECT_FALSE(session->active());
    controller.forget();
    EXPECT_TRUE(session->end().steps.empty());  // once only
    EXPECT_TRUE(controller.calls().empty());
}

TEST(Session, AFailedStartLeavesWhatItDidNotChange) {
    MiniController controller;
    auto robot = connect(controller);
    controller.answer("getRobotMode", "0");  // teach: nothing is sent
    const auto teach = ec::Session::start(robot);
    ASSERT_FALSE(teach);
    EXPECT_EQ(teach.error().blocker, Blocker::NotRemote);
    EXPECT_EQ(controller.methods(), conditions());

    controller.forget();
    controller.answer("getRobotMode", "2");
    controller.answer("get_servo_precise_position_status", "0");
    std::atomic<bool> on{false};
    controller.answer("set_servo_status", [&on](const ec::sdk::Json& p) {
        on = p["status"] == 1;
        return std::optional<std::string>("true");
    });
    controller.answer("getServoStatus",
                      [&on](const auto&) { return std::optional<std::string>(on ? "true" : "false"); });
    const auto imprecise = ec::Session::start(robot, {.require_precise = true});  // servos on, then not precise
    ASSERT_FALSE(imprecise);
    EXPECT_EQ(imprecise.error().blocker, Blocker::NotPrecise);
    EXPECT_FALSE(on.load());  // switched off again
    EXPECT_EQ(controller.count("set_servo_status"), 2u);
    EXPECT_EQ(controller.count("stop"), 0u);  // nothing of the session moved
}

}  // namespace
