// The conditions observed from a state: each flag, state and threshold removes
// exactly the conditions that depend on it.

#include "ec/motion/conditions.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace {

using namespace std::chrono_literals;
using ec::motion::Condition;
using ec::motion::Conditions;
using ec::motion::observe;
using ec::motion::Thresholds;

// Remote, servos on, synced, stopped, at rest.
ec::feedback::State ready() {
    ec::feedback::State s;
    s.robot_mode = ec::RobotMode::Remote;
    s.robot_state = ec::RobotState::Stop;
    s.brakes_released = true;
    s.motors_synced = true;
    return s;
}

const Conditions kAll = ec::motion::kCanStartMotion | Conditions{Condition::AtRest};

TEST(Conditions, AReadyRobotMeetsThemAll) { EXPECT_EQ(observe(ready(), 8ms, Thresholds{}), kAll); }

TEST(Conditions, EachObservationRemovesItsOwnConditions) {
    struct Case {
        const char* what;
        void (*change)(ec::feedback::State&);
        Conditions lost;
    };
    const Case cases[] = {
        {"teach mode", [](ec::feedback::State& s) { s.robot_mode = ec::RobotMode::Teach; }, {Condition::Remote}},
        {"E-stop flag", [](ec::feedback::State& s) { s.emergency_stop = true; }, {Condition::NoEmergencyStop}},
        {"E-stop state",
         [](ec::feedback::State& s) { s.robot_state = ec::RobotState::EmergencyStop; },
         {Condition::NoEmergencyStop, Condition::Standstill}},
        {"alarm",
         [](ec::feedback::State& s) { s.robot_state = ec::RobotState::Alarm; },
         {Condition::NoAlarm, Condition::Standstill}},
        {"collision flag", [](ec::feedback::State& s) { s.collision = true; }, {Condition::NoCollision}},
        {"collision state",
         [](ec::feedback::State& s) { s.robot_state = ec::RobotState::Collision; },
         {Condition::NoCollision, Condition::Standstill}},
        {"servo off", [](ec::feedback::State& s) { s.brakes_released = false; }, {Condition::ServoOn}},
        {"not synced", [](ec::feedback::State& s) { s.motors_synced = false; }, {Condition::Synced}},
        {"running at rest",
         [](ec::feedback::State& s) { s.robot_state = ec::RobotState::Running; },
         {Condition::Standstill}},
        {"a joint moving",
         [](ec::feedback::State& s) { s.joint_velocity[4] = -0.01; },
         {Condition::AtRest, Condition::Standstill}},
    };
    for (const Case& c : cases) {
        auto s = ready();
        c.change(s);
        EXPECT_EQ(observe(s, 8ms, Thresholds{}), kAll.missing_from(c.lost)) << c.what;
    }
}

TEST(Conditions, AnOldStateIsNotFresh) {
    EXPECT_FALSE(observe(ready(), 50ms, Thresholds{}).has(Condition::Fresh));
    EXPECT_TRUE(observe(ready(), 49ms, Thresholds{}).has(Condition::Fresh));
    EXPECT_FALSE(observe(ready(), -1ms, Thresholds{}).has(Condition::Fresh));  // from the future: not trusted
    EXPECT_TRUE(observe(ready(), 90ms, Thresholds{.fresh_within = 100ms}).has(Condition::Fresh));
}

TEST(Conditions, AnUndefinedRobotStateMeetsNoneThatDependOnIt) {
    auto s = ready();
    s.robot_state = static_cast<ec::RobotState>(9);
    const auto held = observe(s, 8ms, Thresholds{});
    EXPECT_FALSE(held.has(Condition::NoEmergencyStop));
    EXPECT_FALSE(held.has(Condition::NoAlarm));
    EXPECT_FALSE(held.has(Condition::NoCollision));
    EXPECT_FALSE(held.has(Condition::Standstill));
    EXPECT_TRUE(held.has(Condition::AtRest));
}

TEST(Conditions, SetOperationsAndNames) {
    const Conditions need{Condition::Remote, Condition::ServoOn};
    const Conditions held{Condition::Remote, Condition::Fresh};
    EXPECT_FALSE(held.contains(need));
    EXPECT_EQ(need.missing_from(held), Conditions{Condition::ServoOn});
    EXPECT_TRUE((need | held).contains(need));
    EXPECT_EQ(need & held, Conditions{Condition::Remote});
    EXPECT_TRUE(Conditions{}.empty());
    EXPECT_EQ(to_string(need), "remote mode, servo on");
    EXPECT_EQ(to_string(Conditions{}), "none");
}

}  // namespace
