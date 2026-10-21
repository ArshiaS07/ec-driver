// readiness: each blocker, and their order, from states built by hand.

#include "ec/readiness.hpp"

#include <gtest/gtest.h>

#include <format>

namespace {

using ec::Blocker;
using ec::readiness;
using ec::RobotState;

// Remote, synchronised, servos on, at stop: nothing missing.
ec::State ready() {
    ec::State s;
    s.system.mode = ec::RobotMode::Remote;
    s.drives = {.servo_on = true, .synced = true};
    s.motion.state = RobotState::Stop;
    return s;
}

TEST(Readiness, NothingMissing) {
    EXPECT_EQ(readiness(ready()), Blocker::None);
    EXPECT_EQ(readiness(ready(), /*precise=*/true), Blocker::None);
}

TEST(Readiness, EachBlockerAlone) {
    auto s = ready();
    s.system.mode = ec::RobotMode::Teach;
    EXPECT_EQ(readiness(s), Blocker::NotRemote);

    s = ready();
    s.safety.emergency_stop = true;
    EXPECT_EQ(readiness(s), Blocker::EmergencyStop);

    s = ready();
    s.safety.collision = true;
    EXPECT_EQ(readiness(s), Blocker::Collision);
    s = ready();
    s.motion.state = RobotState::Collision;
    EXPECT_EQ(readiness(s), Blocker::Collision);

    s = ready();
    s.motion.state = RobotState::Alarm;
    EXPECT_EQ(readiness(s), Blocker::Alarm);

    s = ready();
    s.drives.synced = false;
    EXPECT_EQ(readiness(s), Blocker::NotSynced);

    s = ready();
    s.drives.servo_on = false;
    EXPECT_EQ(readiness(s), Blocker::ServoOff);

    EXPECT_EQ(readiness(ready(), /*precise=*/false), Blocker::NotPrecise);

    for (const auto state : {RobotState::Running, RobotState::Pause, static_cast<RobotState>(9)}) {
        s = ready();
        s.motion.state = state;
        EXPECT_EQ(readiness(s), Blocker::NotStopped) << std::format("{}", state);
    }
}

TEST(Readiness, TheShortLivedEmergencyStopStateIsAnAlarmOnceTheButtonIsOut) {
    auto s = ready();
    s.motion.state = RobotState::EmergencyStop;
    EXPECT_EQ(readiness(s), Blocker::Alarm);
    s.safety.emergency_stop = true;
    EXPECT_EQ(readiness(s), Blocker::EmergencyStop);
}

TEST(Readiness, TheOperatorsStepsComeFirstThenTheManualsOrder) {
    ec::State s;  // teach, nothing on
    s.safety = {.emergency_stop = true, .collision = true};
    s.motion.state = RobotState::Alarm;
    EXPECT_EQ(readiness(s, false), Blocker::NotRemote);
    s.system.mode = ec::RobotMode::Remote;
    EXPECT_EQ(readiness(s, false), Blocker::EmergencyStop);
    s.safety.emergency_stop = false;
    EXPECT_EQ(readiness(s, false), Blocker::Collision);
    s.safety.collision = false;
    EXPECT_EQ(readiness(s, false), Blocker::Alarm);
    s.motion.state = RobotState::Stop;
    EXPECT_EQ(readiness(s, false), Blocker::NotSynced);
    s.drives.synced = true;
    EXPECT_EQ(readiness(s, false), Blocker::ServoOff);  // calibrating needs the servo on
    s.drives.servo_on = true;
    EXPECT_EQ(readiness(s, false), Blocker::NotPrecise);
    EXPECT_EQ(readiness(s, true), Blocker::None);
}

TEST(Readiness, NamesAndRemedies) {
    for (const auto b : {Blocker::None, Blocker::NotRemote, Blocker::EmergencyStop, Blocker::Collision, Blocker::Alarm,
                         Blocker::NotSynced, Blocker::ServoOff, Blocker::NotPrecise, Blocker::NotStopped}) {
        EXPECT_NE(ec::to_string(b), "unknown");
        EXPECT_NE(ec::remedy(b), "unknown");
    }
    EXPECT_EQ(std::format("{}: {}", Blocker::ServoOff, ec::remedy(Blocker::ServoOff)),
              "servo off: set_servo_status 1 (the brakes release with a small motion)");
    EXPECT_TRUE(ec::needs_operator(Blocker::NotRemote));
    EXPECT_TRUE(ec::needs_operator(Blocker::EmergencyStop));
    EXPECT_FALSE(ec::needs_operator(Blocker::Alarm));
}

}  // namespace
