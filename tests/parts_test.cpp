// The parts (Drives, Model, System) against a scripted controller on
// 127.0.0.1 (tests/support/mini_controller.hpp): each operation is its call,
// with its arguments in the controller's units and its reply in SI; ranges
// are checked before anything is sent; settings wait while a motion is open;
// a part outlived by its robot says so.

#include <gtest/gtest.h>

#include <chrono>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ec/motion/move.hpp"
#include "ec/pose.hpp"
#include "ec/robot.hpp"
#include "support/mini_controller.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Error;
using ec::test::MiniController;

ec::Robot connect(const MiniController& controller) {
    auto robot = ec::Robot::connect(controller.options(/*monitor=*/false));
    if (!robot) throw std::runtime_error(describe(robot.error()));
    return std::move(*robot);
}

TEST(Drives, EachOperationIsItsCall) {
    MiniController controller;
    const auto robot = connect(controller);
    const auto drives = robot.drives();
    controller.answer("get_servo_precise_position_status", "0");
    controller.answer("get_servo_alarm_state", "1");
    controller.answer("get_servo_brake_off_status", "[1,0,1,1,1,1]");
    controller.answer("get_backdrive_status", "0");

    EXPECT_EQ(drives.servo_on(), true);
    EXPECT_EQ(drives.synced(), true);
    EXPECT_EQ(drives.precise(), false);
    EXPECT_EQ(drives.servo_alarm(), true);
    EXPECT_EQ(drives.brakes_released(), (std::vector<bool>{true, false, true, true, true, true}));
    EXPECT_EQ(drives.backdrive(), false);
    EXPECT_TRUE(drives.switch_servo(false));
    EXPECT_EQ(controller.last("set_servo_status")["status"], 0);
    EXPECT_TRUE(drives.synchronise());
    EXPECT_TRUE(drives.clear_alarm());
    EXPECT_TRUE(drives.reset());
    EXPECT_TRUE(drives.enter_backdrive());
    EXPECT_TRUE(drives.exit_backdrive());
    EXPECT_EQ(controller.methods(),
              (std::vector<std::string>{"getServoStatus", "getMotorStatus", "get_servo_precise_position_status",
                                        "get_servo_alarm_state", "get_servo_brake_off_status", "get_backdrive_status",
                                        "set_servo_status", "syncMotorStatus", "clearAlarm", "reset_robot_status",
                                        "enter_backdrive", "exit_backdrive"}));

    controller.answer("clearAlarm", "false");
    const auto refused = drives.clear_alarm();
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().kind, Error::Kind::Command);
}

TEST(Drives, ActionsWaitWhileAMotionIsOpenReadingsDoNot) {
    MiniController controller;
    auto robot = connect(controller);
    controller.states({3, 3, 0});  // running, then stopped
    ec::motion::Move move(robot);
    ASSERT_TRUE(move.joint(ec::Joints{}));
    EXPECT_EQ(robot.drives().switch_servo(false).error().kind, Error::Kind::Busy);
    EXPECT_TRUE(robot.drives().servo_on());
    EXPECT_TRUE(move.wait());
    EXPECT_TRUE(robot.drives().switch_servo(false));
}

TEST(Parts, OutlivingTheRobotIsDisconnected) {
    MiniController controller;
    std::optional<ec::Drives> drives;
    std::optional<ec::System> system;
    {
        const auto robot = connect(controller);
        drives = robot.drives();
        system = robot.system();
        EXPECT_TRUE(drives->servo_on());
    }
    EXPECT_EQ(drives->servo_on().error().kind, Error::Kind::Disconnected);
    EXPECT_EQ(system->log().error().kind, Error::Kind::Disconnected);
}

TEST(Model, KinematicsInSiOnBothSides) {
    MiniController controller;
    const auto robot = connect(controller);
    const auto model = robot.model();
    const ec::Joints near{0.0, -std::numbers::pi / 2, std::numbers::pi / 2, 0.0, 0.0, 0.0};
    const Eigen::Isometry3d target = ec::from_mm_rad({400.0, -50.0, 300.0, 3.0, 0.1, -0.2});

    const auto q = model.ik(target, near);  // the scripted controller answers the reference
    ASSERT_TRUE(q) << describe(q.error());
    for (std::size_t i = 0; i < 6; ++i) EXPECT_NEAR((*q)[i], near[i], 1e-12);
    const auto sent = controller.last("inverseKinematic");
    EXPECT_NEAR(sent["targetPose"][0].get<double>(), 400.0, 1e-9);    // mm
    EXPECT_NEAR(sent["referencePos"][1].get<double>(), -90.0, 1e-9);  // deg
    EXPECT_EQ(sent["unit_type"], 1);

    const auto T = model.fk(near);
    ASSERT_TRUE(T) << describe(T.error());
    EXPECT_TRUE(T->isApprox(target, 1e-12));
    EXPECT_NEAR(controller.last("positiveKinematic")["targetPos"][2].get<double>(), 90.0, 1e-9);

    controller.answer("convertPoseFromCartToUser", "[100.0,0.0,0.0,0.0,0.0,0.0]");
    const auto in_user = model.to_user(target, 2);
    ASSERT_TRUE(in_user);
    EXPECT_NEAR(in_user->translation().x(), 0.1, 1e-12);
    EXPECT_EQ(controller.last("convertPoseFromCartToUser")["userNo"], 2);
}

TEST(Model, ToolsPayloadsAndUserFrames) {
    MiniController controller;
    const auto robot = connect(controller);
    const auto model = robot.model();
    controller.answer("getAutoRunToolNumber", "3");
    controller.answer("getTcpPos", "[0.0,0.0,120.0,0.0,0.0,0.0]");
    controller.answer("get_tool_payload", R"({"m":2.5,"tool_cog":[0.0,10.0,50.0]})");
    controller.answer("getUserFrame", "[500.0,0.0,0.0,0.0,0.0,1.5]");

    EXPECT_EQ(model.active_tool(), 3);
    EXPECT_TRUE(model.select_tool(1));
    EXPECT_EQ(controller.last("setAutoRunToolNumber")["tool_num"], 1);  // the REMOTE tool, not TEACH's

    const auto tcp = model.tool(1);
    ASSERT_TRUE(tcp);
    EXPECT_NEAR(tcp->translation().z(), 0.120, 1e-12);
    Eigen::Isometry3d T_flange_tcp = Eigen::Isometry3d::Identity();
    T_flange_tcp.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);
    EXPECT_TRUE(model.set_tool(1, T_flange_tcp));
    EXPECT_NEAR(controller.last("cmd_set_tcp")["point"][2].get<double>(), 200.0, 1e-9);

    const auto payload = model.payload(1);
    ASSERT_TRUE(payload) << describe(payload.error());
    EXPECT_DOUBLE_EQ(payload->mass, 2.5);
    EXPECT_NEAR(payload->centre_of_mass.z(), 0.050, 1e-12);
    EXPECT_TRUE(model.set_payload(1, {.mass = 6.0, .centre_of_mass = Eigen::Vector3d(0.0, 0.0, 0.1)}));
    EXPECT_EQ(controller.last("cmd_set_payload")["m"], 6.0);
    EXPECT_NEAR(controller.last("cmd_set_payload")["cog"][2].get<double>(), 100.0, 1e-9);

    const auto frame = model.user_frame(4);
    ASSERT_TRUE(frame);
    EXPECT_NEAR(frame->translation().x(), 0.5, 1e-12);
    EXPECT_TRUE(model.select_user_frame(4));
    EXPECT_EQ(controller.last("setUserNumber")["user_num"], 4);
}

TEST(Model, OutOfTheManualsRangesNothingIsSent) {
    MiniController controller;
    const auto robot = connect(controller);
    const auto model = robot.model();
    controller.forget();
    Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
    far.translation() = Eigen::Vector3d(0.0, 0.0, 0.6);
    const auto kind = [](const auto& r) { return r ? std::optional<Error::Kind>{} : r.error().kind; };
    EXPECT_EQ(kind(model.tool(8)), Error::Kind::Invalid);
    EXPECT_EQ(kind(model.select_tool(-1)), Error::Kind::Invalid);
    EXPECT_EQ(kind(model.set_tool(0, far)), Error::Kind::Invalid);
    EXPECT_EQ(kind(model.set_payload(0, {.mass = 7.5})), Error::Kind::Invalid);
    EXPECT_EQ(kind(model.set_payload(0, {.mass = 1.0, .centre_of_mass = Eigen::Vector3d(6.0, 0.0, 0.0)})),
              Error::Kind::Invalid);
    EXPECT_EQ(kind(model.user_frame(9)), Error::Kind::Invalid);
    EXPECT_EQ(kind(model.to_user(far, 8)), Error::Kind::Invalid);
    EXPECT_TRUE(controller.calls().empty());
}

TEST(System, ModeVersionAlarmsClock) {
    MiniController controller;
    const auto robot = connect(controller);
    const auto system = robot.system();
    controller.answer("getSoftVersion", R"("V3.15.3")");
    controller.answer("getAlarmNum", "[1030, 1800, 0, 0, 0]");

    EXPECT_EQ(system.mode(), ec::RobotMode::Remote);
    EXPECT_EQ(system.version(), "V3.15.3");
    EXPECT_EQ(system.alarms(), (std::vector<std::int64_t>{1030, 1800, 0, 0, 0}));

    using namespace std::chrono;
    const sys_seconds t = sys_days{2026y / October / 25} + 12h + 34min + 56s;
    EXPECT_TRUE(system.set_clock(t));
    EXPECT_EQ(controller.last("set_system_time")["time"], ec::sdk::Json::array({2026, 10, 25, 12, 34, 56}));
    EXPECT_EQ(system.set_clock(sys_days{2040y / January / 1}).error().kind, Error::Kind::Invalid);
}

TEST(System, TheLogIsReadFromTheLogPort) {
    MiniController controller;
    const auto robot = connect(controller);
    controller.log_lines({"2026-10-25 12:00:00 Info: connected", "2026-10-25 12:00:01 Error: 0-1030-ca"});
    const auto lines = robot.system().log({.level = ec::log::Level::Error, .lines = 2});
    ASSERT_TRUE(lines) << describe(lines.error());
    EXPECT_EQ(*lines, (std::vector<std::string>{"2026-10-25 12:00:00 Info: connected",
                                                "2026-10-25 12:00:01 Error: 0-1030-ca"}));
    // The scripted port records a request once it has seen its "exit".
    for (int i = 0; i < 200 && controller.log_requests().empty(); ++i) std::this_thread::sleep_for(10ms);
    EXPECT_EQ(controller.log_requests(), (std::vector<std::string>{"Type=Error|2|exit"}));
    EXPECT_TRUE(controller.calls().empty());  // nothing on the command connection
}

}  // namespace
