#pragma once
/// @file
/// Parameter service (ParamService), SDK manual section 2.2.2. Values are in the
/// controller's units, as in the manual; commands with a `unit_type` parameter
/// always send 1, so their angles are radians. "8056: x" names the monitoring
/// field that carries the same value: prefer the stream, and use the call to verify.
/// Replies marked "shape unverified" follow the manual's wording and await a
/// check on the robot.

#include <array>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ec/sdk/error.hpp"
#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/request.hpp"
#include "ec/status.hpp"

namespace ec::sdk {

/// The coordinate system jogging and Cartesian commands refer to (§2.2.2.6, §2.2.2.16).
enum class CoordinateSystem : std::uint8_t { Joint = 0, Base = 1, Tool = 2, User = 3, Cylindrical = 4 };
[[nodiscard]] constexpr bool is_valid(CoordinateSystem c) noexcept { return static_cast<int>(c) <= 4; }

/// What an end-effector button does (§2.2.2.51, §2.2.2.52).
enum class FlangeButtonFunction : std::uint8_t { Disabled = 0, Drag = 1, RecordPoint = 2 };
[[nodiscard]] constexpr bool is_valid(FlangeButtonFunction f) noexcept { return static_cast<int>(f) <= 2; }

/// The end-effector buttons (§2.2.2.51, §2.2.2.52).
enum class FlangeButton : std::uint8_t { Blue = 0, Green = 1 };

/// A tool's payload (§2.2.2.62).
struct ToolPayload {
    double mass_kg = 0.0;            ///< m
    std::array<double, 3> cog_mm{};  ///< tool_cog: centre of mass x, y, z
    bool operator==(const ToolPayload&) const = default;
};

/// §2.2.2.1 Robot state: stopped, paused, emergency stop, running, alarm or
/// collision (8056: robotState). An emergency stop shows here only briefly
/// before the alarm state replaces it: use GetEstopStatus for the E-stop itself.
struct GetRobotState {
    static constexpr std::string_view method = "getRobotState";
    static constexpr Kind kind = Kind::Query;
    using Reply = RobotState;
};

/// §2.2.2.2 Robot mode: teach, play or remote (8056: robotMode).
struct GetRobotMode {
    static constexpr std::string_view method = "getRobotMode";
    static constexpr Kind kind = Kind::Query;
    using Reply = RobotMode;
};

/// §2.2.2.3 Joint positions on the output side (after the gearbox) [deg] (8056: machinePos).
struct GetJointPos {
    static constexpr std::string_view method = "get_joint_pos";
    static constexpr Kind kind = Kind::Query;
    using Reply = JointsDeg;
};

/// §2.2.2.4 The TCP pose of a tool in a frame (8056: machinePose for the current
/// tool in the base frame).
struct GetTcpPose {
    static constexpr std::string_view method = "get_tcp_pose";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;
    int user_frame = -1;  ///< coordinate_num: -1 base frame, 0..7 a user frame
    int tool = -1;        ///< tool_num: -1 the current tool, 0..7 a tool

    [[nodiscard]] Json params() const {
        return Json{{"coordinate_num", user_frame}, {"tool_num", tool}, {"unit_type", 1}};
    }
};

/// §2.2.2.5 Motor speeds; the manual gives no unit (8056: motor_speed, in rpm).
struct GetMotorSpeed {
    static constexpr std::string_view method = "get_motor_speed";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.6 The coordinate system currently selected for jogging.
struct GetCurrentCoord {
    static constexpr std::string_view method = "getCurrentCoord";
    static constexpr Kind kind = Kind::Query;
    using Reply = CoordinateSystem;
};

/// §2.2.2.7 Cycle mode: single step, single cycle or continuous (8056: autorun_cycelMode).
struct GetCycleMode {
    static constexpr std::string_view method = "getCycleMode";
    static constexpr Kind kind = Kind::Query;
    using Reply = CycleMode;
};

/// §2.2.2.8 The line of the running program; it counts the lines holding point
/// data too, so it differs from the pendant's line numbers.
struct GetCurrentJobLine {
    static constexpr std::string_view method = "getCurrentJobLine";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.9 Encoder values of the six joints.
struct GetCurrentEncode {
    static constexpr std::string_view method = "getCurrentEncode";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.10 The tool number in teach mode, 0..7.
struct GetToolNumber {
    static constexpr std::string_view method = "getToolNumber";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.11 Selects the teach-mode tool. Remote mode only.
struct SetToolNumber {
    static constexpr std::string_view method = "setToolNumber";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int tool = 0;  ///< tool_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}}; }
};

/// §2.2.2.12 The current user frame number, 0..7.
struct GetUserNumber {
    static constexpr std::string_view method = "getUserNumber";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.13 Selects the current user frame. Remote mode only.
struct SetUserNumber {
    static constexpr std::string_view method = "setUserNumber";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int user_frame = 0;  ///< user_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"user_num", user_frame}}; }
};

/// §2.2.2.14 Joint torques as per mille of the rated torque (8056: torque).
struct GetMotorTorque {
    static constexpr std::string_view method = "get_motor_torque";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.15 The index of the waypoint being executed; -1 outside waypoint motion.
struct GetPathPointIndex {
    static constexpr std::string_view method = "getPathPointIndex";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.16 Selects the coordinate system for jogging. Remote mode only.
struct SetCurrentCoord {
    static constexpr std::string_view method = "setCurrentCoord";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    CoordinateSystem system = CoordinateSystem::Joint;  ///< coord_mode

    [[nodiscard]] Json params() const { return Json{{"coord_mode", static_cast<int>(system)}}; }
};

/// §2.2.2.17 Switches hand guiding (drag teaching) on or off. Remote mode only.
struct DragTeachSwitch {
    static constexpr std::string_view method = "drag_teach_switch";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
    bool on = false;  ///< switch: 1 on, 0 off

    [[nodiscard]] Json params() const { return Json{{"switch", on ? 1 : 0}}; }
};

/// §2.2.2.18 Whether hand guiding is enabled (8056: dragState).
struct GetDragState {
    static constexpr std::string_view method = "get_drag_state";
    static constexpr Kind kind = Kind::Query;
    using Reply = bool;
};

/// §2.2.2.19 Sets a tool's payload mass and centre of gravity. Remote mode only.
struct CmdSetPayload {
    static constexpr std::string_view method = "cmd_set_payload";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int tool = 0;                    ///< tool_num, 0..7
    double mass_kg = 0.0;            ///< m; EA66: 0..7.2
    std::array<double, 3> cog_mm{};  ///< cog: x, y, z, each -5000..5000

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}, {"m", mass_kg}, {"cog", cog_mm}}; }
};

/// §2.2.2.20 Sets a tool's centre point. Remote mode only.
struct CmdSetTcp {
    static constexpr std::string_view method = "cmd_set_tcp";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int tool = 0;       ///< tool_num, 0..7
    PoseMmRad point{};  ///< point: x, y, z within -500..500 mm

    [[nodiscard]] Json params() const { return Json{{"point", point}, {"tool_num", tool}, {"unit_type", 1}}; }
};

/// §2.2.2.21 Whether a collision has been detected (8056: collision).
struct GetCollisionState {
    static constexpr std::string_view method = "getCollisionState";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.2.22 A user frame's pose in the base frame.
struct GetUserFrame {
    static constexpr std::string_view method = "getUserFrame";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;
    int user_frame = 0;  ///< user_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"user_num", user_frame}, {"unit_type", 1}}; }
};

/// §2.2.2.23 Sets the cycle mode. Remote mode only.
struct SetCycleMode {
    static constexpr std::string_view method = "setCycleMode";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    CycleMode mode = CycleMode::Step;  ///< cycle_mode

    [[nodiscard]] Json params() const { return Json{{"cycle_mode", static_cast<int>(mode)}}; }
};

/// §2.2.2.24 Defines a user frame. Remote mode only.
struct SetUserFrame {
    static constexpr std::string_view method = "setUserFrame";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int user_frame = 0;  ///< user_num, 0..7
    PoseMmRad frame{};   ///< user_frame, in the base frame

    [[nodiscard]] Json params() const {
        return Json{{"user_num", user_frame}, {"user_frame", frame}, {"unit_type", 1}};
    }
};

/// §2.2.2.25 A tool frame's data (its TCP relative to the flange).
struct GetTcpPos {
    static constexpr std::string_view method = "getTcpPos";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;
    int tool = 0;  ///< tool_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}, {"unit_type", 1}}; }
};

/// §2.2.2.26 A tool's payload mass [kg]. Deprecated: use GetToolPayload.
struct GetPayload {
    static constexpr std::string_view method = "getPayload";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
    int tool = 0;  ///< tool_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}}; }
};

/// §2.2.2.27 A tool's payload centre of mass [mm]; shape unverified.
/// Deprecated: use GetToolPayload.
struct GetCentreMass {
    static constexpr std::string_view method = "getCentreMass";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 3>;
    int tool = 0;  ///< tool_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}}; }
};

/// §2.2.2.28 Robot type: 62 six-axis collaborative, 60 vertical articulated,
/// 41 four-axis, 40 palletizing, 43 SCARA, 30 delta.
struct GetRobotType {
    static constexpr std::string_view method = "getRobotType";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.29 One DH link parameter. The manual gives no unit; the EA66 returns
/// 95.431 for index 0, which reads as millimetres.
struct GetDH {
    static constexpr std::string_view method = "getDH";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
    int index = 0;  ///< 0..11 for d1..d12; the controller rejects others (Kind::Rejected)

    [[nodiscard]] Json params() const { return Json{{"index", index}}; }
};

/// §2.2.2.30 Switches collision detection. Deprecated: use SetSafetyParams.
struct SetCollisionEnable {
    static constexpr std::string_view method = "setCollisionEnable";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    bool on = true;  ///< enable: 1 on, 0 off

    [[nodiscard]] Json params() const { return Json{{"enable", on ? 1 : 0}}; }
};

/// §2.2.2.31 Sets the collision sensitivity: a lower value triggers at a smaller
/// force (EC66 pendant). Remote mode only. Deprecated: use SetSafetyParams.
struct SetCollisionSensitivity {
    static constexpr std::string_view method = "setCollisionSensitivity";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int value_pct = 100;  ///< value, 10..100

    [[nodiscard]] Json params() const { return Json{{"value", value_pct}}; }
};

/// §2.2.2.32 The controller-generated string that SetSafetyParams' password
/// is derived from, when a remote-mode password is set.
struct GetRemoteSysPassword {
    static constexpr std::string_view method = "get_remote_sys_password";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::string;
};

/// §2.2.2.33 Sets the safety limits of one mode, and optionally collision
/// detection. Remote and play mode. The limits have no defaults on purpose: one
/// left out is 0, outside its range, so the controller rejects the whole call
/// and nothing changes unless every limit is given.
struct SetSafetyParams {
    static constexpr std::string_view method = "setSafetyParams";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    /// "123456" if no remote-mode password is set in the UI; otherwise the MD5 of
    /// GetRemoteSysPassword's string followed by that password.
    std::string password{};
    bool enabled = true;                             ///< enable: whether the limits apply
    bool reduced_mode = false;                       ///< mode: the limits of 0 normal, 1 reduced mode
    double power_w;                                  ///< power, 80..1500
    double momentum_kg_m_s;                          ///< momentum, 5..90
    double tool_force_n;                             ///< tool_force, 100..400
    double elbow_force_n;                            ///< elbow_force, 100..400
    double speed_pct;                                ///< speed, 0..100
    std::optional<bool> collision_detection{};       ///< collision_enable
    std::optional<int> collision_sensitivity_pct{};  ///< collision_sensitivity, 10..100; lower = more sensitive

    [[nodiscard]] Json params() const {
        Json p{{"password", password},         {"enable", enabled ? 1 : 0},
               {"mode", reduced_mode ? 1 : 0}, {"power", power_w},
               {"momentum", momentum_kg_m_s},  {"tool_force", tool_force_n},
               {"elbow_force", elbow_force_n}, {"speed", speed_pct}};
        if (collision_detection) p["collision_enable"] = *collision_detection ? 1 : 0;
        if (collision_sensitivity_pct) p["collision_sensitivity"] = *collision_sensitivity_pct;
        return p;
    }
};

/// §2.2.2.34 The play/remote speed setting [%].
struct GetSpeed {
    static constexpr std::string_view method = "getSpeed";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
};

/// §2.2.2.35 Clears the collision state. Remote mode only.
struct ResetCollisionState {
    static constexpr std::string_view method = "resetCollisionState";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.2.36 The tool number in remote (and play) mode, 0..7.
struct GetAutoRunToolNumber {
    static constexpr std::string_view method = "getAutoRunToolNumber";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.37 Selects the remote-mode tool. Remote mode only.
struct SetAutoRunToolNumber {
    static constexpr std::string_view method = "setAutoRunToolNumber";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int tool = 0;  ///< tool_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}}; }
};

/// §2.2.2.38 The flange centre's pose in the base frame (8056: machineFlangePose).
struct GetBaseFlangePose {
    static constexpr std::string_view method = "get_base_flange_pose";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;

    [[nodiscard]] Json params() const { return Json{{"unit_type", 1}}; }
};

/// §2.2.2.39 The flange centre's pose in the current user frame (8056: machineUserFlangePose).
struct GetUserFlangePose {
    static constexpr std::string_view method = "get_user_flange_pose";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;

    [[nodiscard]] Json params() const { return Json{{"unit_type", 1}}; }
};

/// §2.2.2.40 Robot subtype.
struct GetRobotSubtype {
    static constexpr std::string_view method = "getRobotSubtype";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.41 Whether the safety limits are enabled.
struct GetRobotSafetyParamsEnabled {
    static constexpr std::string_view method = "getRobotSafetyParamsEnabled";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.2.42 Power limits [W]: normal mode, then reduced mode.
struct GetRobotSafetyPower {
    static constexpr std::string_view method = "getRobotSafetyPower";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 2>;
};

/// §2.2.2.43 Momentum limits [kg m/s]: normal mode, then reduced mode.
struct GetRobotSafetyMomentum {
    static constexpr std::string_view method = "getRobotSafetyMomentum";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 2>;
};

/// §2.2.2.44 Tool force limits [N]: normal mode, then reduced mode.
struct GetRobotSafetyToolForce {
    static constexpr std::string_view method = "getRobotSafetyToolForce";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 2>;
};

/// §2.2.2.45 Estimated external force [N] and torque [N m] at the TCP
/// (8056: exTcpForceInTool, in the tool frame).
struct GetTcpForce {
    static constexpr std::string_view method = "get_tcp_force";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
    bool in_tool_frame = true;  ///< ref_tcp: 1 the TCP frame, 0 the base frame

    [[nodiscard]] Json params() const { return Json{{"ref_tcp", in_tool_frame ? 1 : 0}}; }
};

/// §2.2.2.46 Joint torques [N m] (8056: extJointTorques holds the external part).
struct GetJointTorques {
    static constexpr std::string_view method = "get_joint_torques";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.47 Elbow force limits [N]: normal mode, then reduced mode.
struct GetRobotSafetyElbowForce {
    static constexpr std::string_view method = "getRobotSafetyElbowForce";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 2>;
};

/// §2.2.2.48 Speed limits [%]: normal mode, then reduced mode.
struct GetRobotSpeedPercentage {
    static constexpr std::string_view method = "getRobotSpeedPercentage";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 2>;
};

/// §2.2.2.49 The maximum speed at which hand guiding starts; no unit given.
struct GetRobotDragStartupMaxSpeed {
    static constexpr std::string_view method = "getRobotDragStartupMaxSpeed";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
};

/// §2.2.2.50 The maximum torque error percentage of the force-control parameters.
struct GetRobotTorqueErrorMaxPercents {
    static constexpr std::string_view method = "getRobotTorqueErrorMaxPercents";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
};

/// §2.2.2.51 Sets what an end-effector button does. Remote mode only.
struct SetFlangeButton {
    static constexpr std::string_view method = "setFlangeButton";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    FlangeButton button = FlangeButton::Blue;                        ///< button_num
    FlangeButtonFunction function = FlangeButtonFunction::Disabled;  ///< state

    [[nodiscard]] Json params() const {
        return Json{{"button_num", static_cast<int>(button)}, {"state", static_cast<int>(function)}};
    }
};

/// §2.2.2.52 What an end-effector button does.
struct CheckFlangeButton {
    static constexpr std::string_view method = "checkFlangeButton";
    static constexpr Kind kind = Kind::Query;
    using Reply = FlangeButtonFunction;
    FlangeButton button = FlangeButton::Blue;  ///< button_num

    [[nodiscard]] Json params() const { return Json{{"button_num", static_cast<int>(button)}}; }
};

/// §2.2.2.53 Whether collision detection is enabled.
struct GetCollisionEnableStatus {
    static constexpr std::string_view method = "get_collision_enable_status";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.2.54 Collision sensitivity [%]: a lower value triggers at a smaller force (EC66 pendant).
struct GetCollisionSensitivity {
    static constexpr std::string_view method = "getCollisionSensitivity";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.2.55 The TCP pose in the current user frame (8056: machineUserPose).
struct GetTcpPoseInUser {
    static constexpr std::string_view method = "getTcpPoseInUser";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;

    [[nodiscard]] Json params() const { return Json{{"unit_type", 1}}; }
};

/// §2.2.2.56 Serial numbers of the five most recent alarms. The controller
/// answers false on failure, which decodes as Kind::Refused. Shape unverified.
struct GetAlarmNum {
    static constexpr std::string_view method = "getAlarmNum";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<std::int64_t>;

    static std::expected<Reply, Error> decode(const Json& r) {
        if (r.is_boolean() && !r.get<bool>())
            return std::unexpected(Error{Error::Kind::Refused, 0, "controller returned false"});
        return ReplyDecoder<Reply>::decode(r);
    }
};

/// §2.2.2.57 Joint speeds [deg/s] (8056: jointSpeed).
struct GetJointSpeed {
    static constexpr std::string_view method = "get_joint_speed";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.58 TCP acceleration [mm/s^2] (8056: tcpAcc).
struct GetTcpAcc {
    static constexpr std::string_view method = "get_tcp_acc";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
};

/// §2.2.2.59 Joint accelerations [deg/s^2] (8056: jointAcc).
struct GetJointAcc {
    static constexpr std::string_view method = "get_joint_acc";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.60 TCP speed [mm/s] (8056: tcpSpeed).
struct GetTcpSpeed {
    static constexpr std::string_view method = "get_tcp_speed";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
};

/// §2.2.2.61 Whether the robot is in emergency stop (8056: emergencyStopState).
struct GetEstopStatus {
    static constexpr std::string_view method = "get_estop_status";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.2.62 A tool's payload mass and centre of mass. The manual names the
/// fields m and tool_cog; decoded from an object with those members, or from
/// [m, [x, y, z]]. Shape unverified.
struct GetToolPayload {
    static constexpr std::string_view method = "get_tool_payload";
    static constexpr Kind kind = Kind::Query;
    using Reply = ToolPayload;
    int tool = 0;  ///< tool_num, 0..7

    [[nodiscard]] Json params() const { return Json{{"tool_num", tool}}; }

    static std::expected<Reply, Error> decode(const Json& r) {
        const Json* m = nullptr;
        const Json* cog = nullptr;
        if (r.is_object() && r.contains("m") && r.contains("tool_cog")) {
            m = &r["m"];
            cog = &r["tool_cog"];
        } else if (r.is_array() && r.size() == 2) {
            m = &r[0];
            cog = &r[1];
        } else {
            return bad_reply(std::format("expected m and tool_cog, got {}", r.dump()));
        }
        const auto mass = ReplyDecoder<double>::decode(*m);
        if (!mass) return std::unexpected(mass.error());
        const auto centre = ReplyDecoder<std::array<double, 3>>::decode(*cog);
        if (!centre) return std::unexpected(centre.error());
        return ToolPayload{*mass, *centre};
    }
};

/// §2.2.2.63 Joint positions on the input (motor) side; units as sent.
struct GetMotorPos {
    static constexpr std::string_view method = "get_motor_pos";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.64 Whether the servo encoders are precise.
struct GetServoPrecisePositionStatus {
    static constexpr std::string_view method = "get_servo_precise_position_status";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.2.65 Whether the servos are in alarm.
struct GetServoAlarmState {
    static constexpr std::string_view method = "get_servo_alarm_state";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.2.66 Clears the queued programs; only while the arm is stopped or in
/// alarm. Remote mode only.
struct BookProgramClear {
    static constexpr std::string_view method = "book_program_clear";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.2.67 The actual TCP pose of a tool in a frame. The manual gives no
/// units; assumed those of GetTcpPose.
struct GetActualTcp {
    static constexpr std::string_view method = "get_actual_tcp";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;
    std::optional<int> tool{};        ///< tool_num, 0..7; none: the current tool
    std::optional<int> user_frame{};  ///< user_num, 0..7; none: the base frame

    [[nodiscard]] Json params() const {
        Json p = Json::object();
        if (tool) p["tool_num"] = *tool;
        if (user_frame) p["user_num"] = *user_frame;
        return p;
    }
};

/// §2.2.2.68 The target (interpolated) TCP pose of a tool in a frame; units as GetActualTcp.
struct GetTargetTcp {
    static constexpr std::string_view method = "get_target_tcp";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;
    std::optional<int> tool{};        ///< tool_num, 0..7; none: the current tool
    std::optional<int> user_frame{};  ///< user_num, 0..7; none: the base frame

    [[nodiscard]] Json params() const {
        Json p = Json::object();
        if (tool) p["tool_num"] = *tool;
        if (user_frame) p["user_num"] = *user_frame;
        return p;
    }
};

/// §2.2.2.69 Actual joint positions [deg] (8056: machinePos).
struct GetActualJoint {
    static constexpr std::string_view method = "get_actual_joint";
    static constexpr Kind kind = Kind::Query;
    using Reply = JointsDeg;
};

/// §2.2.2.70 Target (interpolated) joint positions [deg]: what the controller
/// is commanding right now.
struct GetTargetJoint {
    static constexpr std::string_view method = "get_target_joint";
    static constexpr Kind kind = Kind::Query;
    using Reply = JointsDeg;
};

/// §2.2.2.71 The pose a fraction of the way from one pose to another (a computation).
struct GetInterpPose {
    static constexpr std::string_view method = "get_interp_pose";
    static constexpr Kind kind = Kind::Compute;
    using Reply = PoseMmRad;
    PoseMmRad from{};    ///< data1
    PoseMmRad to{};      ///< data2
    double ratio = 0.0;  ///< 0: `from`, 1: `to`

    [[nodiscard]] Json params() const { return Json{{"data1", from}, {"data2", to}, {"ratio", ratio}}; }
};

/// §2.2.2.72 Joint temperatures [degC] (8056: jointTemperature).
struct GetJointTemp {
    static constexpr std::string_view method = "get_joint_temp";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

/// §2.2.2.73 Every inverse-kinematics solution for a pose. The manual's
/// description of the reply contradicts itself, so it comes back as JSON.
struct InverseKinematicAll {
    static constexpr std::string_view method = "inverseKinematicAll";
    static constexpr Kind kind = Kind::Compute;
    using Reply = Json;
    PoseMmRad target{};  ///< targetPose

    [[nodiscard]] Json params() const { return Json{{"targetPose", target}}; }
};

/// §2.2.2.74 Sets the controller's clock (1970..2037). Firmware 3.10.2 or later.
struct SetSystemTime {
    static constexpr std::string_view method = "set_system_time";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    std::array<int, 6> time{};  ///< year, month, day, hour, minute, second

    [[nodiscard]] Json params() const { return Json{{"time", time}}; }
};

static_assert(Command<GetRobotState>);
static_assert(Command<GetRobotMode>);
static_assert(Command<GetJointPos>);
static_assert(Command<GetTcpPose>);
static_assert(Command<GetMotorSpeed>);
static_assert(Command<GetCurrentCoord>);
static_assert(Command<GetCycleMode>);
static_assert(Command<GetCurrentJobLine>);
static_assert(Command<GetCurrentEncode>);
static_assert(Command<GetToolNumber>);
static_assert(Command<SetToolNumber>);
static_assert(Command<GetUserNumber>);
static_assert(Command<SetUserNumber>);
static_assert(Command<GetMotorTorque>);
static_assert(Command<GetPathPointIndex>);
static_assert(Command<SetCurrentCoord>);
static_assert(Command<DragTeachSwitch>);
static_assert(Command<GetDragState>);
static_assert(Command<CmdSetPayload>);
static_assert(Command<CmdSetTcp>);
static_assert(Command<GetCollisionState>);
static_assert(Command<GetUserFrame>);
static_assert(Command<SetCycleMode>);
static_assert(Command<SetUserFrame>);
static_assert(Command<GetTcpPos>);
static_assert(Command<GetPayload>);
static_assert(Command<GetCentreMass>);
static_assert(Command<GetRobotType>);
static_assert(Command<GetDH>);
static_assert(Command<SetCollisionEnable>);
static_assert(Command<SetCollisionSensitivity>);
static_assert(Command<GetRemoteSysPassword>);
static_assert(Command<SetSafetyParams>);
static_assert(Command<GetSpeed>);
static_assert(Command<ResetCollisionState>);
static_assert(Command<GetAutoRunToolNumber>);
static_assert(Command<SetAutoRunToolNumber>);
static_assert(Command<GetBaseFlangePose>);
static_assert(Command<GetUserFlangePose>);
static_assert(Command<GetRobotSubtype>);
static_assert(Command<GetRobotSafetyParamsEnabled>);
static_assert(Command<GetRobotSafetyPower>);
static_assert(Command<GetRobotSafetyMomentum>);
static_assert(Command<GetRobotSafetyToolForce>);
static_assert(Command<GetTcpForce>);
static_assert(Command<GetJointTorques>);
static_assert(Command<GetRobotSafetyElbowForce>);
static_assert(Command<GetRobotSpeedPercentage>);
static_assert(Command<GetRobotDragStartupMaxSpeed>);
static_assert(Command<GetRobotTorqueErrorMaxPercents>);
static_assert(Command<SetFlangeButton>);
static_assert(Command<CheckFlangeButton>);
static_assert(Command<GetCollisionEnableStatus>);
static_assert(Command<GetCollisionSensitivity>);
static_assert(Command<GetTcpPoseInUser>);
static_assert(Command<GetAlarmNum>);
static_assert(Command<GetJointSpeed>);
static_assert(Command<GetTcpAcc>);
static_assert(Command<GetJointAcc>);
static_assert(Command<GetTcpSpeed>);
static_assert(Command<GetEstopStatus>);
static_assert(Command<GetToolPayload>);
static_assert(Command<GetMotorPos>);
static_assert(Command<GetServoPrecisePositionStatus>);
static_assert(Command<GetServoAlarmState>);
static_assert(Command<BookProgramClear>);
static_assert(Command<GetActualTcp>);
static_assert(Command<GetTargetTcp>);
static_assert(Command<GetActualJoint>);
static_assert(Command<GetTargetJoint>);
static_assert(Command<GetInterpPose>);
static_assert(Command<GetJointTemp>);
static_assert(Command<InverseKinematicAll>);
static_assert(Command<SetSystemTime>);

}  // namespace ec::sdk
