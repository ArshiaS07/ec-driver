#pragma once
// One port-8056 monitoring frame (SDK manual, section 3.1), in wire units.
//
// Wire layout (checked against the manual's table):
//   * 1024 bytes per frame, big-endian, tightly packed (no padding)
//   * bytes [0, 801)     : 30 documented fields
//   * bytes [801, 1020)  : 219 reserved bytes
//   * bytes [1020, 1024) : matching word 0xEC8056EC
// Joint arrays are AXIS_COUNT = 8 wide on the wire; [0..5] are the arm.
//
// Units are the controller's (deg, mm, rad, N*m, ...) and part of every member
// name. Conversion to SI happens at one boundary above this layer, never here:
// this struct mirrors the wire so it can be checked against the manual and the
// teach pendant.

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string_view>

#include "ec/status.hpp"

namespace ec::feedback {

inline constexpr std::size_t kFrameSize = 1024;
inline constexpr std::size_t kValidBytes = 801;
inline constexpr std::size_t kMatchOffset = 1020;
inline constexpr std::uint32_t kMatchingWord = 0xEC8056ECu;
inline constexpr std::size_t kWireAxes = 8;  // AXIS_COUNT
inline constexpr std::size_t kArmAxes = 6;

using RawFrame = std::array<std::uint8_t, kFrameSize>;

using WireJoints = std::array<double, kWireAxes>;  // [0..5] are the arm
using ArmJoints = std::array<double, kArmAxes>;
using WirePose = std::array<double, 6>;    // x, y, z [mm]; rx, ry, rz [rad]
using WireWrench = std::array<double, 6>;  // fx, fy, fz [N]; tx, ty, tz [N*m]

struct Frame {
    // time
    std::uint32_t message_size = 0;  // bytes, always 1024 when valid
    std::uint64_t timestamp_ms = 0;  // controller clock, ms since 1970-01-01

    // joint space
    WireJoints joint_pos_deg{};                             // machinePos
    WireJoints joint_vel_deg_s{};                           // jointSpeed
    WireJoints joint_acc_deg_s2{};                          // jointAcc
    WireJoints joint_torque_permille{};                     // torque, per-mille of rated torque
    std::array<std::int32_t, kWireAxes> motor_speed_rpm{};  // motor_speed
    ArmJoints joint_torque_nm{};                            // jointTorque (actual output torque)
    ArmJoints joint_torque_ext_nm{};                        // extJointTorques (estimated external)
    ArmJoints joint_temp_c{};                               // jointTemperature

    // Cartesian space
    WirePose tcp_pose_base{};     // machinePose
    WirePose tcp_pose_user{};     // machineUserPose
    WirePose flange_pose_base{};  // machineFlangePose
    WirePose flange_pose_user{};  // machineUserFlangePose
    double tcp_speed_mm_s = 0.0;  // tcpSpeed
    double tcp_acc_mm_s2 = 0.0;   // tcpAcc
    WireWrench tcp_wrench_ext{};  // exTcpForceInTool (current tool frame)

    // status
    RobotState robot_state = RobotState::Stop;
    RobotMode robot_mode = RobotMode::Teach;
    CycleMode cycle_mode = CycleMode::Step;
    bool servo_ready = false;    // servoReady
    bool motors_synced = false;  // can_motor_run
    bool collision = false;
    bool emergency_stop = false;
    bool drag_enabled = false;
    bool force_sensor_connected = false;

    // IO
    std::uint64_t digital_in = 0;  // bit i = DI i
    std::uint64_t digital_out = 0;
    std::array<double, 3> analog_in_v{};
    std::array<double, 5> analog_out_v{};
};

enum class DecodeError {
    BadMatchingWord,  // last 4 bytes are not 0xEC8056EC
    BadMessageSize,   // header length field is not 1024
};
[[nodiscard]] std::string_view to_string(DecodeError e) noexcept;

// The span's static extent makes a wrong buffer size a compile error.
[[nodiscard]] std::expected<Frame, DecodeError> decode(std::span<const std::uint8_t, kFrameSize> bytes) noexcept;

// True if the 4 bytes are the big-endian matching word. Used for framing.
[[nodiscard]] bool is_matching_word(std::span<const std::uint8_t, 4> bytes) noexcept;

}  // namespace ec::feedback
