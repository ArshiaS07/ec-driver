#pragma once
/// @file
/// One port-8056 monitoring frame (SDK manual section 3.1), in the controller's
/// units, and its decoder.

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string_view>

#include "ec/status.hpp"

namespace ec::feedback {

/// Wire layout, as in the manual's table: big-endian, tightly packed; bytes
/// [0, 801) hold the 30 documented fields, [801, 1020) are reserved, and
/// [1020, 1024) hold the matching word.
inline constexpr std::size_t kFrameSize = 1024;
inline constexpr std::size_t kValidBytes = 801;
inline constexpr std::size_t kMatchOffset = 1020;
inline constexpr std::uint32_t kMatchingWord = 0xEC8056ECu;
inline constexpr std::size_t kWireAxes = 8;  ///< AXIS_COUNT: joint arrays on the wire; [0..5] are the arm
inline constexpr std::size_t kArmAxes = 6;

using RawFrame = std::array<std::uint8_t, kFrameSize>;

using WireJoints = std::array<double, kWireAxes>;  ///< [0..5] are the arm
using ArmJoints = std::array<double, kArmAxes>;
using WirePose = std::array<double, 6>;    ///< x, y, z [mm]; rx, ry, rz [rad]
using WireWrench = std::array<double, 6>;  ///< fx, fy, fz [N]; tx, ty, tz [N*m]

/// A decoded frame, mirroring the wire so it can be checked against the manual
/// and the teach pendant. Units are the controller's and part of each member's
/// name; each comment names the manual's field.
///
/// Status fields keep whatever the controller sent, including values the manual
/// does not define: check them with is_valid() before relying on them.
struct Frame {
    // time
    std::uint32_t message_size = 0;  ///< Message Size: bytes, 1024 in every valid frame
    std::uint64_t timestamp_ms = 0;  ///< timestamp: controller clock, ms since 1970-01-01

    // joint space
    WireJoints joint_pos_deg{};                             ///< machinePos
    WireJoints joint_vel_deg_s{};                           ///< jointSpeed
    WireJoints joint_acc_deg_s2{};                          ///< jointAcc
    WireJoints joint_torque_permille{};                     ///< torque: per mille of rated torque
    std::array<std::int32_t, kWireAxes> motor_speed_rpm{};  ///< motor_speed
    ArmJoints joint_torque_nm{};                            ///< jointTorque: actual output torque
    ArmJoints joint_torque_ext_nm{};                        ///< extJointTorques: estimated external torque
    ArmJoints joint_temp_c{};                               ///< jointTemperature

    // Cartesian space
    WirePose tcp_pose_base{};     ///< machinePose
    WirePose tcp_pose_user{};     ///< machineUserPose
    WirePose flange_pose_base{};  ///< machineFlangePose
    WirePose flange_pose_user{};  ///< machineUserFlangePose
    double tcp_speed_mm_s = 0.0;  ///< tcpSpeed
    double tcp_acc_mm_s2 = 0.0;   ///< tcpAcc
    WireWrench tcp_wrench_ext{};  ///< exTcpForceInTool: estimated external wrench, current tool frame

    // status
    RobotState robot_state = RobotState::Stop;  ///< robotState
    RobotMode robot_mode = RobotMode::Teach;    ///< robotMode
    CycleMode cycle_mode = CycleMode::Step;     ///< autorun_cycelMode
    bool servo_ready = false;                   ///< servoReady: servo on, the brakes released
    bool motors_synced = false;                 ///< can_motor_run: encoders synchronized
    bool collision = false;                     ///< collision: collision alarm
    bool emergency_stop = false;                ///< emergencyStopState
    bool drag_enabled = false;                  ///< dragState
    bool force_sensor_connected = false;        ///< sensor_connected_state

    // IO
    std::uint64_t digital_in = 0;          ///< digital_ioInput: bit i is DI i
    std::uint64_t digital_out = 0;         ///< digital_ioOutput: bit i is DO i
    std::array<double, 3> analog_in_v{};   ///< analog_ioInput
    std::array<double, 5> analog_out_v{};  ///< analog_ioOutput
};

/// Why a frame could not be decoded.
enum class DecodeError {
    BadMatchingWord,  ///< the last 4 bytes are not 0xEC8056EC
    BadMessageSize,   ///< the size field is not 1024
};

/// Short description of a decode error.
[[nodiscard]] std::string_view to_string(DecodeError e) noexcept;

/// Decodes one aligned frame. The span's static extent makes a buffer of the
/// wrong size a compile error.
[[nodiscard]] std::expected<Frame, DecodeError> decode(std::span<const std::uint8_t, kFrameSize> bytes) noexcept;

/// True if the 4 bytes are the matching word, 0xEC8056EC in big-endian order.
[[nodiscard]] bool is_matching_word(std::span<const std::uint8_t, 4> bytes) noexcept;

}  // namespace ec::feedback
