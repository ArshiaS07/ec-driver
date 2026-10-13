#pragma once
/// @file
/// Every typed SDK call: a compile-time list, for generic code and tests, and a
/// run-time table of names and kinds, for tools.

#include <cstddef>
#include <expected>
#include <span>
#include <string_view>

#include "ec/command/backdrive.hpp"
#include "ec/command/error.hpp"
#include "ec/command/ethernet_ip.hpp"
#include "ec/command/force_control.hpp"
#include "ec/command/force_sensor.hpp"
#include "ec/command/io.hpp"
#include "ec/command/json.hpp"
#include "ec/command/kinematics.hpp"
#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "ec/command/profinet.hpp"
#include "ec/command/request.hpp"
#include "ec/command/serial.hpp"
#include "ec/command/servo.hpp"
#include "ec/command/system.hpp"
#include "ec/command/trajectory.hpp"
#include "ec/command/transparent.hpp"
#include "ec/command/variables.hpp"

namespace ec::command {

/// A list of command types.
template <Command... Cs>
struct CommandList {
    static constexpr std::size_t size = sizeof...(Cs);

    /// Calls `f.template operator()<C>()` for every command C, in order.
    template <typename F>
    static constexpr void for_each(F&& f) {
        (f.template operator()<Cs>(), ...);
    }
};

/// Every typed call, in the manual's order.
using AllCommands = CommandList<
    // 2.2.1 servo service
    GetServoStatus, SetServoStatus, SyncMotorStatus, ClearAlarm, GetMotorStatus,
    // 2.2.2 parameter service
    GetRobotState, GetRobotMode, GetJointPos, GetTcpPose, GetMotorSpeed, GetCurrentCoord, GetCycleMode,
    GetCurrentJobLine, GetCurrentEncode, GetToolNumber, SetToolNumber, GetUserNumber, SetUserNumber, GetMotorTorque,
    GetPathPointIndex, SetCurrentCoord, DragTeachSwitch, GetDragState, CmdSetPayload, CmdSetTcp, GetCollisionState,
    GetUserFrame, SetCycleMode, SetUserFrame, GetTcpPos, GetPayload, GetCentreMass, GetRobotType, GetDH,
    SetCollisionEnable, SetCollisionSensitivity, GetRemoteSysPassword, SetSafetyParams, GetSpeed, ResetCollisionState,
    GetAutoRunToolNumber, SetAutoRunToolNumber, GetBaseFlangePose, GetUserFlangePose, GetRobotSubtype,
    GetRobotSafetyParamsEnabled, GetRobotSafetyPower, GetRobotSafetyMomentum, GetRobotSafetyToolForce, GetTcpForce,
    GetJointTorques, GetRobotSafetyElbowForce, GetRobotSpeedPercentage, GetRobotDragStartupMaxSpeed,
    GetRobotTorqueErrorMaxPercents, SetFlangeButton, CheckFlangeButton, GetCollisionEnableStatus,
    GetCollisionSensitivity, GetTcpPoseInUser, GetAlarmNum, GetJointSpeed, GetTcpAcc, GetJointAcc, GetTcpSpeed,
    GetEstopStatus, GetToolPayload, GetMotorPos, GetServoPrecisePositionStatus, GetServoAlarmState, BookProgramClear,
    GetActualTcp, GetTargetTcp, GetActualJoint, GetTargetJoint, GetInterpPose, GetJointTemp, InverseKinematicAll,
    SetSystemTime,
    // 2.2.3 movement service
    MoveByJoint, MoveByLine, MoveByArc, MoveByRotate, AddPathPoint, ClearPathPoint, MoveByPath, Jog, Stop, Run, Pause,
    CheckJbiExist, RunJbi, GetJbiState, SetSpeed, MoveBySpeedj, Stopj, MoveBySpeedl, Stopl, MoveByLineCoord,
    CalibrateEncoderZeroPosition,
    // 2.2.4 kinematics service
    InverseKinematic, PositiveKinematic, ConvertPoseFromCartToUser, ConvertPoseFromUserToCart, PoseMul, PoseInv,
    // 2.2.5 IO service
    GetInput, GetOutput, SetOutput, GetVirtualInput, GetVirtualOutput, SetVirtualOutput, GetRegisters, GetAnalogInput,
    GetAnalogOutput, SetAnalogOutput,
    // 2.2.6 variable service
    GetSysVarB, SetSysVarB, GetSysVarI, SetSysVarI, GetSysVarD, SetSysVarD, GetSysVarPState, GetSysVarP, SetSysVarP,
    GetSysVarV, SetSysVarV, SaveVarData,
    // 2.2.7 transparent transmission
    TransparentTransmissionInit, TtSetCurrentServoJoint, GetTransparentTransmissionState, TtPutServoJointToBuf,
    TtClearServoJointBuf,
    // 2.2.8 system service
    GetSoftVersion, GetJointVersion,
    // 2.2.9 timestamp service
    StartPushPos, PushPos, StopPushPos, CheckTrajectory, StartTrajectory, PauseTrajectory, ResumeTrajectory,
    StopTrajectory, FlushTrajectory,
    // 2.2.10 Profinet
    GetProfinetIntInputRegisters, GetProfinetIntOutputRegisters, GetProfinetFloatInputRegisters,
    GetProfinetFloatOutputRegisters, SetProfinetIntOutputRegisters, SetProfinetFloatOutputRegisters,
    // 2.2.11 backdrive service
    GetServoBrakeOffStatus, GetBackdriveStatus, EnterBackdrive, ExitBackdrive, ResetRobotStatus,
    // 2.2.12 EtherNet/IP
    GetEipIntInputRegisters, GetEipIntOutputRegisters, GetEipFloatInputRegisters, GetEipFloatOutputRegisters,
    SetEipIntOutputRegisters, SetEipFloatOutputRegisters,
    // 2.2.13 external force sensor
    StartPushForce, PushExternalForce, StopPushForce, GetForceCtrlMode, GetTcpForceBySensor,
    // 2.2.14 and 2.2.15 serial ports
    OpenSerialPort, SetoptSerialPort, RecvSerialPort, SendSerialPort, FlushSerialPort, CloseSerialPort, OpenTci,
    SetoptTci, RecvTci, SendTci, FlushTci, CloseTci,
    // 2.2.16 force control
    StartForceMode, EndForceMode, GetForceModeState, ZeroForceSensor, FcAct, FcDeact>;

/// One typed call, for tools that work by name.
struct CommandInfo {
    std::string_view method;
    Kind kind;
    /// Decodes an (unwrapped) result as this call's reply type: nothing if it
    /// fits, otherwise the decoder's error.
    std::expected<void, Error> (*check_reply)(const Json& result);
    /// The request of the command with its default members: for queries, a
    /// valid call (e.g. tool 0, address 0) to try the reply's type with.
    Request (*example)();
};

/// Every typed call, in the manual's order.
[[nodiscard]] std::span<const CommandInfo> all_commands() noexcept;

/// The typed call named `method`, or nullptr.
[[nodiscard]] const CommandInfo* find_command(std::string_view method) noexcept;

}  // namespace ec::command
