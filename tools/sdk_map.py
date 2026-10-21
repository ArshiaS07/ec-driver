"""Writes docs/sdk_map.md: the typed SDK calls grouped by robot part. Run from the repository root."""
import re, glob, collections
rows = {}
for f in sorted(glob.glob('include/ec/sdk/*.hpp')):
    s = open(f).read()
    for m in re.finditer(r'((?:///[^\n]*\n)*)struct (\w+) \{\s*static constexpr std::string_view method = "([^"]+)";\s*static constexpr Kind kind = Kind::(\w+);', s):
        sec = re.search(r'§\s*([0-9.]+)', m.group(1)).group(1).rstrip('.')
        rows[m.group(3)] = (m.group(4), sec, m.group(2))

C = collections.OrderedDict()
C["State: joints and TCP (observed)"] = """get_joint_pos get_actual_joint get_target_joint get_motor_pos get_joint_speed get_joint_acc
 get_motor_speed get_motor_torque get_joint_torques get_joint_temp getCurrentEncode get_tcp_pose getTcpPoseInUser
 get_actual_tcp get_target_tcp get_base_flange_pose get_user_flange_pose get_tcp_speed get_tcp_acc get_tcp_force"""
C["Drives: power, enable, brakes, encoders, alarms"] = """getServoStatus set_servo_status getMotorStatus syncMotorStatus
 get_servo_precise_position_status calibrate_encoder_zero_position clearAlarm reset_robot_status get_servo_alarm_state
 get_servo_brake_off_status get_backdrive_status enter_backdrive exit_backdrive"""
C["Motion: the motion controller and its families"] = """getRobotState stop getSpeed setSpeed moveByJoint moveByLine moveByArc
 moveByRotate moveByLineCoord clearPathPoint addPathPoint moveByPath getPathPointIndex jog getCurrentCoord setCurrentCoord
 moveBySpeedj stopj moveBySpeedl stopl transparent_transmission_init tt_set_current_servo_joint
 get_transparent_transmission_state tt_put_servo_joint_to_buf tt_clear_servo_joint_buf start_push_pos push_pos
 stop_push_pos check_trajectory start_trajectory pause_trajectory resume_trajectory stop_trajectory flush_trajectory
 drag_teach_switch get_drag_state"""
C["Model: kinematics, tools, payload, frames"] = """inverseKinematic inverseKinematicAll positiveKinematic poseMul poseInv
 convertPoseFromCartToUser convertPoseFromUserToCart get_interp_pose getDH getRobotType getRobotSubtype getToolNumber
 setToolNumber getAutoRunToolNumber setAutoRunToolNumber getTcpPos cmd_set_tcp cmd_set_payload get_tool_payload
 getPayload getCentreMass getUserNumber setUserNumber getUserFrame setUserFrame"""
C["Safety: stops, collision, limits"] = """get_estop_status getCollisionState resetCollisionState get_collision_enable_status
 setCollisionEnable getCollisionSensitivity setCollisionSensitivity getRobotSafetyParamsEnabled getRobotSafetyPower
 getRobotSafetyMomentum getRobotSafetyToolForce getRobotSafetyElbowForce getRobotSpeedPercentage
 getRobotDragStartupMaxSpeed getRobotTorqueErrorMaxPercents get_remote_sys_password setSafetyParams"""
C["Programs: JBI, cycle, variables"] = """checkJbiExist runJbi getJbiState run pause getCurrentJobLine getCycleMode setCycleMode
 book_program_clear getSysVarB setSysVarB getSysVarI setSysVarI getSysVarD setSysVarD getSysVarPState getSysVarP setSysVarP
 getSysVarV setSysVarV save_var_data"""
C["I/O: box, flange, virtual bits, fieldbus registers"] = """getInput getOutput setOutput getAnalogInput get_analog_output
 setAnalogOutput getVirtualInput getVirtualOutput setVirtualOutput getRegisters checkFlangeButton setFlangeButton
 get_profinet_int_input_registers get_profinet_int_output_registers get_profinet_float_input_registers
 get_profinet_float_output_registers set_profinet_int_output_registers set_profinet_float_output_registers
 get_eip_int_input_registers get_eip_int_output_registers get_eip_float_input_registers get_eip_float_output_registers
 set_eip_int_output_registers set_eip_float_output_registers"""
C["System: mode, versions, alarms, clock"] = """getRobotMode getSoftVersion getJointVersion getAlarmNum set_system_time"""
C["Peripherals: serial ports, force sensor, force control"] = """open_serial_port setopt_serial_port recv_serial_port
 send_serial_port flush_serial_port close_serial_port open_tci setopt_tci recv_tci send_tci flush_tci close_tci
 start_push_force push_external_force stop_push_force get_force_ctrl_mode get_tcp_force_by_sensor start_force_mode
 end_force_mode get_force_mode_state zero_force_sensor fc_act fc_deact"""

# Values that every 8056 frame also carries (S 3.1).
STREAM = set("""getRobotState getRobotMode getCycleMode getServoStatus getMotorStatus get_joint_pos get_actual_joint
 get_motor_speed get_motor_torque get_joint_speed get_joint_acc get_joint_temp get_joint_torques get_tcp_pose
 getTcpPoseInUser get_actual_tcp get_base_flange_pose get_user_flange_pose get_tcp_speed get_tcp_acc get_tcp_force
 getCollisionState get_estop_status getInput getOutput getAnalogInput get_analog_output get_drag_state""".split())

seen = []
out = ["# The SDK by component", "",
       "The 184 calls of the 3.11.2 SDK, typed in `include/ec/sdk/`, regrouped by the part of the robot they",
       "belong to (`docs/architecture.md` §2) instead of the vendor's services. Generated from the headers by",
       "`python3 tools/sdk_map.py`; do not edit by hand.", "",
       "Columns: the call; its kind in the code (Query, Compute, Setting, Action, Motion, Session); the manual's",
       "section; **8056** when every monitor frame already carries the value, so the observed state needs no call.", ""]
summary = []
for comp, names in C.items():
    names = names.split()
    seen += names
    n8056 = sum(1 for n in names if n in STREAM)
    summary.append((comp, len(names), n8056))
    out += [f"## {comp}", "", "| Call | Kind | § | 8056 |", "|---|---|---|---|"]
    for n in names:
        kind, sec, _ = rows[n]
        out.append(f"| `{n}` | {kind} | {sec} | {'8056' if n in STREAM else ''} |")
    out.append("")
missing = set(rows) - set(seen); extra = set(seen) - set(rows)
dups = [n for n in seen if seen.count(n) > 1]
assert not missing and not extra and not dups, (missing, extra, dups)
table = ["| Component | Calls | of them in 8056 |", "|---|---|---|"] + [f"| {c} | {n} | {k} |" for c, n, k in summary]
table.append(f"| **All** | **{sum(n for _, n, _ in summary)}** | **{sum(k for _, _, k in summary)}** |")
out[9:9] = table + [""]
open("docs/sdk_map.md", "w").write("\n".join(out))
print("\n".join(table))
