# Service manual EC66 Ver3.19.2 (2025-01-15) [Sv]
## Hardware facts
- Joints are modules on a daisy chain: each joint-to-joint connection = 48 VDC (brown) + GND (black) + one twisted
  pair "485 communication" [Sv 3.2.5-3.2.13]. => servo drives live IN the joints (integrated joint drives); the
  control box feeds 48 V DC and a field bus over RS-485 physical [I: protocol is Elite's own; alarm texts also name
  EtherCAT ("EtherCAT servo disconnected", "Ethercat DC synchronization failed", 0-1030-3E/3F/40/41/42), probably for
  other models/external axes].
- Joint sizes (spare parts 5-1): J1 EC66-25-1, J2 EC66-25-2, J3 EC66-20-3, J4/J5 EC63-14-4/5, J6 EC63-14-6.
  Per size: "Electromagnet Assembly" (= the holding brake, electromagnetic) and "Magnetic Encoder Assembly".
- Encoders per joint (alarms 0-1800-3c..52, servo alarms 60-82): a JOINT encoder (output side; magnetic absolute
  [I from part name]) and a MOTOR encoder (A/B pulses + index; "contaminated/polluted" => optical incremental [I]),
  plus HALL sensors (commutation, electrical angle). The drive cross-checks joint vs motor encoder
  ("data difference between joint and motor encoder is too large", 61/63) and needs them calibrated against each
  other (62/74). Multi-turn ("multi circle") data is kept in the joint's flash (servo alarms 200, 202, 203).
- Last shutdown position stored and compared at start-up: "0-1030-4F The last shutdown position of joint %s is
  different from the current position"; "0-1031-1/0-1031-9 At rest, the feedback position ... greatly different from
  the actual position. Please check and restart the encoder." Manual brake release: do not move a joint more than
  160 deg unpowered "to enable the robot finding its original physical position" [Sv 3.2.2].
- Holding brakes: released electrically at servo-on; at brake opening the drive turns the motor forward and back to
  verify the brake opened (servo alarms 90-92 "holding brake abnormal and the forward/reverse rotation of motor
  fails") [I: brake check at enable]. Alarm 94: sliding distance after closing the brake too large. Manual release:
  remove joint cap, push the brake down, rotate [Sv 3.2.2]. Alarm 22/31: over-current at start-up / deviation too
  large -> "servo brake is not opened" -> use reverse drive (backdrive) to move to lower gravity position, then the
  normal brake opening function.
- Control box (B2 control cabinet; models ERB2E1K6 and ERB2E1K0M) [Sv 5-2, 3.3]: core board ("Core Plate", the main
  controller), PAD IO board (+ PADIO expansion board), brake plate (bus overvoltage / braking resistor: servo alarm 120
  "bus voltage is overvoltage, check whether the brake plate is connected"), brake resistance components, DC/DC
  48 V, 24 V, 5 V supplies, fan + filters (2 filters, one each side), battery, limit switch, network cable, 220 V
  cord. Teach pendant ERP300 connects by DVI (display) + network cable [Sv 3.3.1].
- Inspection: E-stop on TP monthly: "robot can immediately stop running and automatically switch off the power
  supply of the joints" [Sv 2.1.2] => E-stop = joint power cut (stop category 0/1 [I]). Free-drive via black
  Freedrive button on the back of the TP. BACKDRIVE: press ON -> status "Standby"; hold Free Drive -> BACKDRIVE:
  brakes released per joint as it is moved, robot clunky [Sv 2.1.2]. Grounding <1 ohm.
## Alarm system (chapter 4) — format "L-MMMM-N" (first digit looks like severity: 0 error, 1 warning, 2 info [I])
- Robot program alarms 0-1000..0-9000, servo drive alarms (numbered 10..203 in 4.2, appear as 0-1800-xx hex).
- Notable states/alarms for a driver: 2-1030-32 / 0-1030-82 / 0-1030-1D / 0-1030-a6 "The servo is not enabled.
  Please turn on the servo first."; 2-1030-33 "The robot is not in PLAY mode"; 0-1030-83 "Robot is not in teach mode
  or remote mode"; 1-1030-34 / 0-1030-88 "Please clear the collision alarm first!"; 2-1030-35/36 "Robot is in the
  safeguard stop"; 2-1030-37 "external pause state"; 2-1030-38 "Safety door triggered"; 2-1030-39 "Pause";
  0-1030-45/46/47 emergency stop alarm X0 / M0 / external; 0-1030-48 collision warning; 0-1030-4E "Emergency stop
  timeout, protection stop timeout alarm"; 1-1030-cc "Enter reduction mode" / 0-1030-cd "Enter normal mode";
  0-1030-ca "Joint %s speed exceeds limit [%s]:%s deg/s"; 0-1030-cb motor overspeed; 0-1030-1C/0-1031-7/0-1032-6
  joint soft limit alarm; 0-1030-86/0-1033-0 "Axis %s exceeds the limit at the target position"; 0-1030-eb /
  0-1100-0 / 0-7000-21.. "The target pose is singular"; 0-1033-2 "Inversion failed. The target point is singular or
  unreachable"; 0-7000-11/15 "The robotic arm is in a singular zone or unreachable zone"; 2-1030-7e "reaches the
  target pose, but the configuration is inconsistent"; 0-1032-8 "Servo off timeout!"; 0-1032-9 "turn off the servo
  enablement before switching the servo control mode"; 2-1031-c "Failed to set servo enable"; 1-1031-3/1-1033-3
  "The robot status has been reset"; 0-1032-7 "Unable to enable drag teaching in PLAY mode"; 0-1032-5 "not in a
  normal state, cannot start dragging"; 0-1100-49 "axes cannot be freely dragged: joint singular or overspeed".
- Trajectory (timestamp) service checks inside: 0-1030-b0 "The timestamp in the trajectory motion instruction is
  not incremented"; 0-1030-b2 "number of timestamps ... does not match the number of points"; JOB file alarms
  (0-1030-a8..b4: "JOB file %s format error", "Successfully uninstalled JOB file") [I: uploaded trajectories become
  JOB files]. 2-2002-104 "TTTARGETJOINT failed, the queue is full!" (TT queue has a capacity). 0-2002-117
  "TTTARGETJOINT instruction inverse kinematic failed". MOVML (multi-point line, 0-1030-b5/ec "first waypoint
  unreachable"), 0-7000-7 "MOVEML trajectory planning calculation failed", node vector (spline) alarms.
- Dynamics: "Dynamic model verification failed, unable to set load parameters" (2-1030-16); the controller has a
  dynamic model used for collision detection, drag, external force estimation ("Since the dynamics is disabled,
  the external force/torque cannot be estimated").
- Servo drive alarms (4.2) with causes: 10/21/23 over-current (load/acceleration params wrong, overload, stuck,
  brake not opened); 20 current zero offset; 22 over-current at start-up (brake not opened); 30/31 following error
  too large; 32 special E-stop input; 40 joint E-stop alarm = TP E-stop pressed -> release, then "reset and
  synchronization operations"; 41 writing parameters; 43 restarting (firmware upgrade); 50-54 joint bus comms
  (verification, not synchronized -> 'reset', master frame not received, frame incomplete); 60-63 joint encoder;
  70-79 motor encoder; 80-82 Hall; 90-94 holding brake; 100 overload; 101 joint collides; 110 max motor speed;
  120 bus overvoltage (brake plate); 121 undervoltage; 130 overheated; 200/202/203 multi-turn data in flash;
  201 "Joint is not ready, initialization is not complete: usually appears after the power is turned on, which is
  a normal alarm; click 'reset' to clear" => alarm after power-on is NORMAL; clearAlarm is part of start-up.
- Software: Lua scripts (max 8 at once, BACKSTAGERUNLUA), JBI programs, P/V variables files, palletizing, welding,
  force control (FT sensor), tracking (conveyor), "mcserver" (0-2-D "Can't connect to mcserver") [I: the motion
  control server process]. Packing program EC_66_packing_angle run in Play mode.
