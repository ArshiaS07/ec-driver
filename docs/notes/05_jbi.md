# JBI Script Manual Ver3.19.2 [J]
- JBI = Elite's own job language, the only language with motion instructions; files *.jbi ("JOB") in /rbctrl via
  sFTP (root/elite2014, port 22) [J preface, 2]. Lua runs alongside, shares M registers and global variables [J 2].
- Variables: B uint [0,2^31-1], I int16, D double [-1e9,1e9], P joint (deg, 8 elements), V pose (x,y,z mm;
  "Rx,Ry,Rz is the rotation vector in rad ... the norm is the rotation angle") [J 2.1]. NOTE: contradicts the
  measured convention on the EA66 (ADR 0012: R = Rz Ry Rx, fixed XYZ); measurement wins [I: manual wording wrong or
  different meaning]. Local variables L*. M0-M1535 virtual coils; user M528-M911, M1424-M1471 [J 2.1].
- SPEED SEMANTICS [J 2.4]: "In TEACH mode, moving to a point in the program uses MOVJ interpolation and only the
  overall (teach) speed matters. In PLAY mode, for speed types VJ, V, VR the robot runs at speed = (speed parameter)
  x (overall speed % set on the TP). For AV, AVR: speed = set value." => relative speeds are scaled by the global
  speed slider (setSpeed/getSpeed in REMOTE, = PLAY speed); absolute AV/AVR are not. Out-of-range speeds are clamped
  to the kinematic parameter range ("runs with the min/max of the parameter range") [J 2.4.7 note].
- MOVJ/MOVL/MOVC (classic, taught points, PL blending 0-7 deprecated, CR blend radius mm), MOVEJ/MOVEL/MOVEC (new:
  P/LP/V/LV/ConstP/ConstV targets, V/VJ/A/ACC/D/DEC absolute or %, CR [0,2608] mm, TOOL/FLANGE/USER, REF for IK,
  UNTIL with OUT capture, FC full circle) [J 2.4.1-2.4.8]. MOVC must come in groups of 3. In single-step PLAY,
  MOVC moves by joint interpolation.
- UNTIL bool: "If the condition is established, the movement will be suspended immediately ... and the next
  instruction is executed" (same as the SDK cond_*) [J 2.4.2].
- MOVML/MOVEML: replay an offline point file at a fixed interval (header dragfile/trajfile/pathfile/loadfile;
  interval [n] ms >= 2 ms; refJointPos; length; postype joint/pose; refFrame; optional outputs per point); LOADML
  preloads into memory (max 32 files, lost on restart) [J 2.4.5, 2.4.10, 2.4.14]. => the controller's native
  "timestamped/interval trajectory" mechanism; the SDK timestamp service likely writes such a JOB ("trajfile")
  internally (JOB-file alarms in [Sv]) [I].
- MOVDRAG/MOVEDRAG: replay a hand-drag recording, VJ to the start, DRAGV speed, RP smoothing [J 2.4.4, 2.4.9].
- TT in JBI: TTINIT T (0.002-0.1 s) LOOKAHEAD (0.01-1 s, must be longer than T) SMOOTHNESS (0-1); TTSTARTJOINT
  (start position, default current); TTTARGETJOINT joint|pose (add target); TTSTOP [J 2.4.11-2.4.16].
- MCWAIT: temporarily disable collision detection until a condition / timeout [J 2.3.7].
- Track (conveyor) TRACK/RUNIOCUTIN [J 2.4.17-18]; force control STARTFORCEMODE/ENDFORCEMODE; EC66 max force from
  the reducers' torque limits: 170 N [J 2.8.5.1].
- CALL max 10 levels deep, 126 programs [J 2.3.3]. GETTCPFORCE / GETJOINTTORQUE only while moving [J 2.6.11-12].
- POSEMUL: "pose1 as the center of rotation, pose2 as the offset" (= T1*T2, matches ADR 0012) [J 2.5.14].
