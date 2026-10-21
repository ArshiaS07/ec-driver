# Communication Protocol Manual Ver3.19.2 [C] (controller called "ERC-G200 control system")
- Fieldbus side, all SLAVE: Modbus RTU (controller RS485: pin 2 GND, 4 485+, 3 485-) and Modbus TCP (port 502);
  Profinet slave (GSD; modules INT 32 in/32 out, FLOAT 32 in/32 out, STATUS 128 regs); EtherNet/IP adapter (EDS;
  11 slots, point-to-point only, no multicast) [C 2-4]. Configure on TP, effective after restart.
- Modbus maps only the virtual bits M: M0-M527 system read-only, M528-M1471 read/write, M1472-M1535 read-only
  (coils = M index; registers = 8 or 16 M per register, "compatible mode") [C 2.3.1].
- Modbus read-only robot info (holding registers, 16 bit): 200.. joints (0.0002 rad), 210.. joint speeds (0.0002
  rad/s; only in joint motion, 0 during line/arc), 220.. TCP pose in base (0.1 mm, 0.0002 rad), 228 safety params
  enabled, 229 collision detection enabled, 230 TCP speed (0.1 mm/s), 240.. DI X0-X63, 250.. DO, 260.. AI (mV),
  270.. AO (mV), 280.. timestamp (ms since 1970, 4 regs), 450 TCP accel, 460.. joint accel, 480-499 safety limits
  (normal/reduced: power W, momentum kg m/s, tool force N, elbow force N, speed %), 500 collision sensitivity,
  192-199 / 284-447 general registers [C 2.4].
- "Generally, the modbus communication cycle is 10ms. When the system is in alarm state, the communication cycle
  will reach 1010ms. Flange I/O: allow an additional 1000 ms." [C 2.4.1] => controller deprioritises comms in alarm.
- Profinet/EIP status: pose (mm, rad), TCP speed/accel, joints (rad), joint speeds (rad/s), joint accel (deg/s^2),
  DIO bitfields, AIO (mV), collision enable/sensitivity, safety params, timestamp; MIN M912.. (in), MOUT M400.. (out),
  int/float registers [C 3.1, 3.3, 4.1.2]. The SDK has calls for these registers (see the SDK manual).
