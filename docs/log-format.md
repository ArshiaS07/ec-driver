# Log formats

Two ways to keep the monitoring stream (port 8056). `ec_record` writes both;
programs use the `ec::log` sinks (`RawWriter`, `H5FeedbackWriter`).

- **Raw recording** (`.ecraw`): every frame exactly as received, with its
  arrival time. Lossless, no dependencies; replays through the same decoder.
- **HDF5 log** (`.h5`): the frames decoded and converted to SI units, one
  dataset per field. For analysis in Python, MATLAB or HDFView.

A raw recording becomes an HDF5 log at any time:
`ec_record --replay run.ecraw --h5 run.h5`.

## Time

- `t`, the arrival time of each frame, is this PC's steady clock in
  nanoseconds: monotonic and unaffected by clock adjustments, but with an
  arbitrary epoch (often the last boot).
- Each file stores the steady clock and the system clock, read together when
  it was created. A record's calendar time is
  `system_ns_at_start + (t - steady_ns_at_start)`, in ns since 1970-01-01 UTC.
- The controller's own timestamp is data: `controller_time`, its clock in ms
  since 1970-01-01, as sent. Nothing aligns the clocks; that is left to analysis.

## Raw recording

Our own fields are little-endian; frames are kept byte for byte (big-endian,
as the controller sends them, layout in manual section 3.1).

| Offset | Bytes | Content |
|---|---|---|
| 0 | 8 | magic `ECRAW001` (the last three characters are the version) |
| 8 | 8 | int64: steady clock at start [ns] |
| 16 | 8 | int64: system clock at start [ns since 1970-01-01 UTC] |
| 24 | 1032 per record | int64 arrival time [ns, steady clock], then the 1024-byte frame |

A recording that stopped abruptly can end with a partial record; readers leave
it out.

Reading one with numpy:

```python
import numpy as np

raw = np.fromfile("run.ecraw", dtype=np.uint8)
assert raw[:8].tobytes() == b"ECRAW001"
steady_at_start, system_at_start = raw[8:24].view("<i8")
n = (raw.size - 24) // 1032
records = raw[24 : 24 + n * 1032].view([("t", "<i8"), ("frame", "u1", 1024)])
t = records["t"]
joints_deg = records["frame"][:, 13 : 13 + 8 * 8].copy().view(">f8")  # machinePos: 8 big-endian doubles at byte 13
```

## HDF5 log

- HDF5 1.10 file format: readable by HDF5 1.10 and later, so by h5py, MATLAB
  and HDFView.
- Root attributes: `schema` ("ec-driver.log/1"; the number changes when
  existing readers would break), `steady_ns_at_start`, `system_ns_at_start`.
- One group per stream; whoever writes it chooses the name (`ec_record` writes
  `/feedback`).
- In a stream's group: one dataset per field, one row per record, shape `[N]`
  or `[N, k]`. All datasets share the row index; `t` is the arrival time.
- Numeric datasets carry a `unit` attribute. Enums and booleans have none:
  their labels say what they are.
- Datasets are chunked (1024 rows), byte-shuffled and deflate-compressed (level
  4): standard filters that every HDF5 reader has, no plugins.
- Status values are HDF5 enums labelled with the manual's names ("running",
  "remote", ...); a value the manual does not define is stored, unlabelled.
  Booleans are one-byte enums FALSE/TRUE, as h5py writes them.

### The feedback stream

Units are SI, except the controller's timestamp (ms) and temperatures (degC),
kept as sent. Rotations are the controller's rx, ry, rz as sent, composed as
R = Rz(rz) · Ry(ry) · Rx(rx): about the fixed X axis, then Y, then Z (measured
on the EA66 with `tools/rotation_convention.py`).

| Dataset | Shape | Unit | Content (manual field) |
|---|---|---|---|
| `t` | N | ns | arrival time, this PC's steady clock |
| `controller_time` | N | ms | timestamp: controller clock, since 1970-01-01 |
| `joint_position` | N×6 | rad | machinePos, J1..J6 |
| `joint_velocity` | N×6 | rad/s | jointSpeed |
| `joint_acceleration` | N×6 | rad/s^2 | jointAcc |
| `joint_torque` | N×6 | N m | jointTorque: actual output torque |
| `joint_torque_external` | N×6 | N m | extJointTorques: estimated external torque |
| `joint_torque_rated` | N×6 | 1 | torque: fraction of the rated torque (per mille on the wire) |
| `motor_speed` | N×6 | rad/s | motor_speed (rpm on the wire) |
| `joint_temperature` | N×6 | degC | jointTemperature |
| `tcp_position_base`, `tcp_position_user` | N×3 | m | machinePose, machineUserPose: x, y, z |
| `tcp_rotation_base`, `tcp_rotation_user` | N×3 | rad | the same poses: rx, ry, rz |
| `flange_position_base`, `flange_position_user` | N×3 | m | machineFlangePose, machineUserFlangePose: x, y, z |
| `flange_rotation_base`, `flange_rotation_user` | N×3 | rad | the same poses: rx, ry, rz |
| `tcp_speed` | N | m/s | tcpSpeed |
| `tcp_acceleration` | N | m/s^2 | tcpAcc |
| `tcp_force` | N×3 | N | exTcpForceInTool: fx, fy, fz in the current tool frame |
| `tcp_torque` | N×3 | N m | exTcpForceInTool: tx, ty, tz |
| `robot_state` | N | enum | robotState |
| `robot_mode` | N | enum | robotMode |
| `cycle_mode` | N | enum | autorun_cycelMode |
| `brakes_released` | N | bool | servoReady |
| `motors_synced` | N | bool | can_motor_run |
| `collision` | N | bool | collision |
| `emergency_stop` | N | bool | emergencyStopState |
| `drag_enabled` | N | bool | dragState |
| `force_sensor_connected` | N | bool | sensor_connected_state |
| `digital_input`, `digital_output` | N | (bits) | digital_ioInput, digital_ioOutput: bit i is channel i |
| `analog_input` | N×3 | V | analog_ioInput |
| `analog_output` | N×5 | V | analog_ioOutput |

Left out: frames that fail to decode (the raw recording keeps them), the
wire's external axes 7 and 8, and its constant size field.

Reading one:

```python
import h5py

with h5py.File("run.h5") as f:
    fb = f["feedback"]
    t = (fb["t"][:] - fb["t"][0]) * 1e-9  # s since the first frame
    q = fb["joint_position"][:]           # N x 6 [rad]
```

```matlab
t = double(h5read("run.h5", "/feedback/t")) * 1e-9;
q = h5read("run.h5", "/feedback/joint_position");  % 6 x N: MATLAB shows row-major data transposed
```
