# Recording formats

Two ways to keep the monitoring stream (port 8056). `ec_record` writes both;
programs use the `ec::record` sinks (`RawWriter`, `H5StateWriter`).

- **Raw recording** (`.ecraw`): every frame exactly as received, with its
  arrival time. Lossless, no dependencies; replays through the same decoder.
- **HDF5 recording** (`.h5`): each frame as the driver sees it, an `ec::State`
  (`include/ec/state.hpp`) in SI units, one dataset per member. For analysis
  in Python, MATLAB or HDFView.

A raw recording becomes an HDF5 recording at any time:
`ec_record --replay run.ecraw --h5 run.h5`.

## Time

- The arrival time of each frame (`arrival` in HDF5, the record's time in a
  raw recording) is this PC's steady clock in nanoseconds: monotonic and
  unaffected by clock adjustments, but with an arbitrary epoch (often the
  last boot).
- Each file stores the steady clock and the system clock, read together when
  it was created. A record's calendar time is
  `system_ns_at_start + (arrival - steady_ns_at_start)`, in ns since 1970-01-01 UTC.
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
records = raw[24 : 24 + n * 1032].view([("arrival", "<i8"), ("frame", "u1", 1024)])
arrival = records["arrival"]  # ns, steady clock
joints_deg = records["frame"][:, 13 : 13 + 8 * 8].copy().view(">f8")  # machinePos: 8 big-endian doubles at byte 13
```

## HDF5 recording

- HDF5 1.10 file format: readable by HDF5 1.10 and later, so by h5py, MATLAB
  and HDFView.
- Root attributes: `schema` ("ec-driver.recording/2"; the number changes when
  existing readers would break), `steady_ns_at_start`, `system_ns_at_start`.
- One group per stream; whoever writes it chooses the name (`ec_record` writes
  `/state`).
- In a stream's group: the layout of `ec::State`, one dataset per member, its
  path the member's (`state.drives.servo_on` is `drives/servo_on`;
  `state.joints[i].position` is column i of `joints/position`). One row per
  frame; all datasets share the row index (row k is the state with seq k + 1).
- Numeric datasets carry a `unit` attribute. Enums and booleans have none:
  their labels say what they are.
- Datasets are chunked (1024 rows), byte-shuffled and deflate-compressed (level
  4): standard filters that every HDF5 reader has, no plugins.
- Status values are HDF5 enums labelled with the manual's names ("running",
  "remote", ...); a value the manual does not define is stored, unlabelled.
  Booleans are one-byte enums FALSE/TRUE, as h5py writes them.

### The state stream

Units are SI, except the controller's timestamp (ms) and temperatures (degC),
kept as sent. A pose is its 4×4 homogeneous matrix T, row by row: the rotation
in the upper left 3×3, the translation [m] in the last column, as
`Eigen::Isometry3d::matrix()` (notation in `include/ec/pose.hpp`; the
controller's rx, ry, rz are R = Rz(rz) · Ry(ry) · Rx(rx), measured on the EA66
with `tools/rotation_convention.py`).

| Dataset | Shape | Unit | Content (manual field) |
|---|---|---|---|
| `arrival` | N | ns | arrival time, this PC's steady clock |
| `controller_time` | N | ms | timestamp: the controller's clock, since 1970-01-01 |
| `joints/position` | N×6 | rad | machinePos, J1..J6 |
| `joints/velocity` | N×6 | rad/s | jointSpeed |
| `joints/acceleration` | N×6 | rad/s^2 | jointAcc |
| `joints/torque` | N×6 | N m | jointTorque: actual output torque |
| `joints/external_torque` | N×6 | N m | extJointTorques: estimated external torque |
| `joints/torque_of_rated` | N×6 | 1 | torque: fraction of the rated torque (per mille on the wire) |
| `joints/motor_speed` | N×6 | rad/s | motor_speed (rpm on the wire) |
| `joints/temperature` | N×6 | degC | jointTemperature |
| `tcp/pose` | N×4×4 | m | machinePose: T_base_tcp |
| `tcp/pose_in_user` | N×4×4 | m | machineUserPose: the TCP in the current user frame |
| `tcp/flange_pose` | N×4×4 | m | machineFlangePose: T_base_flange |
| `tcp/flange_pose_in_user` | N×4×4 | m | machineUserFlangePose |
| `tcp/speed` | N | m/s | tcpSpeed |
| `tcp/acceleration` | N | m/s^2 | tcpAcc |
| `tcp/force` | N×3 | N | exTcpForceInTool: fx, fy, fz in the current tool frame |
| `tcp/torque` | N×3 | N m | exTcpForceInTool: tx, ty, tz |
| `drives/servo_on` | N | bool | servoReady: servo on, brakes released |
| `drives/synced` | N | bool | can_motor_run: encoders synchronised |
| `motion/state` | N | enum | robotState |
| `motion/hand_guiding` | N | bool | dragState |
| `safety/emergency_stop` | N | bool | emergencyStopState |
| `safety/collision` | N | bool | collision |
| `io/digital_inputs`, `io/digital_outputs` | N | (bits) | digital_ioInput, digital_ioOutput: bit i is channel i |
| `io/analog_inputs` | N×3 | V | analog_ioInput |
| `io/analog_outputs` | N×5 | V | analog_ioOutput |
| `programs/cycle_mode` | N | enum | autorun_cycelMode |
| `system/mode` | N | enum | robotMode: the key switch |
| `system/force_sensor_connected` | N | bool | sensor_connected_state |

Left out: frames that fail to decode (the raw recording keeps them), the
wire's external axes 7 and 8, and its constant size field.

Reading one:

```python
import h5py

with h5py.File("run.h5") as f:
    s = f["state"]
    t = (s["arrival"][:] - s["arrival"][0]) * 1e-9  # s since the first frame
    q = s["joints/position"][:]                     # N x 6 [rad]
    T = s["tcp/pose"][:]                            # N x 4 x 4; T[k, :3, 3] is the TCP position [m]
```

```matlab
t = double(h5read("run.h5", "/state/arrival")) * 1e-9;
q = h5read("run.h5", "/state/joints/position");  % 6 x N: MATLAB shows row-major data transposed
T = h5read("run.h5", "/state/tcp/pose");         % 4 x 4 x N, each matrix transposed: use T(:,:,k).'
```
