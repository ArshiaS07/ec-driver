# Architecture

Controller: Elite EC series (Annuo EA66 = EC66), firmware V3.15.3, SDK manual
Ver3.11.2 (`docs/sdk_calls.md` lists every call). Language: C++23. One robot,
one host.

This document describes what exists. Section 5 is a draft roadmap, open to
change.

---

## 1. The controller's interface

| Port | Direction | Protocol | Who initiates |
|---|---|---|---|
| 8055 | PC → robot → PC | JSON-RPC 2.0, one request per line, one response per request | always the PC |
| 8056 | robot → PC | 1024-byte big-endian frames, about every 8 ms | always the robot |

Modbus (502), EtherNet/IP (44818) and the log ports are out of scope.

---

## 2. Components

Small components with one job each, generic where the job allows, and composed
by the layers above; a lower layer knows nothing of the layers above it.
"Pure" means no I/O, no threads and no clocks: testable without a robot.

| Namespace | Component | Its job | Pure |
|---|---|---|---|
| `ec::net` | `TcpSocket` | TCP client, Linux and Windows; every call bounded by a timeout | no |
| `ec::feedback` | `Framer` | 8056 byte stream → whole 1024-byte frames, realigning after junk | yes |
| `ec::feedback` | `Frame`, `decode` | 1024 bytes → typed frame, in the controller's units | yes |
| `ec::feedback` | `State`, `to_si` | a frame in SI units: the one place where 8056's units are converted | yes |
| `ec::feedback` | `Reader` | monitoring port → `Sample`s (each frame as received, with its arrival time) | no |
| `ec::feedback` | `Receiver` | a thread that keeps the monitoring port connected and hands every `Sample` to a callback | no |
| `ec` | `Latest<T>` | the newest value and its sequence number, shared between threads; wait for the next one | no (a mutex) |
| `ec::log` | `Sink<Record>` | the interface every recorder implements: `write(record)` | yes |
| `ec::log` | `RawWriter`, `RawReader` | raw recordings: every frame as received, with its arrival time | no |
| `ec::log` (target `ec::hdf5`, optional) | `H5File`, `H5FeedbackWriter` | HDF5 logs: one dataset per field, SI units | no |
| `ec::command` | `LineFramer` | 8055 byte stream → lines | yes |
| `ec::command` | `encode`, `parse_response` | request → JSON-RPC line; response line → id, and result or error | yes |
| `ec::command` | command structs, `ReplyDecoder` | one type per SDK call (all 184): method, kind, parameters, reply type and its decoding | yes |
| `ec::command` | `AllCommands`, `all_commands()` | every typed call, as a type list and as a table of names and kinds | yes |
| `ec::command` | `Client` | one call at a time: send, wait for the response with its id, decode | no |
| `ec` | `RobotState`, `RobotMode`, `CycleMode` | status values both ports report | yes |
| `ec` | `Joints`, unit conversions | SI types, and deg/mm conversion where commands are built | yes |
| `ec` | poses (`Eigen::Isometry3d`, T_ab), `from_mm_rad`, `to_mm_rad` | poses in SI, converted to the controller's mm and rx, ry, rz only at the wire | yes |
| `ec` | `Clock`, `Stamped`, `PoseTrajectory` | the one (steady) clock; a value stamped with its time; a motion of the TCP as poses stamped with their time since the start | yes |
| `ec` | `Error` | the driver's error: a kind and what was observed | yes |
| `ec::motion` | `plan()`, `check_step()`, `JointSpline`, `JointTrajectory` | a motion of the TCP solved by a given inverse kinematics (the controller's) into a checked joint trajectory | yes |
| `ec::motion` | `Move`, `Timestamped`, `ServoStream`, `move_along` | motion families (ADR 0018): each sends its family's sequence from the manual over the robot's connection; one open at a time | no |
| `ec` | `Robot` | one controller: the command connection (one call at a time, each answered), the newest monitoring frame, the controller's kinematics, bring-up, stop; what applications use | no |

Command-line tools (`apps/`): `ec_monitor` prints the 8056 stream; `ec_record`
records it (raw and/or HDF5) or converts a raw recording; `ec_query` sends one
SDK call (queries and computations; anything else needs `--write`), lists
every call, or checks every query's reply against its type; `ec_move` moves
the arm through `ec::Robot` (bring-up, joint moves, a streamed sine, stop),
printing its plan unless given `--yes`, and records 8056 while it runs with
`--record`; `sbx_trajectory`, a sandbox to scroll through, draws a circle of
the TCP from a free function returning stamped poses: brought up, verified,
executed by the timestamp service. File formats: `docs/log-format.md`.
The motion design and its reasons: `docs/motion.md`.

---

## 3. Rules the code follows

- Failures are values (`std::expected`), not exceptions, throughout the library.
- The wire layer (`feedback::Frame`, the command structs) uses the controller's
  units (deg, mm, rad, %), with the unit in each member's name, so values can be
  checked against the manual and the teach pendant. Above it, values are SI,
  converted at one boundary (`feedback::to_si` for the 8056 stream).
- Records are kept as they arrive (a frame's bytes and its arrival time); sinks
  derive what they need. Nothing derivable is computed while recording.
- Every socket call waits at most its timeout, and reports a timeout only once
  it has fully elapsed.
- A command describes a call and does no I/O: a method name and a reply type,
  plus `params()` or `decode()` only when needed. Decoding checks type and range
  and never throws.
- The controller wraps every result in a JSON string; the codec unwraps it.
- The client has one call in flight. It discards, and counts, late responses
  to timed-out calls and lines that are not responses; a failed send or a lost
  connection closes the connection for good.
- A status value the manual does not define fails a reply that carries it; in a
  frame it is kept, and `is_valid()` reports it.
- Every command declares its kind (query, compute, setting, action, motion,
  session); what may be sent when follows from it (ADR 0015).
- One connection to 8055 per robot: calls go one at a time, each answered
  before the next, and the motion families send their manual's sequences;
  nothing is sent unconfirmed (ADR 0018, after the first robot runs).
- The driver starts no thread but the optional 8056 receiver; streams are
  paced by the application, waits poll `getRobotState`.

---

## 4. Testing

- GoogleTest unit tests for every pure component, and real-socket tests against
  a loopback server (`tests/support/loopback_server.hpp`).
- Activities on a bench (`tests/support/bench.hpp`) that answers their
  commands and plays the state cycle by cycle; the control loop and
  `ec::Robot` against loopback servers.
- Every tool end to end against the fake servers (`tests/apps/test_apps.py`,
  run by ctest), `ec_move` and `sbx_trajectory` against a simulated arm
  (`tools/fake_robot.py`) with injected faults (emergency stop, teach mode, a
  rejected call).
- Fake servers (`tools/`): replay the EA66's recorded replies, stream
  synthetic frames, or simulate a moving arm, with fault injection.
- CI: format check; all six presets built and tested; AddressSanitizer and
  UndefinedBehaviorSanitizer in `linux-debug`, ThreadSanitizer in `linux-tsan`;
  clang-tidy.

---

## 5. Roadmap (draft)

Not reviewed in depth; expected to change with experience on the robot. Each
step ends with something testable.

| # | Step | Test |
|---|---|---|
| 4 | Done: `feedback::Receiver` (8056 thread), `ec::Latest` (newest value); also `Reader`, `to_si`, and the recording of step 15 | on the robot: pull the cable, staleness detected and the stream back after plugging in |
| 5 | Done: all 184 calls typed, with kinds and a registry | `ec_query --check-all` on the robot |
| 6 | Built, then replaced (ADR 0018): no executor or conditions gate; the controller's own refusals | unit tests |
| 7 | The supervisor, observe-only (status without moving: `ec_move status`) | pendant mode change, E-stop, pushing the arm: each detected and named |
| 8 | Preflight: identity and configuration checked against a config file | a changed tool: refuses to start |
| 9 | Done: `Robot::bring_up()`, the manual's sequence, waiting for the servos | `ec_move --yes bring-up` on the robot |
| 10 | Done: `motion::Move` (moveByJoint) | first move at 10 % speed; a stop mid-move |
| 11 | Experiments: stop latency, two 8055 connections, close sequences; tuning the numbers in `docs/motion.md` §7.2 | numbers for timeouts |
| 12 | Done: `JointTrajectory`, `validate()` | bad trajectories rejected |
| 13 | Done differently: the controller's own kinematics, in joint-space planning (ADR 0017, proposed); checked on the robot with `ec_query positiveKinematic` against `ec_monitor` | `sbx_trajectory` |
| 14 | Done: `motion::Timestamped`, `plan()`, `move_along`; the circle ran on the robot, 2026-10-14 | a planned circle on the robot (`sbx_trajectory`) |
| 15 | Logging of commands, setpoints and events (the feedback stream is done: raw and HDF5) | commanded against actual |
| 16 | Transparent-transmission experiments (`docs/motion.md` §7: underrun, step response, round trip) | numbers for the loop design |
| 17 | Done in part: `motion::ServoStream`, paced by the application, no watchdog yet (ADR 0018) | `ec_move --yes sine 6 2 4`, then a streamed TCP motion on the robot |
| 18 | Removed for now; returns as a family in milestone B (ADR 0018) | velocity commands that stop on timeout |
| 19 | Done: `ec::Robot` (ADR 0018); examples and a guide still to write | |
| 20 | Extras as needed: force control, Python bindings, ROS 2 | |

Proposed changes to this draft, not decided either:

- Steps 6–10 and 16–19 followed ADR 0014 (one control thread running
  activities), replaced on 2026-10-14 by ADR 0018 (proposed): one class per
  motion family on one connection, no control thread (`docs/motion.md` §5.0).

- Poses are Eigen isometries since the convention was measured (ADR 0012) and
  planning needed them (ADR 0017); rx/ry/rz stay at the wire.
- The motion lease enforced by a sender above the command client, not inside
  the command layer.
- The supervisor's states built step by step; observe-only needs only a few.

---

## 6. Open questions for the robot

- What does the controller send back for a malformed request line?
- A recorded stream of frames, checked against the pendant, as a test fixture.
- `tools/fake_8055.py` lists a reply to `moveByJoint` as recorded from the
  controller: confirm it on the robot.
- Motion: the questions in `docs/motion.md` §7, each with the default used
  until it is measured. The read-only ones (round trip, several requests in
  flight, two connections, clock drift) run with `tools/robot_checks.py`.
- Do the close sequences in `sdk_calls.md` leave the controller ready for every
  other family?
- Which tool does `machinePose` refer to? (Answered: rx, ry, rz compose as
  R = Rz·Ry·Rx, and `poseMul` is T1·T2; `tools/rotation_convention.py`.)
- Frame period: about 8 ms per the manual; 120–122 Hz observed on the EA66.
