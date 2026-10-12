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
| `ec::command` | command structs, `ReplyDecoder` | one type per SDK call: method, parameters, reply type and its decoding | yes |
| `ec::command` | `Client` | one call at a time: send, wait for the response with its id, decode | no |
| `ec` | `RobotState`, `RobotMode`, `CycleMode` | status values both ports report | yes |

Command-line tools (`apps/`): `ec_monitor` prints the 8056 stream; `ec_record`
records it (raw and/or HDF5) or converts a raw recording; `ec_query` sends one
SDK call (read-only methods only, unless `--write`). File formats:
`docs/log-format.md`.

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

---

## 4. Testing

- GoogleTest unit tests for every pure component, and real-socket tests against
  a loopback server (`tests/support/loopback_server.hpp`).
- Both tools end to end against the fake servers (`tests/apps/test_apps.py`,
  run by ctest).
- Fake servers (`tools/`): replay the EA66's recorded replies and stream
  synthetic frames, with fault injection.
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
| 5 | Typed commands for supervision: servo, sync, alarm, tool and user frame, payload, safety limits | typed queries |
| 6 | `supervisor::step`: the pure state machine with its condition tables | unit tests |
| 7 | The supervisor, observe-only | pendant mode change, E-stop, pushing the arm: each detected and named |
| 8 | Preflight: identity and configuration checked against a config file | a changed tool: refuses to start |
| 9 | Bring-up: clear alarm → sync → servo on, each step confirmed from feedback | servos on and off from code |
| 10 | Motion lease and point-to-point motion through the supervisor | first move at 10 % speed; a stop mid-move |
| 11 | Experiments: stop latency, two 8055 connections, close sequences | numbers for timeouts |
| 12 | Trajectory type and validator | bad trajectories rejected |
| 13 | Kinematics checked against the controller's `positiveKinematic` | forward-kinematics error statistics |
| 14 | Timestamped trajectories with a tracking monitor | a planned trajectory on the robot |
| 15 | Logging of commands, setpoints and events (the feedback stream is done: raw and HDF5) | commanded against actual |
| 16 | Transparent-transmission latency experiments | numbers for the loop design |
| 17 | Streamed setpoints with a real-time loop | a streamed sine, one joint, then all |
| 18 | Velocity control (speedj / speedl) | velocity commands that stop on timeout |
| 19 | `ec::Robot`, one object for applications; examples and a guide | |
| 20 | Extras as needed: force control, Python bindings, ROS 2 | |

Proposed changes to this draft, not decided either:

- Orientation stays the controller's rx/ry/rz until its convention is checked
  on the robot, so a math library (Eigen) waits for kinematics.
- The motion lease enforced by a sender above the command client, not inside
  the command layer.
- The supervisor's states built step by step; observe-only needs only a few.

---

## 6. Open questions for the robot

- What does the controller send back for a malformed request line?
- A recorded stream of frames, checked against the pendant, as a test fixture.
- `tools/fake_8055.py` lists a reply to `moveByJoint` as recorded from the
  controller: confirm it on the robot.
- Does the controller accept two simultaneous 8055 connections?
- What does streaming (transparent transmission) do when starved? What does a
  timestamped trajectory do if the host dies mid-run?
- Do the close sequences in `sdk_calls.md` leave the controller ready for every
  other family?
- Which tool does `machinePose` refer to? (Answered: rx, ry, rz compose as
  R = Rz·Ry·Rx, and `poseMul` is T1·T2; `tools/rotation_convention.py`.)
- Frame period: about 8 ms per the manual; 120–122 Hz observed on the EA66.
