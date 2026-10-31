# Demo day: a checklist and notes

For showing the driver on the EA66 in front of people: a check of 5 to 10
minutes alone with the robot first, then the demo, what to do when something
goes wrong, and what to tell people.

Be aware: the tools used here (`ec_play`, the session that switches the
servos off, the waits on the stream) have run against the simulated arm
only. Tomorrow is their first time on the robot, which is why the check
ends with a slow rehearsal. If anything looks wrong in it, show the
pendant's own motion instead and keep the driver for another day.

Commands below: `B=./build/linux-release/apps`, from the repository root.

## The day before

- [ ] Build: `cmake --preset linux-release && cmake --build --preset linux-release`
- [ ] Tests pass: `ctest --preset linux-release` (the tools against the simulated arm included)
- [ ] Try the demo once on the fake: `python3 tools/fake_robot.py --ready` in one terminal,
      then `$B/ec_play examples/star.csv --host 127.0.0.1 --yes` in another

## Before the audience: 5 to 10 minutes

**Safety and the cell** (1 min)

- [ ] The emergency stop within reach, released, and you know where it is
- [ ] Nobody within the arm's reach (914 mm from its base, more with a tool); the cell clear
- [ ] Nothing loose on the flange; the tool and payload set on the pendant as mounted

**The pendant** (1 min)

- [ ] No alarm shown (or note it: bring-up clears what it can and says what it cannot)
- [ ] The key switch at **REMOTE** (every motion command needs it)
- [ ] Poses shown in the **base** frame, for the tool the program will use (`ec_info` below shows
      which: "REMOTE/PLAY tool")

**Read only: is the robot answering?** (1 min)

- [ ] `ping -c 3 192.168.0.200`
- [ ] `$B/ec_move status`: mode remote, no emergency stop, no collision; the last line says what a
      motion still needs (servo off, not synchronised: bring-up does that)
- [ ] `$B/ec_info | head -40`: the version, the active tool and its TCP; keep the whole output
      (`$B/ec_info > info.txt`) for later
- [ ] `$B/ec_log --level error --lines 10`: no fresh errors

**Put the shapes where the arm is comfortable** (3 min)

- [ ] On the pendant, jog the TCP to a pose in the middle of the free space, the tool pointing
      down, the arm bent (not stretched out, the wrist not straight: J3 and J5 away from 0 and
      180 deg), with about 70 mm free all round and below
- [ ] Read x, y, z [mm] and rx, ry, rz [deg] on the pendant; put them in `CENTRE` and `TOOL` at the
      top of `examples/shapes.py`; run `python3 examples/shapes.py`
- [ ] Dry run every file you will show (nothing moves; each takes a few seconds, one IK call per
      point): `$B/ec_play examples/square.csv`, then star, circle, helix, cone. Each must say
      "planned into N points", and "the arm is ... deg from the start" should be a few degrees
- [ ] A file that fails names the pose and the reason: move `CENTRE` and regenerate (see below)

**Rehearse once, slowly** (2 to 3 min)

- [ ] Hand on the emergency stop: `$B/ec_play examples/square.csv --yes --speed 0.3`
- [ ] Watch: bring-up, a slow joint move to the start, the upload (a second or two), the square, then
      "shut-down: ... the servos report off"
- [ ] `$B/ec_move status` says servos off; the pendant shows no alarm
- [ ] Decide the order and the speeds (below); leave the servos off and the key at REMOTE

## The demo: about 5 minutes

Each `ec_play` is a whole session: it switches the servos on, moves to the
start, runs, and switches them off at the end, also after Ctrl+C or a fault.

1. **The robot as the driver sees it.** `$B/ec_move status`, and `$B/ec_info | head -30`
   (versions, DH parameters, tools). Say: these only read; nothing moves.
2. **A plan, without moving.** `$B/ec_play examples/star.csv`: the poses were solved by the
   controller's own inverse kinematics, checked, and nothing moved without `--yes`.
3. **The square, then the star.** `$B/ec_play examples/square.csv --yes --speed 0.5`, then
   `$B/ec_play examples/star.csv --yes`. Point out: it slows into every corner and pauses there;
   the servos go off at the end.
4. **The smooth ones.** `$B/ec_play examples/circle.csv --yes` (two laps at 40 mm/s), and
   `$B/ec_play examples/helix.csv --yes` (it rises 60 mm: three dimensions).
5. **The cone.** `$B/ec_play examples/cone.csv --yes`: the tool tip holds still while the tool
   tilts round it. If the tip wanders, the tool's TCP is set wrong on the controller: a check
   integrators use.
6. **Stopping.** Press Ctrl+C during a run: the arm stops, and the shut-down still runs (servos
   off). Or press the emergency stop, release it, then `$B/ec_move --yes recover` (it shows the
   alarm and the log, then brings the robot up again, leaving the servos on: the next `ec_play`
   switches them off at its end, or `$B/ec_move --yes shut-down` does).

Optional, if there is time: `--record run.ecraw` on a run, then
`$B/ec_record --replay run.ecraw --h5 run.h5`: every 8 ms frame of the run, in SI, for plotting.

## If something goes wrong

| What you see | What to do |
|---|---|
| `connect: command port ...` | the network or the address: `ping`; the controller booted? |
| `not in remote mode: turn the key switch to REMOTE` | the key switch, then run again |
| `emergency stop: remove the cause and release ...` | release the button, then `$B/ec_move --yes recover` |
| `alarm` or `still alarm after its remedy` | read it: `$B/ec_log --level error --lines 20`; clear it on the pendant; `recover` |
| `collision` during a run | the arm touched something or moved too fast: `recover`; lower `--speed` |
| `plan: inverse kinematics failed for the pose ...` | out of reach from there: move `CENTRE` closer to the base, regenerate |
| `plan: ... singularity` or `... beyond the joint limit` | another `CENTRE`: the arm more bent, the wrist turned (J5 away from 0 and 180 deg) |
| `plan: joint ... would move at ... above ...` | the shape too fast for the joints there: lower `SPEED` in shapes.py, regenerate |
| a large distance "from the start" in the dry run | the arm would swing a long way first: jog nearer to `CENTRE`, or regenerate around where it is |
| the servos stay on after a failure | `$B/ec_move --yes shut-down` |
| the tools hang or the ports stop answering | Ctrl+C; if 8055/8056 stay dead, power-cycle the controller (it happened once, after a burst of unconfirmed calls; these tools never send one) |

## Rules for the day

- **One tool at a time.** Whether the controller takes a second client at once is not yet known
  (`docs/ec66.md` §8 #4): no `ec_monitor` beside `ec_play`, no `tools/robot_checks.py` today.
- **The emergency stop is the safety function.** Ctrl+C and the session's shut-down are
  conveniences on top of it.
- **Slow first.** Every new file: a dry run, then `--speed 0.3`, then faster.

## Notes: what to tell people

- **What it is.** A C++23 driver for the Elite EC66 (our EA66): the controller's whole SDK (184
  calls) as typed C++, and on top of it the robot as its parts (joints, drives, model, safety,
  I/O, programs, the motion controller), workflows for starting and ending a session, and tools.
- **Two paths to one robot.** Commands go over one port, one at a time, each answered before the
  next. The robot's state streams on another port every 8 ms: positions, speeds, torques,
  temperatures, the TCP, the flags. The driver keeps the newest snapshot, free to read.
- **Who does what.** The controller owns the motion: its inverse kinematics solves every pose,
  and its timestamp service plays the trajectory on its own clock. The driver plans the timing
  (a smooth spline, checked against the joints' limits), uploads it, starts it, and supervises.
- **Safe by construction.** Nothing moves without `--yes`; a motion whose code is left (an error,
  an early return) is stopped; every program ends with the servos off, however it ends; the
  driver never fires unconfirmed commands at the robot (a lesson from the robot itself).
- **Tested without the robot.** Unit tests against scripted controllers, and the tools end to end
  against a simulated arm with fault injection (emergency stops, alarms, collisions, slow
  answers), with sanitizers; the build covers Linux, Windows and 64-bit ARM.
- **What is next.** Trajectories first (this demo); then closed-loop control, streaming targets
  every few milliseconds; the driver is refined as it gets used.
