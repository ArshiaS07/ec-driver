#!/usr/bin/env python3
"""Trajectories for ec_play: a few shapes for the TCP, written as CSV files
next to this one (t [s]; x, y, z [mm]; rx, ry, rz [deg], as the pendant shows
poses). Standard library only.

Each shape is a function returning stamped poses. They are drawn around
CENTRE with the tool held at TOOL, in the base frame's XY plane. Set both from
the pendant (a pose the arm reaches comfortably, with the cell clear around it
by the shape's size, about 70 mm), then run

    python3 examples/shapes.py

The values below are the fake arm's (tools/fake_robot.py at the joints
0 -90 90 -90 -90 0), where the committed files are checked. Run each with
`ec_play FILE` first (it plans and prints; nothing moves), then add --yes.

Every stretch of a path starts and ends at rest, speeding up and slowing down
smoothly over RAMP seconds (a cosine ramp), at SPEED in between; corners are
held for PAUSE. Rows are DT apart.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import poses  # noqa: E402  (the rotation convention measured on the EA66: R = Rz Ry Rx)

CENTRE = (-496.0, -122.0, 424.431)  # [mm], base frame
TOOL = (180.0, 0.0, 90.0)  # rx, ry, rz [deg]: the tool pointing down

SPEED = 40.0  # [mm/s] along a path
TURN_RATE = 30.0  # [deg/s] for the cone's turn
RAMP = 1.0  # [s] from rest to speed, and back
PAUSE = 0.3  # [s] held at a corner
DT = 0.05  # [s] between rows


# ------------------------------------------------------------------ timing

def timed(total: float, rate: float, start: float):
    """(t, u) from u = 0 at `start` to u = total, at rest at both ends: a cosine
    ramp up to `rate` over RAMP (shorter if the stretch is short), cruise, ramp down."""
    ramp = min(RAMP, total / rate)
    cruise = total / rate - ramp
    end = 2.0 * ramp + cruise

    def u_at(t: float) -> float:
        if t <= 0.0:
            return 0.0
        if t >= end:
            return total
        if t < ramp:  # the integral of rate (1 - cos(pi t / ramp)) / 2
            return rate * (t / 2.0 - ramp / (2.0 * math.pi) * math.sin(math.pi * t / ramp))
        if t <= ramp + cruise:
            return rate * (ramp / 2.0 + t - ramp)
        return total - u_at(end - t)

    steps = max(1, math.ceil(end / DT))
    return [(start + end * k / steps, u_at(end * k / steps)) for k in range(steps + 1)]


def polyline(points: list):
    """Arc length along the corners `points`, and the point at a given length."""
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.dist(a, b))

    def at(s: float):
        for k in range(1, len(points)):
            if s <= lengths[k] or k == len(points) - 1:
                a, b = points[k - 1], points[k]
                u = 0.0 if lengths[k] == lengths[k - 1] else (s - lengths[k - 1]) / (lengths[k] - lengths[k - 1])
                return tuple(p + (q - p) * min(max(u, 0.0), 1.0) for p, q in zip(a, b))
        return points[-1]

    return lengths[-1], at


def row(t: float, xyz, rxyz=TOOL) -> list:
    return [t, *xyz, *rxyz]


def through(corners: list, held: bool) -> list:
    """Straight lines between `corners` (offsets from CENTRE [mm]), each from
    rest to rest; held at every corner for PAUSE if `held`."""
    rows, t = [], 0.0
    for k in range(1, len(corners)):
        length, at = polyline([corners[k - 1], corners[k]])
        stretch = timed(length, SPEED, t)
        rows += [row(ti, offset(at(s))) for ti, s in (stretch if not rows else stretch[1:])]
        t = stretch[-1][0]
        if held and k + 1 < len(corners):  # the same pose again, later: a pause
            t += PAUSE
            rows.append(row(t, offset(corners[k])))
    return rows


def along(curve, samples: int = 2000) -> list:
    """A smooth curve: `curve(v)` for v in [0, 1] gives an offset from CENTRE
    [mm]; timed by arc length at SPEED, from rest to rest."""
    points = [curve(k / samples) for k in range(samples + 1)]
    length, at = polyline(points)
    return [row(t, offset(at(s))) for t, s in timed(length, SPEED, 0.0)]


def offset(d) -> tuple:
    return tuple(c + x for c, x in zip(CENTRE, d))


# ------------------------------------------------------------------ shapes

def star() -> list:
    """A five-pointed star (a pentagram), 60 mm from the centre to each point,
    drawn in one stroke; held at each point."""
    r = 60.0
    tip = [(r * math.sin(2 * math.pi * k / 5), r * math.cos(2 * math.pi * k / 5), 0.0) for k in range(5)]
    return through([tip[0], tip[2], tip[4], tip[1], tip[3], tip[0]], held=True)


def square() -> list:
    """A 60 mm square, starting at the centre's corner; held at each corner."""
    s = 60.0
    return through([(0, 0, 0), (s, 0, 0), (s, s, 0), (0, s, 0), (0, 0, 0)], held=True)


def circle() -> list:
    """Two laps of a 60 mm circle, starting on its right."""
    r = 60.0
    return along(lambda v: (r * math.cos(4 * math.pi * v), r * math.sin(4 * math.pi * v), 0.0))


def figure_eight() -> list:
    """A figure eight 120 mm wide and 60 mm high, from the centre and back."""
    a = 60.0
    return along(lambda v: (a * math.sin(2 * math.pi * v), a * math.sin(2 * math.pi * v) * math.cos(2 * math.pi * v), 0.0))


def helix() -> list:
    """Three turns of a 40 mm circle, rising 60 mm (from 30 mm below the centre)."""
    r, rise, turns = 40.0, 60.0, 3
    return along(lambda v: (r * math.cos(2 * math.pi * turns * v), r * math.sin(2 * math.pi * turns * v), -rise / 2 + rise * v))


def cone() -> list:
    """The TCP held at the centre while the tool tilts 10 deg and its axis goes
    once round a cone: a check of the tool's TCP (its tip should not move)."""
    tilt = 10.0
    base = poses.rotation("ZYX", *(math.radians(a) for a in TOOL))

    def tilted(angle: float, around: float) -> tuple:
        # The tool turned by `angle` about the base-frame axis in the XY plane at `around` + 90 deg.
        r = poses.matmul(poses.axis_rotation("Z", math.radians(around)),
                         poses.matmul(poses.axis_rotation("Y", math.radians(angle)),
                                      poses.axis_rotation("Z", -math.radians(around))))
        return tuple(math.degrees(a) for a in poses.angles("ZYX", poses.matmul(r, base)))

    rows, t = [], 0.0
    for t_, u in timed(tilt, TURN_RATE, t):  # tilt in
        rows.append(row(t_, CENTRE, tilted(u, 0.0)))
    t = rows[-1][0] + PAUSE
    rows.append(row(t, CENTRE, tilted(tilt, 0.0)))
    for t_, u in timed(360.0, TURN_RATE, t)[1:]:  # once round
        rows.append(row(t_, CENTRE, tilted(tilt, u)))
    t = rows[-1][0] + PAUSE
    rows.append(row(t, CENTRE, tilted(tilt, 360.0)))
    for t_, u in timed(tilt, TURN_RATE, t)[1:]:  # tilt out
        rows.append(row(t_, CENTRE, tilted(tilt - u, 360.0)))
    return rows


SHAPES = {"star": star, "square": square, "circle": circle, "figure_eight": figure_eight, "helix": helix,
          "cone": cone}


def csv_text(name: str, rows: list) -> str:
    doc = SHAPES[name].__doc__.split("\n")
    head = [f"# {name}: " + " ".join(line.strip() for line in doc).strip(),
            f"# Around {CENTRE} mm with the tool at {TOOL} deg (examples/shapes.py writes this file; set both",
            "# there from the pendant). Run `ec_play FILE` first: it plans and prints, nothing moves.",
            "t,x,y,z,rx,ry,rz"]
    body = [",".join(f"{v:.3f}" if k == 0 else f"{v:.4f}" for k, v in enumerate(r)) for r in rows]
    return "\n".join(head + body) + "\n"


def write(directory: Path) -> list:
    """Writes every shape's CSV into `directory`; their paths."""
    written = []
    for name, shape in SHAPES.items():
        path = directory / f"{name}.csv"
        path.write_text(csv_text(name, shape()), encoding="utf-8")
        written.append(path)
    return written


if __name__ == "__main__":
    for path in write(Path(__file__).resolve().parent):
        print(path)
