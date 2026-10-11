#!/usr/bin/env python3
"""Finds the controller's rotation convention, using only its pose arithmetic.

Sends poseMul and poseInv (pure computations: nothing moves) through ec_query,
then tests every hypothesis: the axis order in which rx, ry, rz compose (six
orders) and the order in which poseMul multiplies (T1.T2 or T2.T1). Results are
compared as rotation matrices, so angle wrap-around at +-180 deg cannot mislead.

    python3 tools/rotation_convention.py --ec-query ./build/linux-debug/apps/ec_query [--host H] [--port N]

Exit codes: 0 exactly one hypothesis fits; 1 none or several; 2 a call failed.
Standard library only.
"""

import argparse
import json
import math
import random
import subprocess
import sys
from pathlib import Path

import poses

POSITION_TOLERANCE = 0.05  # same unit as the poses (mm)
MATRIX_TOLERANCE = 1e-3    # per element; the controller rounds its angles

# The three hand-picked probes [label, pose1, pose2], angles in degrees:
# A tells T1.T2 from T2.T1, B and C tell Rz.Ry.Rx from Rx.Ry.Rz.
PROBES = [
    ("A", [100, 0, 0, 0, 0, 90], [10, 0, 0, 0, 0, 0]),
    ("B", [0, 0, 0, 0, 10, 30], [0, 0, 0, 20, 0, 0]),
    ("C", [0, 0, 0, 20, 0, 0], [0, 0, 0, 0, 10, 30]),
]


def random_pose(rng: random.Random) -> list:
    return [round(rng.uniform(-300, 300), 3) for _ in range(3)] + [round(rng.uniform(-170, 170), 3) for _ in range(3)]


def call(args: argparse.Namespace, method: str, **params) -> list:
    command = [str(args.ec_query)]
    if args.host:
        command += ["--host", args.host]
    if args.port:
        command += ["--port", str(args.port)]
    command += [method] + [f"{k}={json.dumps(v)}" for k, v in params.items()]
    done = subprocess.run(command, capture_output=True, text=True, timeout=10)
    if done.returncode != 0:
        print(f"{method} failed: {done.stderr.strip()}", file=sys.stderr)
        sys.exit(2)
    return json.loads(done.stdout)


def pose_error(order: str, expected: list, got: list) -> float:
    """How far `got` is from `expected` (both in radians), as the larger of the
    position error over the tolerance and the matrix error over the tolerance."""
    position = max(abs(a - b) for a, b in zip(expected[:3], got[:3]))
    r1 = poses.rotation(order, *expected[3:])
    r2 = poses.rotation(order, *got[3:])
    matrix = max(abs(r1[i][j] - r2[i][j]) for i in range(3) for j in range(3))
    return max(position / POSITION_TOLERANCE, matrix / MATRIX_TOLERANCE)


def describe(order: str) -> str:
    product = ".".join(f"R{a.lower()}(r{a.lower()})" for a in order)
    fixed = ", then ".join(f"fixed {a}" for a in reversed(order))
    return f"R = {product}: rotate about the {fixed} axis"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ec-query", type=Path, required=True)
    p.add_argument("--host", help="passed to ec_query (default: ec_query's own)")
    p.add_argument("--port", type=int)
    args = p.parse_args()

    rng = random.Random(8055)
    probes = PROBES + [(f"R{i}", random_pose(rng), random_pose(rng)) for i in range(1, 7)]
    inversions = [random_pose(rng) for _ in range(3)]

    print("poseMul probes (degrees):")
    multiplied = []
    for label, pose1, pose2 in probes:
        got = call(args, "poseMul", pose1=pose1, pose2=pose2, unit_type=0)
        multiplied.append((pose1, pose2, got))
        if not label.startswith("R"):
            print(f"  {label}: poseMul({pose1}, {pose2}) = {[round(v, 4) for v in got]}")
    inverted = [(pose, call(args, "poseInv", pose=pose, unit_type=0)) for pose in inversions]

    results = []
    for order in poses.ORDERS:
        for product in ("T1.T2", "T2.T1"):
            worst = 0.0
            for pose1, pose2, got in multiplied:
                a, b = poses.to_radians(pose1), poses.to_radians(pose2)
                expected = poses.compose(order, a, b) if product == "T1.T2" else poses.compose(order, b, a)
                worst = max(worst, pose_error(order, expected, poses.to_radians(got)))
            for pose, got in inverted:
                worst = max(worst, pose_error(order, poses.invert(order, poses.to_radians(pose)), poses.to_radians(got)))
            results.append((worst, order, product))

    results.sort()
    print("\nhypotheses, best first (error 1.0 = at the tolerance):")
    for worst, order, product in results[:4]:
        print(f"  {order}  poseMul = {product}   error {worst:.3g}")

    fitting = [(order, product) for worst, order, product in results if worst <= 1.0]
    if len(fitting) == 1:
        order, product = fitting[0]
        print(f"\nconvention: {describe(order)}")
        print(f"poseMul(pose1, pose2) = {product}" + (" (pose2 expressed in pose1's frame)" if product == "T1.T2" else ""))
        sys.exit(0)
    print(f"\n{len(fitting)} hypotheses fit; the convention is not a Tait-Bryan order of rx, ry, rz as tested."
          if not fitting else f"\nseveral hypotheses fit: {fitting}")
    sys.exit(1)


if __name__ == "__main__":
    main()
