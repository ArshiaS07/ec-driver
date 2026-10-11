"""Pose arithmetic for the development tools. Standard library only.

A pose is [x, y, z, rx, ry, rz] with angles in radians. An "order" names how
rx, ry, rz compose into a rotation matrix: order "ZYX" means
R = Rz(rz) . Ry(ry) . Rx(rx), that is, rotate about the fixed X axis, then the
fixed Y axis, then the fixed Z axis. All six Tait-Bryan orders are supported.
"""

import math

ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")
_INDEX = {"X": 0, "Y": 1, "Z": 2}


def axis_rotation(axis: str, angle: float) -> list:
    c, s = math.cos(angle), math.sin(angle)
    if axis == "X":
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    if axis == "Y":
        return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def matmul(a: list, b: list) -> list:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a: list, v: list) -> list:
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a: list) -> list:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rotation(order: str, rx: float, ry: float, rz: float) -> list:
    """R = R_order[0] . R_order[1] . R_order[2], each about its own axis's angle."""
    angle = {"X": rx, "Y": ry, "Z": rz}
    r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for axis in order:
        r = matmul(r, axis_rotation(axis, angle[axis]))
    return r


def angles(order: str, r: list) -> list:
    """rx, ry, rz such that rotation(order, rx, ry, rz) == r."""
    i, j, k = (_INDEX[a] for a in order)
    sign = 1.0 if (j - i) % 3 == 1 else -1.0  # +1 for XYZ, YZX, ZXY
    middle = math.asin(max(-1.0, min(1.0, sign * r[i][k])))
    first = math.atan2(-sign * r[j][k], r[k][k])
    last = math.atan2(-sign * r[i][j], r[i][i])
    out = {order[0]: first, order[1]: middle, order[2]: last}
    return [out["X"], out["Y"], out["Z"]]


def compose(order: str, p1: list, p2: list) -> list:
    """T1 . T2: pose p2 expressed in the frame of pose p1."""
    r1 = rotation(order, *p1[3:])
    r2 = rotation(order, *p2[3:])
    position = [a + b for a, b in zip(p1[:3], matvec(r1, p2[:3]))]
    return position + angles(order, matmul(r1, r2))


def invert(order: str, p: list) -> list:
    rt = transpose(rotation(order, *p[3:]))
    return [-v for v in matvec(rt, p[:3])] + angles(order, rt)


def to_radians(pose: list) -> list:
    return pose[:3] + [math.radians(a) for a in pose[3:]]


def to_degrees(pose: list) -> list:
    return pose[:3] + [math.degrees(a) for a in pose[3:]]
