"""Kinematics of an EC66-like arm, for the fake controller. Standard library only.

A six-joint arm in the layout of the EC series (and Universal Robots):
standard Denavit-Hartenberg parameters, the first link offset d1 as recorded
from the EA66 (getDH index 0), the others typical of the EC66. Not the robot's
calibration: the fake only needs a model that is consistent with itself.

Units at this module's edge are the controller's: joints in degrees, poses as
[x, y, z, rx, ry, rz] in mm and radians with R = Rz(rz) . Ry(ry) . Rx(rx).
"""

import math

import poses

# Standard DH: a [mm], alpha [rad], d [mm], one per joint; theta is the joint.
A = [0.0, -418.0, -398.0, 0.0, 0.0, 0.0]
ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
D = [95.431, 0.0, 0.0, 122.0, 98.0, 89.0]


def _link(i: int, theta: float) -> list:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(ALPHA[i]), math.sin(ALPHA[i])
    return [[ct, -st * ca, st * sa, A[i] * ct],
            [st, ct * ca, -ct * sa, A[i] * st],
            [0.0, sa, ca, D[i]],
            [0.0, 0.0, 0.0, 1.0]]


def _mul(a: list, b: list) -> list:
    return [[a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j] for j in range(4)]
            for i in range(4)]


def frames(q_rad: list) -> list:
    """The base frame and the frame after each joint, as 4x4 transforms."""
    out = [[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]]
    for i, theta in enumerate(q_rad):
        out.append(_mul(out[-1], _link(i, theta)))
    return out


def forward(q_deg: list) -> list:
    """The flange pose [mm, rad] of joint positions [deg]."""
    t = frames([math.radians(v) for v in q_deg])[-1]
    r = [row[:3] for row in t[:3]]
    return [t[0][3], t[1][3], t[2][3]] + poses.angles("ZYX", r)


def _solve6(m: list, b: list) -> list:
    """x with m x = b, by Gaussian elimination with partial pivoting; None if singular."""
    n = len(b)
    a = [row[:] + [b[i]] for i, row in enumerate(m)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(a[r][c]))
        if abs(a[p][c]) < 1e-15:
            return None
        a[c], a[p] = a[p], a[c]
        for r in range(c + 1, n):
            f = a[r][c] / a[c][c]
            if f != 0.0:
                for k in range(c, n + 1):
                    a[r][k] -= f * a[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (a[r][n] - sum(a[r][k] * x[k] for k in range(r + 1, n))) / a[r][r]
    return x


def _cross(a: list, b: list) -> list:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def inverse(pose: list, reference_deg: list, iterations: int = 100) -> list:
    """Joint positions [deg] for a flange pose [mm, rad], found from `reference`
    by damped least squares, so the solution is the one near it; None if it
    does not converge (out of reach, or a singularity in the way)."""
    target_p = [v / 1000.0 for v in pose[:3]]
    target_r = poses.rotation("ZYX", *pose[3:])
    q = [math.radians(v) for v in reference_deg]
    damping = 1e-4
    for _ in range(iterations):
        f = frames(q)
        t = f[-1]
        p = [t[0][3] / 1000.0, t[1][3] / 1000.0, t[2][3] / 1000.0]
        r = [row[:3] for row in t[:3]]
        e_p = [a - b for a, b in zip(target_p, p)]
        e_o = [0.0, 0.0, 0.0]  # half the sum of the column cross products: the small-angle rotation error
        for c in range(3):
            col, goal = [r[i][c] for i in range(3)], [target_r[i][c] for i in range(3)]
            e_o = [x + 0.5 * y for x, y in zip(e_o, _cross(col, goal))]
        if max(abs(v) for v in e_p) < 1e-9 and max(abs(v) for v in e_o) < 1e-9:
            return [math.degrees(v) for v in q]
        jac = []  # 6 x 6, rows: vx vy vz wx wy wz
        for i in range(6):
            z = [f[i][0][2], f[i][1][2], f[i][2][2]]
            o = [f[i][0][3] / 1000.0, f[i][1][3] / 1000.0, f[i][2][3] / 1000.0]
            jac.append(_cross(z, [a - b for a, b in zip(p, o)]) + z)
        jac = [[jac[c][r] for c in range(6)] for r in range(6)]  # columns were joints
        e = e_p + e_o
        jjt = [[sum(jac[i][k] * jac[j][k] for k in range(6)) + (damping if i == j else 0.0) for j in range(6)]
               for i in range(6)]
        y = _solve6(jjt, e)
        if y is None:
            return None
        dq = [sum(jac[k][i] * y[k] for k in range(6)) for i in range(6)]
        step = max(abs(v) for v in dq)
        if step > 0.2:  # at most about 11 degrees per iteration
            dq = [v * 0.2 / step for v in dq]
        q = [a + b for a, b in zip(q, dq)]
    return None
