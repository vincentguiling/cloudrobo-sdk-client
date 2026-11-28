"""Pose math shared by the operator mappings (port of core/PoseMath.cs).

A pose is (position [x,y,z], quaternion [x,y,z,w]) in the pose frame used
by TeleopFrame's Pose7D: [tx,ty,tz,qx,qy,qz,qw].
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence


@dataclass
class Pose:
    p: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    q: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])

    def copy(self) -> "Pose":
        return Pose(list(self.p), list(self.q))

    def as_tuple(self) -> tuple[list[float], list[float]]:
        return (list(self.p), list(self.q))


IDENTITY = Pose()


def qmul(a: Sequence[float], b: Sequence[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def qconj(q: Sequence[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def qnorm(q: Sequence[float]) -> list[float]:
    n = math.sqrt(sum(x * x for x in q)) or 1.0
    return [x / n for x in q]


def qrot(q: Sequence[float], v: Sequence[float]) -> list[float]:
    """Rotate vector v by quaternion q (v' = q (v,0) q*)."""
    qv = [q[0], q[1], q[2]]
    t = [
        2 * (qv[1] * v[2] - qv[2] * v[1]),
        2 * (qv[2] * v[0] - qv[0] * v[2]),
        2 * (qv[0] * v[1] - qv[1] * v[0]),
    ]
    return [
        v[0] + q[3] * t[0] + (qv[1] * t[2] - qv[2] * t[1]),
        v[1] + q[3] * t[1] + (qv[2] * t[0] - qv[0] * t[2]),
        v[2] + q[3] * t[2] + (qv[0] * t[1] - qv[1] * t[0]),
    ]


def compose(a: Pose, b: Pose) -> Pose:
    """T_a * T_b: position a.p + R(a.q)*b.p, rotation a.q ⊗ b.q."""
    rq = qmul(qnorm(a.q), qnorm(b.q))
    rp = qrot(a.q, b.p)
    return Pose([a.p[i] + rp[i] for i in range(3)], rq)


def inverse(p: Pose) -> Pose:
    qi = qconj(qnorm(p.q))
    return Pose([-x for x in qrot(qi, p.p)], qi)


def clamp_reach(anchor: Sequence[float], p: Sequence[float], limit: float) -> list[float]:
    """Clamp p into the sphere of radius `limit` around anchor."""
    d = [p[i] - anchor[i] for i in range(3)]
    n = math.sqrt(sum(x * x for x in d))
    if n <= limit or n < 1e-9:
        return [p[0], p[1], p[2]]
    return [anchor[i] + d[i] * limit / n for i in range(3)]


def ema(prev: Pose, new: Pose, alpha: float) -> Pose:
    if alpha >= 1.0:
        return new
    return Pose(
        [prev.p[i] + (new.p[i] - prev.p[i]) * alpha for i in range(3)],
        qnorm([prev.q[i] + (new.q[i] - prev.q[i]) * alpha for i in range(4)]),
    )


def euler_to_quaternion(rx: float, ry: float, rz: float, order: str = "xyz") -> list[float]:
    """Intrinsic euler angles (radians) -> quaternion [x,y,z,w]."""
    q = [0.0, 0.0, 0.0, 1.0]
    for axis in (order or "xyz").lower():
        half = {"x": rx, "y": ry, "z": rz}.get(axis, 0.0) / 2.0
        s, c = math.sin(half), math.cos(half)
        if axis == "x":
            dq = [s, 0.0, 0.0, c]
        elif axis == "y":
            dq = [0.0, s, 0.0, c]
        else:
            dq = [0.0, 0.0, s, c]
        q = qmul(q, dq)
    return qnorm(q)


def pose7d(p: Pose) -> list[float]:
    return [p.p[0], p.p[1], p.p[2], p.q[0], p.q[1], p.q[2], p.q[3]]


def pose_from_7d(values: Sequence[float]) -> Pose:
    return Pose([values[0], values[1], values[2]], [values[3], values[4], values[5], values[6]])


def quat_continuity_fix(q: Sequence[float], prev: Sequence[float] | None) -> list[float]:
    """Pick q or -q so that it stays on the same hemisphere as prev (signs)."""
    if prev is None:
        return [q[0], q[1], q[2], q[3]]
    dot = sum(q[i] * prev[i] for i in range(4))
    if dot < 0:
        return [-q[0], -q[1], -q[2], -q[3]]
    return [q[0], q[1], q[2], q[3]]