"""HDR50-22 forward / inverse kinematics from the official URDF.

Joint table copied from HD Hyundai Robotics ``hdr_description`` (branch
``humble``, commit f026592, ``urdf/robots/hdr50_22/hdr50_22.urdf.xacro``):
origins, axes and position / velocity limits are taken verbatim; nothing is
inferred. ``tool0`` is ``flange`` rotated by pitch +pi/2 (its z axis points
out of the flange), as in the same URDF.

Frames: ``base`` is the robot ``base_link``. IK is closed form (shoulder
offset + two links + spherical wrist, up to 8 branches) with the joint
offsets read from the same table, every branch verified by forward
kinematics and polished by a few damped-least-squares steps.
"""

from dataclasses import dataclass
import math

import numpy as np

URDF_SOURCE = "hyundai-robotics/hdr_description@f026592 urdf/robots/hdr50_22/hdr50_22.urdf.xacro"


@dataclass(frozen=True)
class Joint:
    name: str
    xyz: tuple
    rpy: tuple
    axis: tuple
    lower: float
    upper: float
    velocity: float  # rad/s


HDR50_22 = (
    Joint("j1", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 0, 1), -3.142, 3.142, 3.054),
    Joint("j2", (0.32, 0.0, 0.48), (0.0, 1.5708, 0.0), (0, -1, 0), -1.134, 2.967, 3.054),
    Joint("j3", (0.0, 0.0, 0.87), (0.0, 0.0, 0.0), (0, -1, 0), -1.396, 3.141, 3.054),
    Joint("j4", (0.0, 0.0, 0.2), (0.0, 0.0, 0.0), (1, 0, 0), -6.283, 6.283, 4.363),
    Joint("j5", (1.03, 0.0, 0.0), (0.0, 0.0, 0.0), (0, -1, 0), -2.182, 2.182, 4.363),
    Joint("j6", (0.185, 0.0, 0.0), (0.0, 0.0, 0.0), (1, 0, 0), -6.283, 6.283, 6.109),
)
TOOL0_RPY = (0.0, 1.5708, 0.0)


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rot_axis(axis, q):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * (k @ k)


def transform(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


_ORIGINS = [transform(rot_rpy(*j.rpy), j.xyz) for j in HDR50_22]
_TOOL0 = transform(rot_rpy(*TOOL0_RPY))
LOWER = np.array([j.lower for j in HDR50_22])
UPPER = np.array([j.upper for j in HDR50_22])
VELOCITY = np.array([j.velocity for j in HDR50_22])


def joint_frames(q, tcp_offset=0.0):
    """Frames after each joint (6), then tool0 and the TCP (``tcp_offset``
    along tool0 z)."""
    T = np.eye(4)
    frames = []
    for origin, joint, qi in zip(_ORIGINS, HDR50_22, q):
        T = T @ origin @ transform(rot_axis(joint.axis, qi))
        frames.append(T)
    tool0 = T @ _TOOL0
    frames.append(tool0)
    frames.append(tool0 @ transform(p=(0.0, 0.0, tcp_offset)))
    return frames


def fk(q, tcp_offset=0.0):
    return joint_frames(q, tcp_offset)[-1]


def _error(T, target):
    dp = target[:3, 3] - T[:3, 3]
    Re = target[:3, :3] @ T[:3, :3].T
    angle = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1.0, 1.0))
    if angle < 1e-9:
        dw = np.zeros(3)
    else:
        dw = angle / (2 * np.sin(angle)) * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return np.concatenate([dp, dw])


def _jacobian(q, tcp_offset):
    frames = joint_frames(q, tcp_offset)
    p_end = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i, joint in enumerate(HDR50_22):
        T = frames[i]
        axis = T[:3, :3] @ np.asarray(joint.axis, dtype=float)
        J[:3, i] = np.cross(axis, p_end - T[:3, 3])
        J[3:, i] = axis
    return J


# geometry used by the closed-form solution (all from the joint table)
_SHOULDER = (HDR50_22[1].xyz[0], HDR50_22[1].xyz[2])     # radial offset, height of j2
_L2 = HDR50_22[2].xyz[2]                                 # j2 -> j3
_FORE = (HDR50_22[3].xyz[2], HDR50_22[4].xyz[0])         # j3 -> j4 (along the arm), j4 -> j5
_L3 = math.hypot(*_FORE)
_BETA = math.atan2(-_FORE[1], _FORE[0])
_WRIST_TO_FLANGE = HDR50_22[5].xyz[0]


def _wrap_near(angle, ref, lo, hi):
    """``angle`` + k*2pi closest to ``ref`` inside [lo, hi] (or None)."""
    best = None
    for k in range(-2, 3):
        a = angle + 2 * math.pi * k
        if lo <= a <= hi and (best is None or abs(a - ref) < abs(best - ref)):
            best = a
    return best


def ik_all(target, tcp_offset=0.0, ref=None):
    """All closed-form branches (inside the joint limits) for ``target``."""
    ref = np.zeros(6) if ref is None else np.asarray(ref, dtype=float)
    R = target[:3, :3]
    z_tool = R[:, 2]
    w = target[:3, 3] - (_WRIST_TO_FLANGE + tcp_offset) * z_tool
    rho = math.hypot(w[0], w[1])
    out = []
    for flip in (False, True):
        q1 = math.atan2(w[1], w[0]) + (math.pi if flip else 0.0)
        q1 = _wrap_near(q1, ref[0], LOWER[0], UPPER[0])
        if q1 is None:
            continue
        r = (-rho if flip else rho) - _SHOULDER[0]
        s = w[2] - _SHOULDER[1]
        c = (r * r + s * s - _L2 * _L2 - _L3 * _L3) / (2 * _L2 * _L3)
        if abs(c) > 1.0:
            continue
        for sign in (1.0, -1.0):
            e = sign * math.acos(c)
            q2 = math.atan2(s, r) - math.atan2(_L3 * math.sin(e), _L2 + _L3 * math.cos(e))
            q3 = e - _BETA
            q2 = _wrap_near(q2, ref[1], LOWER[1], UPPER[1])
            q3 = _wrap_near(q3, ref[2], LOWER[2], UPPER[2])
            if q2 is None or q3 is None:
                continue
            q = np.array([q1, q2, q3, 0.0, 0.0, 0.0])
            R03 = joint_frames(q)[3][:3, :3]  # after j4 at q4 = 0
            M = R03.T @ R @ _TOOL0[:3, :3].T  # = Rx(q4) Ry(-q5) Rx(q6)
            b0 = math.acos(max(-1.0, min(1.0, M[0, 0])))
            for b in ((b0, -b0) if b0 > 1e-6 else (0.0,)):
                sb = math.sin(b)
                if abs(sb) > 1e-6:
                    a = math.atan2(M[1, 0] / sb, -M[2, 0] / sb)
                    cc = math.atan2(M[0, 1] / sb, M[0, 2] / sb)
                else:  # wrist singular: put the whole roll on j6
                    a = ref[3]
                    cc = math.atan2(M[2, 1], M[1, 1]) - a
                q4 = _wrap_near(a, ref[3], LOWER[3], UPPER[3])
                q5 = -b
                q6 = _wrap_near(cc, ref[5], LOWER[5], UPPER[5])
                if q4 is None or q6 is None or not (LOWER[4] <= q5 <= UPPER[4]):
                    continue
                sol = np.array([q1, q2, q3, q4, q5, q6])
                sol = _polish(sol, target, tcp_offset)
                if sol is not None:
                    out.append(sol)
    return out


def _polish(q, target, tcp_offset, tol_pos=1e-4, tol_rot=1e-3, iters=20, damping=0.01):
    for _ in range(iters):
        e = _error(fk(q, tcp_offset), target)
        if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
            return q if np.all(q >= LOWER) and np.all(q <= UPPER) else None
        J = _jacobian(q, tcp_offset)
        q = q + J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), e)
    return None


def ik(target, tcp_offset=0.0, seeds=(), **_):
    """Branch closest to the first seed (joint-space distance), or None."""
    seeds = [np.asarray(s, dtype=float) for s in seeds if s is not None]
    ref = seeds[0] if seeds else np.zeros(6)
    sols = ik_all(target, tcp_offset, ref)
    if not sols:
        return None
    return min(sols, key=lambda q: float(np.max(np.abs(q - ref))))
