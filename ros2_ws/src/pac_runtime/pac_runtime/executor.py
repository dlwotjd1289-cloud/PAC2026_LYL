"""Stage 7 (virtual): execute a placement, then verify it from the top view.

The virtual robot holds the box at the centre of its top face (vacuum), so
the true box lands centred on the planned TCP point plus a small placement
error, with its *true* size. It settles on whatever is really under it.
The post check compares the planned and the measured box (heightmap before
/ after) and escalates (flowchart "편차·실패 발생 시"):

- L0: within the L0 tolerance -> done
- L1: small deviation -> accepted, the state takes the measured pose
- L2: larger deviation but still safe -> accepted with the measured pose;
  plans made on the old state are stale (partial replan of what it touches)
- L3: grip failure -> retry, then the other grasp (gripper yaw + pi), then
  the box goes to the inspection area
- L4: unsafe result (penetration, protrusion, too little support, height)
  -> HOLD, operator corrects it (the box is put at the planned pose)
"""

from dataclasses import dataclass
import math

from pac_common import Pose3D
from pac_candidates.geometry import rotated_dims


@dataclass
class TrueBox:
    box_id: str
    size: object      # true Size3D
    pose: Pose3D      # true min corner


@dataclass
class Execution:
    grip_ok: bool
    attempts: int
    other_grasp: bool
    level: str = ""             # L0..L4 or "L3" for a failed grip
    true_pose: Pose3D = None
    measured_pose: Pose3D = None
    dxy_m: float = 0.0
    dz_m: float = 0.0
    issues: tuple = ()
    time_s: float = 0.0


def _bounds(size, pose):
    dx, dy, dz = rotated_dims(size, pose.yaw)
    return (pose.x, pose.y, pose.z), (pose.x + dx, pose.y + dy, pose.z + dz)


def _overlap_1d(a0, a1, b0, b1):
    return min(a1, b1) - max(a0, b0)


class ExecutorSim:
    def __init__(self, exec_config, verify_config, rng):
        self.cfg = exec_config
        self.vc = verify_config
        self.rng = rng

    def grip(self):
        """Returns (ok, attempts, used the other grasp)."""
        c = self.cfg
        for attempt in range(1, c.max_grip_retries + 3):
            if self.rng.random() >= c.grip_fail_probability:
                return True, attempt, attempt > c.max_grip_retries + 1
        return False, c.max_grip_retries + 2, True

    def settle_z(self, lo, hi, stack):
        z = 0.0
        for tb in stack:
            (bx0, by0, _), (bx1, by1, bz1) = _bounds(tb.size, tb.pose)
            if _overlap_1d(lo[0], hi[0], bx0, bx1) > 1e-6 and _overlap_1d(lo[1], hi[1], by0, by1) > 1e-6:
                z = max(z, bz1)
        return z

    def check(self, true_size, pose, stack, pallet_size):
        """True-geometry safety of a settled box: penetration, protrusion,
        height and support ratio."""
        lo, hi = _bounds(true_size, pose)
        issues = []
        for tb in stack:
            blo, bhi = _bounds(tb.size, tb.pose)
            pen = min(_overlap_1d(lo[i], hi[i], blo[i], bhi[i]) for i in range(3))
            if pen > self.vc.max_overlap_m:
                issues.append(f"PENETRATION:{tb.box_id}")
        m = self.vc.max_protrusion_m
        if lo[0] < -m or lo[1] < -m or hi[0] > pallet_size.x + m or hi[1] > pallet_size.y + m:
            issues.append("PROTRUSION")
        if hi[2] > pallet_size.z + 0.002:
            issues.append("HEIGHT_LIMIT")
        area = (hi[0] - lo[0]) * (hi[1] - lo[1])
        if pose.z > 1e-6:
            sup = 0.0
            for tb in stack:
                blo, bhi = _bounds(tb.size, tb.pose)
                if abs(bhi[2] - pose.z) <= 0.005:
                    ox = _overlap_1d(lo[0], hi[0], blo[0], bhi[0])
                    oy = _overlap_1d(lo[1], hi[1], blo[1], bhi[1])
                    if ox > 0 and oy > 0:
                        sup += ox * oy
            if sup / area < self.vc.min_support_ratio:
                issues.append(f"LOW_SUPPORT:{sup / area:.2f}")
        return tuple(issues)

    def place(self, measured_box, true_size, candidate, stack, pallet_size):
        """Execute after a successful grip. Returns an ``Execution``."""
        planned = candidate.target_pose
        mdx, mdy, mdz = rotated_dims(measured_box.size, planned.yaw)
        cx, cy = planned.x + mdx / 2, planned.y + mdy / 2
        ex = self.rng.gauss(0.0, self.cfg.place_xy_noise_std_m)
        ey = self.rng.gauss(0.0, self.cfg.place_xy_noise_std_m)
        tdx, tdy, tdz = rotated_dims(true_size, planned.yaw)
        tx, ty = cx + ex - tdx / 2, cy + ey - tdy / 2
        z = self.settle_z((tx, ty), (tx + tdx, ty + tdy), stack)
        true_pose = Pose3D("pallet", tx, ty, z, yaw=planned.yaw)
        issues = self.check(true_size, true_pose, stack, pallet_size)
        # top view measures where the box really is; the state keeps the
        # measured footprint centred on it (measured size)
        measured = Pose3D("pallet", cx + ex - mdx / 2, cy + ey - mdy / 2, z, yaw=planned.yaw)
        dxy = math.hypot(ex, ey)
        dz = abs(z - planned.z)
        v = self.vc
        if issues:
            level = "L4"
        elif dxy <= v.l0_xy_m and dz <= v.l0_z_m:
            level = "L0"
        elif dxy <= v.l1_xy_m and dz <= v.l1_z_m:
            level = "L1"
        else:
            level = "L2"
        return Execution(True, 1, False, level, true_pose, measured, dxy, dz, issues)
