"""Stage 6: robot feasibility of a placement candidate on the HDR50-22.

Order follows the runtime flowchart: descent path of box + gripper ->
reach / IK -> arm collision -> payload. The call matches the contract in
donghan's ``docs/integration.md``::

    verdict = robot.validate_robot_motion(box, candidate, latest_state, robot_state)

Frames: ``pallet`` (corner origin, z = 0 on the deck, as in pac_common),
robot ``base`` (URDF ``base_link``). The box is held by a top-down vacuum
gripper at the centre of its top face; the gripper yaw follows the box yaw
(both 180 deg variants are tried). Approach and retreat are vertical.

What this checks and what it does not:
- joint limits and IK from the official URDF (exact),
- the gripper body against taller neighbours during the vertical descent,
- the arm as capsules against the placed boxes, the pallet deck and the floor
  (radii are assumptions; MoveIt mesh collision remains the final word),
- payload = box + gripper against the rated payload,
- not: dynamic torque limits, self-collision, the transfer path from the
  conveyor (only its end points and a joint-space time estimate).
"""

import math

import numpy as np

from pac_common import RejectCode, ValidationResult

from .config import RobotCheckConfig
from .kinematics import HDR50_22, LOWER, UPPER, VELOCITY, URDF_SOURCE, ik, ik_all, joint_frames, transform

HOME = np.array([0.0, 1.5707, 0.0, 0.0, 0.0, 0.0])  # hdr50_22_moveit_config "home"
DOWN = np.diag([1.0, -1.0, -1.0])  # tool z pointing down


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rotated(size, yaw):
    turns = int(round(yaw / (math.pi / 2))) % 2
    return (size.y, size.x, size.z) if turns else (size.x, size.y, size.z)


def _aabb(pose, size):
    dx, dy, dz = _rotated(size, pose.yaw)
    return np.array([pose.x, pose.y, pose.z]), np.array([pose.x + dx, pose.y + dy, pose.z + dz])


def _point_box_distance(points, lo, hi):
    d = np.maximum(np.maximum(lo - points, 0.0), points - hi)
    return np.linalg.norm(d, axis=-1)


class RobotFeasibility:
    def __init__(self, config=None):
        self.cfg = config or RobotCheckConfig()
        self._pick_q = {}

    # ------------------------------------------------------------------ frames
    def pallet_to_base(self, pallet_size):
        """4x4 transform mapping pallet-frame points into the robot base frame."""
        bx, by, bz, byaw = self.cfg.cell.base_from_pallet_center
        base_in_pallet = transform(_rz(byaw), (pallet_size.x / 2 + bx, pallet_size.y / 2 + by, bz))
        return np.linalg.inv(base_in_pallet)

    def _tcp_target(self, center_top, yaw, T_pb):
        return T_pb @ transform(_rz(yaw) @ DOWN, center_top)

    # ------------------------------------------------------------------ checks
    def payload(self, box):
        total = box.weight_kg + self.cfg.gripper.mass_kg
        return total, total <= self.cfg.rated_payload_kg + 1e-9

    @staticmethod
    def box_descent_clear(center_top, dims, placed):
        """The held box itself sweeps its footprint from the approach height
        down to its bottom: no placed box may rise above that bottom inside
        the footprint (5-2 checks this too; repeated so stage 6 stands alone)."""
        dx, dy, dz = dims
        bottom = center_top[2] - dz
        for b in placed:
            lo, hi = _aabb(b.pose, b.size)
            if hi[2] <= bottom + 1e-6:
                continue
            if lo[0] < center_top[0] + dx / 2 - 1e-6 and hi[0] > center_top[0] - dx / 2 + 1e-6 and \
               lo[1] < center_top[1] + dy / 2 - 1e-6 and hi[1] > center_top[1] - dy / 2 + 1e-6:
                return False, b.box_id
        return True, None

    def gripper_descent_clear(self, center_top, yaw, placed):
        """Gripper body (footprint rotated by ``yaw``) above the box top must
        not overlap any placed box that rises above the box top."""
        g = self.cfg.gripper
        hx, hy = g.footprint_m[0] / 2 + g.clearance_m, g.footprint_m[1] / 2 + g.clearance_m
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        ex, ey = hx * c + hy * s, hx * s + hy * c  # AABB of the rotated footprint
        top = center_top[2]
        worst = None
        for b in placed:
            lo, hi = _aabb(b.pose, b.size)
            if hi[2] <= top + 1e-6:
                continue
            if lo[0] < center_top[0] + ex and hi[0] > center_top[0] - ex and \
               lo[1] < center_top[1] + ey and hi[1] > center_top[1] - ey:
                worst = b.box_id
                break
        return worst is None, worst

    def _capsules(self, q):
        cfg = self.cfg.collision
        f = joint_frames(q, self.cfg.gripper.tcp_offset_m)
        o = [T[:3, 3] for T in f]
        # f[1] after j2 (shoulder), f[2] after j3 (elbow), f[3] after j4,
        # f[4] after j5 (wrist centre), f[6] tool0
        return [(o[1], o[2], cfg.upper_arm_radius_m), (o[2], o[3], cfg.forearm_radius_m),
                (o[3], o[4], cfg.forearm_radius_m), (o[4], o[6], cfg.wrist_radius_m)]

    def arm_clearance(self, q, T_bp, obstacles, pallet_size):
        """Smallest distance (m) from the arm capsules to the obstacles,
        negative when penetrating; plus the id of the closest obstacle."""
        step = self.cfg.collision.sample_step_m
        best, who = math.inf, None
        deck = self.cfg.cell.deck_height_m
        for k, (a, b, r) in enumerate(self._capsules(q)):
            n = max(2, int(np.linalg.norm(b - a) / step) + 1)
            pts_base = a + (b - a) * np.linspace(0.0, 1.0, n)[:, None]
            pts = (T_bp[:3, :3] @ pts_base.T).T + T_bp[:3, 3]  # pallet frame
            for box_id, lo, hi in obstacles:
                d = float(np.min(_point_box_distance(pts, lo, hi))) - r
                if d < best:
                    best, who = d, box_id
            if k == 0:
                continue  # the shoulder sits on the base: floor / deck are not reachable for it
            d = float(np.min(pts[:, 2] + deck)) - r  # floor
            if d < best:
                best, who = d, "floor"
            on_pallet = (pts[:, 0] > 0) & (pts[:, 0] < pallet_size.x) & (pts[:, 1] > 0) & (pts[:, 1] < pallet_size.y)
            if on_pallet.any():
                d = float(np.min(pts[on_pallet, 2])) - r  # pallet deck
                if d < best:
                    best, who = d, "pallet_deck"
        return best, who

    def _within_limits(self, q):
        m = self.cfg.motion.limit_margin_rad
        return bool(np.all(q >= LOWER + m) and np.all(q <= UPPER - m))

    def _paths(self, center_top, yaw, T_pb):
        """Vertical paths from the place pose up to the approach height, one
        per IK branch at the place pose. Yields (q list top -> bottom,
        failing code or None)."""
        mc = self.cfg.motion
        n = max(1, int(math.ceil(mc.approach_clearance_m / mc.path_step_m)))
        heights = [mc.approach_clearance_m * k / n for k in range(n + 1)]  # bottom -> top
        targets = [self._tcp_target(center_top + np.array([0.0, 0.0, dz]), yaw, T_pb) for dz in heights]
        branches = [q for q in ik_all(targets[0], self.cfg.gripper.tcp_offset_m, HOME) if self._within_limits(q)]
        if not branches:
            yield [], RejectCode.IK_FAIL
            return
        for q0 in branches:
            qs, code = [q0], None
            for target in targets[1:]:
                q = ik(target, self.cfg.gripper.tcp_offset_m, seeds=[qs[-1]])
                if q is None or not self._within_limits(q) or \
                        np.max(np.abs(q - qs[-1])) > mc.max_joint_jump_rad:
                    code = RejectCode.APPROACH_FAIL
                    break
                qs.append(q)
            yield qs[::-1], code

    # ------------------------------------------------------------------ pick
    def pick_configuration(self, box):
        """Joint vector over the conveyor pick point for ``box`` (cached by height)."""
        key = round(box.size.z, 3)
        if key not in self._pick_q:
            px, py, pz = self.cfg.cell.pick_point_base_m
            target = transform(DOWN, (px, py, pz + box.size.z + self.cfg.motion.approach_clearance_m))
            self._pick_q[key] = ik(target, self.cfg.gripper.tcp_offset_m, seeds=[_j1_seed(target), HOME])
        return self._pick_q[key]

    def pick_path(self, box_height):
        """(q above the pick point, q at the pick point) for a box of this
        height lying on the conveyor; None where unreachable."""
        px, py, pz = self.cfg.cell.pick_point_base_m
        tcp = self.cfg.gripper.tcp_offset_m
        top = pz + box_height
        up = ik(transform(DOWN, (px, py, top + self.cfg.motion.approach_clearance_m)), tcp,
                seeds=[_j1_seed(transform(DOWN, (px, py, top))), HOME])
        if up is None:
            return None, None
        down = ik(transform(DOWN, (px, py, top)), tcp, seeds=[up])
        return up, down

    def cycle_time(self, box, q_place_top, q_pick=None):
        """Estimated pick-and-place cycle (s): two vertical moves at each end,
        a joint-space transfer each way and the grip / release times."""
        mc = self.cfg.motion
        q_pick = self.pick_configuration(box) if q_pick is None else q_pick
        if q_pick is None:
            q_pick = HOME
        transfer = float(np.max(np.abs(np.asarray(q_place_top) - q_pick) / (VELOCITY * mc.speed_scale)))
        vertical = 4 * mc.approach_clearance_m / mc.linear_speed_mps
        return 2 * transfer + vertical + mc.grip_time_s + mc.release_time_s

    # ------------------------------------------------------------------ main
    def validate_robot_motion(self, box, candidate, state, robot_state=None):
        details = {"robot_model": self.cfg.robot_model, "kinematics": URDF_SOURCE}
        if candidate.base_state_version != state.state_version:
            return ValidationResult(False, (RejectCode.STALE_PLAN,), details)
        if candidate.target_pose.frame_id != "pallet":
            return ValidationResult(False, (RejectCode.INVALID_STATE,), {**details, "frame": candidate.target_pose.frame_id})
        total, ok = self.payload(box)
        details["payload_kg"] = round(total, 3)
        details["rated_payload_kg"] = self.cfg.rated_payload_kg
        if not ok:
            return ValidationResult(False, (RejectCode.PAYLOAD_EXCEEDED,), details)

        pose = candidate.target_pose
        dx, dy, dz = _rotated(box.size, pose.yaw)
        center_top = np.array([pose.x + dx / 2, pose.y + dy / 2, pose.z + dz])
        details["load_com_below_tcp_m"] = round(dz / 2, 4)
        pallet_size = state.pallet.size
        T_pb = self.pallet_to_base(pallet_size)
        T_bp = np.linalg.inv(T_pb)
        placed = tuple(state.pallet.boxes)
        obstacles = [(b.box_id, *_aabb(b.pose, b.size)) for b in placed]
        seed = self.pick_configuration(box)

        tried, best = [], None
        clear, hit = self.box_descent_clear(center_top, (dx, dy, dz), placed)
        if not clear:
            details["box_hits"] = hit
            return ValidationResult(False, (RejectCode.ROBOT_COLLISION,), details)
        for yaw in (pose.yaw, pose.yaw + math.pi):
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            clear, hit = self.gripper_descent_clear(center_top, yaw, placed)
            if not clear:
                tried.append((yaw, RejectCode.ROBOT_COLLISION, {"gripper_hits": hit}))
                continue
            for qs, code in self._paths(center_top, yaw, T_pb):
                if code is not None:
                    tried.append((yaw, code, {"reached_samples": len(qs)}))
                    continue
                clearance, who = min((self.arm_clearance(q, T_bp, obstacles, pallet_size) for q in qs),
                                     key=lambda t: t[0])
                if clearance < 0:
                    tried.append((yaw, RejectCode.ROBOT_COLLISION,
                                  {"arm_hits": who, "clearance_m": round(clearance, 4)}))
                    continue
                cycle = self.cycle_time(box, qs[0], seed)
                if best is None or cycle < best[0]:
                    best = (cycle, yaw, qs, clearance, who)
        if best is not None:
            cycle, yaw, qs, clearance, who = best
            details.update({
                "gripper_yaw_rad": round(yaw, 6),
                "q_place": [round(float(v), 5) for v in qs[-1]],
                "q_approach": [round(float(v), 5) for v in qs[0]],
                # the checked vertical line, approach -> place (path_step_m apart): execute
                # through these, a two-point joint move does not stay on the vertical
                "q_path": [[round(float(v), 5) for v in q] for q in qs],
                "arm_clearance_m": round(clearance, 4), "closest_obstacle": who,
                "cycle_time_s": round(cycle, 3),
                "retreat": "same vertical path as the approach",
            })
            return ValidationResult(True, (), details)
        details["attempts"] = [{"gripper_yaw_rad": round(y, 6), "code": c.value, **extra} for y, c, extra in tried]
        codes = tuple(dict.fromkeys(c for _, c, _ in tried))
        return ValidationResult(False, codes, details)

    def first_executable(self, box, ranked, state, robot_state=None):
        """First candidate of ``ranked`` the robot can execute (flowchart:
        "실패 시 다음 후보"); returns (candidate, verdict, rejected dict)."""
        rejected = {}
        for cand in ranked:
            verdict = self.validate_robot_motion(box, cand, state, robot_state)
            if verdict.success:
                return cand, verdict, rejected
            rejected[cand.candidate_id] = verdict
        return None, None, rejected

    def robot_time_sec_by_candidate(self, box, candidates, state):
        """Cycle-time estimates for ``PlanningContext.robot_time_sec_by_candidate``
        (only for candidates the robot can reach)."""
        out = {}
        for cand in candidates:
            verdict = self.validate_robot_motion(box, cand, state)
            if verdict.success:
                out[cand.candidate_id] = verdict.details["cycle_time_s"]
        return out


def _j1_seed(target):
    p = target[:3, 3]
    q = HOME.copy()
    q[0] = math.atan2(p[1], p[0])
    return q


__all__ = ["RobotFeasibility", "HOME", "HDR50_22"]
