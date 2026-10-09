"""The 1 -> 8 runtime loop on a virtual cell (one box at a time).

1 perception -> 2 State Validator (NG / inspection branch) -> 3 Supervisor
-> 4 HighLevelDecider -> 5 candidates + hard mask + ranking -> 6 robot
feasibility (inside the placer: only executable candidates reach stage 4)
-> 7 execution + post check (escalation L0-L4) -> 8 State Manager (measured
state) -> next box.

The decisions are made by ``RuntimeCore`` (the same object a real cell or
the ROS 2 node drives); this module only simulates the plant around it
(``FieldBox`` truth, ``PerceptionSim``, ``ExecutorSim``, the clock).
The conveyor camera sees the next ``hl_config.close.visible_boxes`` boxes of
the stream (environment ``conveyor.visible_boxes``); they are handed to the
core as the look-ahead window before every decision. The search time is the
ROS runtime's: ``runtime.planning`` share of the previous command's expected
duration (it is planned during that command), ``wait_budget_s`` otherwise.
"""

from dataclasses import dataclass, field
import random

from pac_common import Pose3D
from pac_candidates.geometry import rotated_dims

from .core import ExecutionReport, RuntimeCore
from .executor import ExecutorSim, TrueBox
from .perception import PerceptionSim


@dataclass
class CellSpec:
    """Everything known before the run (order list, pallet, catalog)."""
    stream: tuple                    # FieldBox in true arrival order (missing boxes absent)
    expected_by_sku: dict            # order list: SKU -> count (includes boxes that never come)
    pallet_size: object
    catalog: dict
    weight_ranges: dict = field(default_factory=dict)
    pallet_max_weight_kg: float = 1000.0
    pallet_prefix: str = "PALLET"


class RuntimeLoop:
    def __init__(self, cell, cand_config, hl_config, rt_config, robot, policy, ranker=None, log_events=True,
                 perception_factory=None, core_class=RuntimeCore):
        self.cell = cell
        self.core_class = core_class  # RuntimeCore subclass (e.g. one that records each decision)
        # perception_factory(stream_name) -> object with observe / base_view
        # (default: PerceptionSim; the pipeline plugs in the depth-camera stage 1)
        self.perception_factory = perception_factory
        self.cand_config = cand_config
        self.hl = hl_config
        self.cfg = rt_config
        self.robot = robot
        self.policy = policy
        self.ranker = ranker
        self.log_events = log_events

    def run(self):
        cell, cfg = self.cell, self.cfg
        # separate streams: every variant sees the same observations of the
        # same boxes, whatever the decisions and executions in between
        make = self.perception_factory or (
            lambda stream: PerceptionSim(cfg.perception, random.Random(f"{cfg.seed}:{stream}")))
        perception = make("perception")
        preview_cam = make("preview")
        visible_n = self.hl.close.visible_boxes
        previews = {}  # box id -> conveyor-camera observation (taken once per box)
        executor = ExecutorSim(cfg.execution, cfg.verify, random.Random(f"{cfg.seed}:execution"))
        core = self.core_class(cell, self.cand_config, self.hl, cfg, self.robot, self.policy, self.ranker)
        sm, supervisor = core.sm, core.supervisor
        travel = self.hl.buffer.travel_times()
        truth, true_stack, pallets, events = {}, [], [], []
        idx, missing_done = 0, False
        prev = None  # command executed last: the next one is planned during it

        def log(kind, **kw):
            if self.log_events:
                events.append({"t": round(sm.t, 2), "v": sm.version, "event": kind, **kw})

        def finish_pallet():
            if true_stack:
                vol = sum(tb.size.x * tb.size.y * tb.size.z for tb in true_stack)
                pallets.append({"pallet_id": sm.pallet_id, "boxes": len(true_stack),
                                "fill_true": vol / (cell.pallet_size.x * cell.pallet_size.y * cell.pallet_size.z),
                                "pallet_size": [cell.pallet_size.x, cell.pallet_size.y, cell.pallet_size.z],
                                "layout": [{"box_id": tb.box_id, "size": [tb.size.x, tb.size.y, tb.size.z],
                                            "mass_kg": truth[tb.box_id].truth.weight_kg if tb.box_id in truth else None,
                                            "pose": [tb.pose.x, tb.pose.y, tb.pose.z, tb.pose.yaw]}
                                           for tb in true_stack]})

        while True:
            # 1 + 2: next box from the conveyor
            if sm.current_id() is None and idx < len(cell.stream) and supervisor.can_pick():
                fb = cell.stream[idx]
                idx += 1
                obs = perception.observe(fb, sm.t)
                verdict = core.on_observation(obs, base_view=lambda prev, fb=fb: perception.base_view(fb, prev))
                if verdict is None:
                    continue
                if verdict.route == "INSPECTION":
                    log("INSPECTION", box=fb.truth.box_id, reason=verdict.kind.value)
                    continue
                truth[verdict.box.box_id] = fb
                log("ARRIVE", box=verdict.box.box_id, sku=verdict.box.sku_id, anomaly=verdict.kind.value)
            if not core.has_work():
                if idx >= len(cell.stream):
                    break
                continue
            # 3: stream over -> confirm MISSING once (order list known)
            if idx >= len(cell.stream) and not missing_done:
                missing_done = True
                sm.t += cfg.supervisor.missing_timeout_s
                out = core.on_conveyor_idle(cfg.supervisor.missing_timeout_s)
                if out:
                    log("MISSING", by_sku=dict(out))
            # 4 + 5 + 6
            ahead = cell.stream[idx: idx + visible_n]
            core.on_preview([previews.setdefault(fb.truth.box_id, preview_cam.observe(fb, sm.t))
                             for fb in ahead])
            budget = core.ahead_budget(prev) if prev is not None else cfg.planning.wait_budget_s
            cmd = core.next_command(budget_s=budget)
            prev = cmd if cmd.action != "WAIT" else None
            a = cmd.action
            if a == "WAIT":
                if cmd.reason == "REPACK_NOT_EXECUTABLE":
                    log("REPACK_ABORT")
                    continue
                break
            # 7 (simulated plant) -> core.on_result (7 check + 8 update)
            if a == "BUFFER_CURRENT":
                sm.t += travel[cmd.slot]
                core.on_result(cmd)
                log("BUFFER", box=cmd.box_id, slot=cmd.slot)
            elif a == "PALLET_CLOSE":
                finish_pallet()
                log("CLOSE", pallet=sm.pallet_id, reason=cmd.reason, boxes=len(true_stack))
                core.on_result(cmd)
                true_stack = []
                sm.t += supervisor.pallet_change(sm.t)
            elif a == "REJECT_NG":
                core.on_result(cmd)
                log("NG", box=cmd.box_id, reason=cmd.reason)
            elif a == "PARTIAL_REPACK":
                started, poses = sm.t, {}
                for box_id, cand, details in cmd.repack:
                    others = [tb for tb in true_stack if tb.box_id != box_id]
                    old = next(tb for tb in true_stack if tb.box_id == box_id)
                    placed = next(p for p in sm.placed if p.box_id == box_id)
                    ex = executor.place(placed, old.size, cand, others, sm.pallet_size)
                    true_stack[:] = others + [TrueBox(box_id, old.size, ex.true_pose)]
                    poses[box_id] = ex.measured_pose
                    sm.t += details["cycle_time_s"]
                    log("REPACK_MOVE", box=box_id, level=ex.level)
                core.on_result(cmd, ExecutionReport(repack_poses=poses))
                supervisor.repacking(started, sm.t - started)
            else:  # PLACE_CURRENT / RETRIEVE_BUFFER
                ok, attempts, other = executor.grip()
                sm.t += cfg.execution.retry_time_s * (attempts - 1)
                if not ok:
                    core.on_result(cmd, ExecutionReport(ok=False, attempts=attempts, other_grasp=True))
                    log("L3_GRIP_FAIL", box=cmd.box_id)
                    continue
                box = sm.tracked[cmd.box_id]
                fb = truth[cmd.box_id]
                ex = executor.place(box, fb.truth.size, cmd.candidate, true_stack, sm.pallet_size)
                sm.t += cmd.robot["cycle_time_s"] + (travel[cmd.slot] if a == "RETRIEVE_BUFFER" else 0.0)
                true_pose = ex.true_pose
                if ex.issues:  # operator puts the box where it was planned (true size)
                    p = cmd.candidate.target_pose
                    mdx, mdy, _ = rotated_dims(box.size, p.yaw)
                    tdx, tdy, _ = rotated_dims(fb.truth.size, p.yaw)
                    true_pose = Pose3D("pallet", p.x + (mdx - tdx) / 2, p.y + (mdy - tdy) / 2,
                                       executor.settle_z((p.x, p.y), (p.x + tdx, p.y + tdy), true_stack),
                                       yaw=p.yaw)
                level = core.on_result(cmd, ExecutionReport(True, attempts, other, ex.measured_pose,
                                                            issues=ex.issues))
                true_stack.append(TrueBox(cmd.box_id, fb.truth.size, true_pose))
                log("PLACE", box=cmd.box_id, action=a, level=level, dxy_mm=round(ex.dxy_m * 1000, 1),
                    dz_mm=round(ex.dz_m * 1000, 1), issues=list(ex.issues),
                    gripper_yaw=cmd.robot.get("gripper_yaw_rad"), cycle_s=cmd.robot["cycle_time_s"],
                    pose=[round(v, 4) for v in (ex.measured_pose.x, ex.measured_pose.y, ex.measured_pose.z,
                                                ex.measured_pose.yaw)])
        if not missing_done:  # the stream ended on an inspection box: still confirm MISSING
            sm.t += cfg.supervisor.missing_timeout_s
            out = core.on_conveyor_idle(cfg.supervisor.missing_timeout_s)
            if out:
                log("MISSING", by_sku=dict(out))
        finish_pallet()
        decisions = dict(core.counts)
        return {
            "boxes_in_stream": len(cell.stream),
            "expected": sum(cell.expected_by_sku.values()),
            "placed": sum(p["boxes"] for p in pallets),
            "pallets": len(pallets),
            "fill_true_mean": (sum(p["fill_true"] for p in pallets) / len(pallets)) if pallets else 0.0,
            "time_s": round(sm.t, 1),
            "inspection": core.inspection,
            "missing": dict(supervisor.missing),
            "decisions": decisions,
            "anomalies": dict(core.anomalies),
            "stage6": dict(core.placer.stats),
            "supervisor_time_s": dict(supervisor.time_in),
            "supervisor_log": supervisor.log,
            "pallet_list": pallets,
            "events": events,
        }
