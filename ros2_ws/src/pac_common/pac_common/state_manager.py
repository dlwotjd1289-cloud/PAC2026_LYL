"""Single writer of ACTUAL SystemState (common standard section 12).

Other modules hand in observations, plans and execution results; only this
class builds the next immutable snapshot and increments state_version.

Commit rule for executed placements (team decision 2026-10-08):
the measured pose is compared with the planned target (both lower-corner
poses in frame "pallet"). Within ``CommitTolerance`` the PLANNED pose is
committed, so neighbouring boxes stay exactly non-overlapping for the
planner (contact solvers and sensors leave ~0.01 mm penetrations that the
planner's 1e-8 m geometry checks reject). Outside the tolerance nothing is
placed: the box becomes FAILED and must be re-perceived before re-planning.
"""

from dataclasses import dataclass, replace
import math

from .models import (
    BoxState,
    BoxStatus,
    ExecutionResult,
    PlacedBox,
    PlacementCandidate,
    Pose3D,
    RejectCode,
    SystemState,
    validate_box,
)


@dataclass(frozen=True)
class CommitTolerance:
    xy_m: float = 0.005
    z_m: float = 0.003
    yaw_rad: float = math.radians(1.0)


@dataclass(frozen=True)
class CommitOutcome:
    state: SystemState
    placed: bool
    codes: tuple[RejectCode, ...] = ()
    reason: str = ""


def _yaw_error(a, b):
    """Box footprints are symmetric under 180 deg."""
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def pose_error(measured: Pose3D, planned: Pose3D):
    """(xy, |z|, yaw) error between two pallet-frame corner poses."""
    if measured.frame_id != planned.frame_id:
        raise ValueError("Poses must share frame_id")
    return (
        math.hypot(measured.x - planned.x, measured.y - planned.y),
        abs(measured.z - planned.z),
        _yaw_error(measured.yaw, planned.yaw),
    )


class StateManager:
    """Single writer for ACTUAL SystemState."""

    def __init__(self, initial_state: SystemState,
                 tolerance: CommitTolerance = CommitTolerance()):
        self._state = initial_state
        self._tolerance = tolerance
        self._pending: dict[str, tuple[PlacementCandidate, tuple]] = {}

    @property
    def snapshot(self) -> SystemState:
        return self._state

    def _bump(self, stamp_sec, **changes) -> SystemState:
        self._state = replace(
            self._state,
            state_version=self._state.state_version + 1,
            stamp_sec=stamp_sec,
            **changes,
        )
        return self._state

    def commit_observation(self, observed_box: BoxState, stamp_sec: float,
                           from_stock: bool = True) -> SystemState:
        """Track a newly observed box (or update a tracked one).

        A box that first gets an ID leaves anonymous stock: remaining_by_sku
        is decremented here, never again by planners (from_stock=False for
        boxes that were not counted in remaining_by_sku).
        """
        validate_box(observed_box)
        tracked = dict(self._state.inventory.tracked_boxes)
        old = tracked.get(observed_box.box_id)
        if old is not None and old.status == BoxStatus.PLACED:
            raise ValueError("Cannot re-observe a PLACED box")
        remaining = dict(self._state.inventory.remaining_by_sku)
        if old is None and from_stock:
            if remaining.get(observed_box.sku_id, 0) <= 0:
                raise ValueError(
                    f"No remaining stock for {observed_box.sku_id}"
                )
            remaining[observed_box.sku_id] -= 1
        tracked[observed_box.box_id] = observed_box
        inv = replace(self._state.inventory, tracked_boxes=tracked,
                      remaining_by_sku=remaining)
        return self._bump(stamp_sec, inventory=inv)

    def register_plan(self, candidate: PlacementCandidate) -> None:
        """Remember the candidate sent to the executor (one per box)."""
        if candidate.base_state_version != self._state.state_version:
            raise ValueError("STALE_PLAN: candidate built on another version")
        box = self._state.inventory.tracked_boxes.get(candidate.box_id)
        if box is None or box.status == BoxStatus.PLACED:
            raise ValueError("Box is not tracked or already placed")
        if candidate.box_id in self._pending:
            raise ValueError("Box already has a pending execution")
        self._pending[candidate.box_id] = (candidate, self._state.pallet.boxes)

    def commit_execution(self, result: ExecutionResult) -> CommitOutcome:
        pending = self._pending.pop(result.box_id, None)
        if pending is None:
            raise ValueError("No pending plan for this box (duplicate result?)")
        candidate, pallet_at_plan = pending
        if candidate.candidate_id != result.candidate_id:
            raise ValueError("Execution result is for another candidate")
        tracked = dict(self._state.inventory.tracked_boxes)
        box = tracked[result.box_id]

        def fail(codes, reason):
            tracked[result.box_id] = replace(
                box, status=BoxStatus.FAILED, stamp_sec=result.stamp_sec
            )
            state = self._bump(
                result.stamp_sec,
                inventory=replace(self._state.inventory, tracked_boxes=tracked),
            )
            return CommitOutcome(state, False, tuple(codes), reason)

        if not result.success or result.actual_pose is None:
            return fail(result.codes or (RejectCode.EXECUTION_FAIL,),
                        "execution reported failure")
        if self._state.pallet.boxes != pallet_at_plan:
            return fail((RejectCode.STALE_PLAN,),
                        "pallet changed between plan and execution")
        xy, z, yaw = pose_error(result.actual_pose, candidate.target_pose)
        tol = self._tolerance
        if xy > tol.xy_m or z > tol.z_m or yaw > tol.yaw_rad:
            return fail(
                (RejectCode.SENSOR_UNCERTAIN,),
                f"measured pose differs from plan (xy {xy*1000:.1f} mm, "
                f"z {z*1000:.1f} mm, yaw {math.degrees(yaw):.2f} deg); "
                "re-perceive before planning",
            )
        pose = candidate.target_pose
        placed = PlacedBox(box.box_id, box.sku_id, box.size, box.weight_kg, pose)
        tracked[result.box_id] = replace(
            box, pose=pose, status=BoxStatus.PLACED, stamp_sec=result.stamp_sec
        )
        state = self._bump(
            result.stamp_sec,
            pallet=replace(self._state.pallet,
                           boxes=self._state.pallet.boxes + (placed,)),
            inventory=replace(self._state.inventory, tracked_boxes=tracked),
        )
        return CommitOutcome(state, True)
