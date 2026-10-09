"""Canonical v0.2 runtime types. SI units; poses must name their frame."""

from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum
import math
import re


class FrozenDict(dict):
    """Read-compatible dict with defensive, recursively frozen values."""

    def __init__(self, value=(), **kwargs):
        dict.__init__(
            self, ((k, freeze(v)) for k, v in dict(value, **kwargs).items())
        )

    def _deny(self, *args, **kwargs):
        raise TypeError("Snapshot mappings are immutable")

    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = (
        _deny
    )
    __ior__ = _deny

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        return self


def freeze(value):
    if isinstance(value, dict):
        return FrozenDict(value)
    if isinstance(value, (list, tuple)):
        return tuple(freeze(v) for v in value)
    return value


def finite(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f"Invalid {name}: {value}")


def count(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")


def identifier(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be nonempty text")


def plain(value):
    """JSON boundary only; algorithms exchange dataclasses."""
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {f.name: plain(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


class BoxStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    DETECTED = "DETECTED"
    MEASURED = "MEASURED"
    ON_CONVEYOR = "ON_CONVEYOR"
    READY_FOR_PICK = "READY_FOR_PICK"
    PICKING = "PICKING"
    IN_TRANSIT = "IN_TRANSIT"
    PLACED = "PLACED"
    BUFFERED = "BUFFERED"
    REJECTED = "REJECTED"
    FAILED = "FAILED"


class RejectCode(str, Enum):
    OUT_OF_BOUND = "OUT_OF_BOUND"
    HEIGHT_LIMIT = "HEIGHT_LIMIT"
    BOX_COLLISION = "BOX_COLLISION"
    LOW_SUPPORT = "LOW_SUPPORT"
    LOAD_VIOLATION = "LOAD_VIOLATION"
    COG_VIOLATION = "COG_VIOLATION"
    PAYLOAD_EXCEEDED = "PAYLOAD_EXCEEDED"
    IK_FAIL = "IK_FAIL"
    ROBOT_COLLISION = "ROBOT_COLLISION"
    APPROACH_FAIL = "APPROACH_FAIL"
    RETREAT_FAIL = "RETREAT_FAIL"
    TIMEOUT = "TIMEOUT"
    INVALID_STATE = "INVALID_STATE"
    STALE_PLAN = "STALE_PLAN"
    SENSOR_UNCERTAIN = "SENSOR_UNCERTAIN"
    TRACKING_LOST = "TRACKING_LOST"
    SIZE_MISMATCH = "SIZE_MISMATCH"
    WEIGHT_MISMATCH = "WEIGHT_MISMATCH"
    EXECUTION_FAIL = "EXECUTION_FAIL"
    UNKNOWN_ERROR = "UNKNOWN_ERROR"


@dataclass(frozen=True)
class Size3D:
    x: float
    y: float
    z: float

    def __post_init__(self):
        for name in ("x", "y", "z"):
            finite(getattr(self, name), name)
            if getattr(self, name) <= 0:
                raise ValueError("All dimensions must be positive")


@dataclass(frozen=True)
class Pose3D:
    frame_id: str
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not isinstance(self.frame_id, str) or not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_/]*", self.frame_id
        ):
            raise ValueError("Invalid frame_id")
        for name in ("x", "y", "z", "roll", "pitch", "yaw"):
            finite(getattr(self, name), name)


def validate_box(obj):
    identifier(obj.box_id, "box_id")
    identifier(obj.sku_id, "sku_id")
    if not isinstance(obj.size, Size3D) or not isinstance(obj.pose, Pose3D):
        raise ValueError("Use canonical Size3D and Pose3D")
    finite(obj.weight_kg, "weight_kg", 0)


@dataclass(frozen=True)
class BoxState:
    box_id: str
    sku_id: str
    size: Size3D
    weight_kg: float
    pose: Pose3D
    allowed_yaws_rad: tuple[float, ...]
    status: BoxStatus
    confidence: float
    stamp_sec: float
    source: str

    def __post_init__(self):
        validate_box(self)
        if not isinstance(self.status, BoxStatus):
            raise ValueError("status must be BoxStatus")
        if not self.allowed_yaws_rad:
            raise ValueError("No allowed orientation")
        for yaw in self.allowed_yaws_rad:
            finite(yaw, "allowed_yaw")
        object.__setattr__(
            self, "allowed_yaws_rad", tuple(self.allowed_yaws_rad)
        )
        finite(self.confidence, "confidence", 0)
        if self.confidence > 1:
            raise ValueError("confidence exceeds one")
        finite(self.stamp_sec, "stamp_sec", 0)
        identifier(self.source, "source")


@dataclass(frozen=True)
class PlacedBox:
    box_id: str
    sku_id: str
    size: Size3D
    weight_kg: float
    pose: Pose3D

    def __post_init__(self):
        validate_box(self)
        if self.pose.frame_id != "pallet":
            raise ValueError("PlacedBox must be in pallet frame")


@dataclass(frozen=True)
class PalletState:
    pallet_id: str
    size: Size3D
    boxes: tuple[PlacedBox, ...]

    def __post_init__(self):
        identifier(self.pallet_id, "pallet_id")
        if not isinstance(self.size, Size3D):
            raise ValueError("Invalid pallet size")
        if any(not isinstance(b, PlacedBox) for b in self.boxes):
            raise ValueError("Expected PlacedBox")
        if len({b.box_id for b in self.boxes}) != len(self.boxes):
            raise ValueError("Duplicate placed box_id")
        object.__setattr__(self, "boxes", tuple(self.boxes))


@dataclass(frozen=True)
class InventoryState:
    tracked_boxes: dict[str, BoxState]
    remaining_by_sku: dict[str, int]

    def __post_init__(self):
        for key, box in self.tracked_boxes.items():
            if not isinstance(box, BoxState) or key != box.box_id:
                raise ValueError("Invalid tracked box")
        for key, number in self.remaining_by_sku.items():
            identifier(key, "sku_id")
            count(number, "remaining_count")
        object.__setattr__(
            self, "tracked_boxes", FrozenDict(self.tracked_boxes)
        )
        object.__setattr__(
            self, "remaining_by_sku", FrozenDict(self.remaining_by_sku)
        )


@dataclass(frozen=True)
class SystemState:
    state_version: int
    stamp_sec: float
    pallet: PalletState
    inventory: InventoryState

    def __post_init__(self):
        count(self.state_version, "state_version")
        finite(self.stamp_sec, "stamp_sec", 0)
        if not isinstance(self.pallet, PalletState) or not isinstance(
            self.inventory, InventoryState
        ):
            raise ValueError("Invalid state model")
        placed = {b.box_id: b for b in self.pallet.boxes}
        for key, box in self.inventory.tracked_boxes.items():
            if key in placed:
                p = placed[key]
                if box.status != BoxStatus.PLACED or any(
                    getattr(box, n) != getattr(p, n)
                    for n in ("sku_id", "size", "weight_kg", "pose")
                ):
                    raise ValueError("Placed and tracked box disagree")
            elif box.status == BoxStatus.PLACED:
                raise ValueError("PLACED box is absent from pallet")


@dataclass(frozen=True)
class PlacementCandidate:
    candidate_id: str
    box_id: str
    target_pose: Pose3D
    base_state_version: int
    score: float | None = None
    score_detail: dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        identifier(self.candidate_id, "candidate_id")
        identifier(self.box_id, "box_id")
        if (
            not isinstance(self.target_pose, Pose3D)
            or self.target_pose.frame_id != "pallet"
        ):
            raise ValueError("target_pose.frame_id must be pallet")
        count(self.base_state_version, "base_state_version")
        if self.score is not None:
            finite(self.score, "score")
        for key, value in self.score_detail.items():
            finite(value, key)
        object.__setattr__(self, "score_detail", FrozenDict(self.score_detail))


@dataclass(frozen=True)
class ValidationResult:
    success: bool
    codes: tuple[RejectCode, ...] = ()
    details: dict[str, object] = field(default_factory=dict)

    def __post_init__(self):
        if type(self.success) is not bool or any(
            not isinstance(c, RejectCode) for c in self.codes
        ):
            raise ValueError("Invalid ValidationResult")
        if self.success and self.codes:
            raise ValueError(
                "Successful validation cannot contain reject codes"
            )
        if not self.success and not self.codes:
            raise ValueError("Rejected validation needs a reason")
        object.__setattr__(self, "codes", tuple(self.codes))
        object.__setattr__(self, "details", FrozenDict(self.details))


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    box_id: str
    candidate_id: str
    actual_pose: Pose3D | None
    codes: tuple[RejectCode, ...]
    stamp_sec: float
