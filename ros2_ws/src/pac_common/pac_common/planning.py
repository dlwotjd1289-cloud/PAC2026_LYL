"""Additive typed context for gaps in v0.2; see docs/integration.md."""

from dataclasses import dataclass, field
from enum import Enum
from .models import (
    BoxState,
    FrozenDict,
    PlacementCandidate,
    Pose3D,
    Size3D,
    SystemState,
    ValidationResult,
    count,
    finite,
    identifier,
)


class StateMode(str, Enum):
    ACTUAL = "ACTUAL"
    PLANNED = "PLANNED"
    SIMULATED = "SIMULATED"


@dataclass(frozen=True)
class SkuSpec:
    sku_id: str
    size: Size3D
    weight_kg: float
    allowed_yaws_rad: tuple[float, ...]
    top_load_capacity_n: float

    def __post_init__(self):
        identifier(self.sku_id, "sku_id")
        if not isinstance(self.size, Size3D):
            raise ValueError("Invalid SKU size")
        finite(self.weight_kg, "SKU weight", 0)
        finite(self.top_load_capacity_n, "top_load_capacity_n", 0)
        if not self.allowed_yaws_rad:
            raise ValueError("No SKU orientations")
        for yaw in self.allowed_yaws_rad:
            finite(yaw, "SKU yaw")
        object.__setattr__(
            self, "allowed_yaws_rad", tuple(self.allowed_yaws_rad)
        )


@dataclass(frozen=True)
class ConstraintEvidence:
    """Normalized safety margins emitted by the authoritative hard validator."""

    support_ratio: float
    cog_margin_ratio: float
    load_margin_ratio: float
    pallet_load_margin_ratio: float
    max_load_ratio: float
    support_centering: float
    dependency_count: int
    source: str

    def __post_init__(self):
        for name in (
            "support_ratio",
            "cog_margin_ratio",
            "load_margin_ratio",
            "pallet_load_margin_ratio",
        ):
            value = getattr(self, name)
            finite(value, name, 0)
            if value > 1:
                raise ValueError(f"{name} must be in [0,1]")
        finite(self.max_load_ratio, "max_load_ratio", 0)
        finite(self.support_centering, "support_centering", 0)
        if self.support_centering > 0.5:
            raise ValueError("support_centering exceeds .5")
        count(self.dependency_count, "dependency_count")
        identifier(self.source, "evidence source")


@dataclass(frozen=True)
class PlanningContext:
    catalog: dict[str, SkuSpec]
    pallet_max_weight_kg: float
    observed_preview: tuple[BoxState, ...] = ()
    capacity_overrides_n: dict[str, float] = field(default_factory=dict)
    ems_upper_by_candidate: dict[str, Pose3D] = field(default_factory=dict)
    robot_time_sec_by_candidate: dict[str, float] = field(default_factory=dict)
    buffer_capacity: int = 2
    uncertain_box_ids: tuple[str, ...] = ()
    distribution_status: str = "IN_DISTRIBUTION"

    def __post_init__(self):
        finite(self.pallet_max_weight_kg, "pallet_max_weight_kg", 0)
        if self.pallet_max_weight_kg <= 0:
            raise ValueError("Pallet load capacity must be positive")
        for key, spec in self.catalog.items():
            if not isinstance(spec, SkuSpec) or spec.sku_id != key:
                raise ValueError("Invalid catalog")
        for name in ("capacity_overrides_n", "robot_time_sec_by_candidate"):
            for key, value in getattr(self, name).items():
                finite(value, name + ":" + key, 0)
        for pose in self.ems_upper_by_candidate.values():
            if not isinstance(pose, Pose3D) or pose.frame_id != "pallet":
                raise ValueError("EMS bounds require pallet frame")
        if len({b.box_id for b in self.observed_preview}) != len(
            self.observed_preview
        ):
            raise ValueError("Duplicate preview ID")
        if any(not isinstance(b, BoxState) for b in self.observed_preview):
            raise ValueError("Preview requires BoxState")
        count(self.buffer_capacity, "buffer_capacity")
        if self.distribution_status not in ("IN_DISTRIBUTION", "OOD"):
            raise ValueError("Invalid distribution_status")
        for name in (
            "catalog",
            "capacity_overrides_n",
            "ems_upper_by_candidate",
            "robot_time_sec_by_candidate",
        ):
            object.__setattr__(self, name, FrozenDict(getattr(self, name)))
        object.__setattr__(
            self, "observed_preview", tuple(self.observed_preview)
        )
        object.__setattr__(
            self, "uncertain_box_ids", tuple(self.uncertain_box_ids)
        )


@dataclass(frozen=True)
class FeatureVector:
    names: tuple[str, ...]
    values: tuple[float, ...]
    metrics: dict[str, float]
    geometry_source: str
    time_source: str

    def __post_init__(self):
        if len(self.names) != len(self.values):
            raise ValueError("Feature size mismatch")
        for value in self.values:
            finite(value, "feature")
        object.__setattr__(self, "metrics", FrozenDict(self.metrics))


@dataclass(frozen=True)
class FutureStats:
    mean: float
    worst: float | None
    cvar: float
    blocking_rate: float
    failure_rate: float
    scenario_values: tuple[float, ...] = ()

    def __post_init__(self):
        for name in ("mean", "cvar", "blocking_rate", "failure_rate"):
            value = getattr(self, name)
            finite(value, name, 0)
            if value > 1 + 1e-8:
                raise ValueError(f"{name} exceeds one")
        if self.cvar > self.mean + 1e-8:
            raise ValueError("Lower-tail mean cannot exceed mean")
        if self.worst is not None:
            finite(self.worst, "worst", 0)
            if self.worst > self.cvar + 1e-8:
                raise ValueError("Worst value cannot exceed lower-tail mean")
        for value in self.scenario_values:
            finite(value, "scenario value", 0)


@dataclass(frozen=True)
class CandidateEvaluation:
    candidate: PlacementCandidate
    features: FeatureVector
    rank_logit: float
    future: FutureStats
    future_source: str


@dataclass(frozen=True)
class PlanningResult:
    base_state_version: int
    ranked: tuple[PlacementCandidate, ...]
    evaluations: tuple[CandidateEvaluation, ...]
    rejected: dict[str, ValidationResult]
    diagnostics: dict[str, object]
    requires_robot_validation: bool = True
    state_mode: StateMode = StateMode.PLANNED

    def __post_init__(self):
        object.__setattr__(self, "rejected", FrozenDict(self.rejected))
        object.__setattr__(self, "diagnostics", FrozenDict(self.diagnostics))


@dataclass(frozen=True)
class SimulationSnapshot:
    state: SystemState
    mode: StateMode = StateMode.SIMULATED
