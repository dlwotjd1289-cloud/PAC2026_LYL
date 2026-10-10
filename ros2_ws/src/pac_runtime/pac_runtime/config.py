"""Settings of the runtime stages 1-3 and 7-8 (``config/taehyeon/runtime.yaml``).

Probabilities and tolerances are ASSUMPTIONS for the virtual cell (no sensor
or robot data yet); they are grouped by flowchart stage so a real sensor /
robot can replace each block without touching the others.
"""

from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
import math

import yaml


@dataclass(frozen=True)
class PerceptionConfig:          # stage 1
    weight_noise_std_ratio: float = 0.01      # inline scale
    size_noise_std_m: float = 0.001           # top-view RGB-D
    uncertain_probability: float = 0.05       # occlusion / glare -> low confidence
    uncertain_size_noise_std_m: float = 0.004
    uncertain_confidence: float = 0.6
    label_fail_probability: float = 0.02      # ID / label not read on the top view
    damage_detect_probability: float = 0.8    # visible dents (same as virtual_data)
    base_view_recover_probability: float = 0.7  # base view reads a missed label
    base_view_size_noise_std_m: float = 0.001


@dataclass(frozen=True)
class ValidatorConfig:           # stage 2
    min_label_confidence: float = 0.5
    size_tolerance_m: float = 0.012           # measured vs SKU nominal, per axis
    size_tolerance_ratio: float = 0.04
    weight_tolerance_ratio: float = 0.08      # outside the SKU weight range by more -> mismatch
    damage_policy: str = "reject"             # reject (flowchart) | place_no_load (0 N on top)


@dataclass(frozen=True)
class SupervisorConfig:          # stage 3
    pallet_change_time_s: float = 60.0
    pallet_change_manual: bool = False        # a person swaps the pallet: robot stopped, HOLD (flowchart)
    missing_timeout_s: float = 30.0           # conveyor idle this long -> remaining boxes MISSING
    operator_time_s: float = 120.0            # HOLD for a manual correction (L4)


@dataclass(frozen=True)
class ExecutionConfig:           # stage 7 (virtual robot)
    # within the 5-2 budget (4 mm gap between boxes, 2 mm edge tolerance)
    place_xy_noise_std_m: float = 0.001
    grip_fail_probability: float = 0.01
    max_grip_retries: int = 1                 # then the other grasp (gripper yaw + pi), then inspection
    retry_time_s: float = 4.0


@dataclass(frozen=True)
class VerifyConfig:              # stage 7 post check (top view heightmap)
    l0_xy_m: float = 0.005
    l0_z_m: float = 0.005
    l1_xy_m: float = 0.02
    l1_z_m: float = 0.01
    max_overlap_m: float = 0.002              # true penetration allowed
    max_protrusion_m: float = 0.002           # beyond the pallet edge
    min_support_ratio: float = 0.6            # true support under the placed box
    heightmap_cell_m: float = 0.01


@dataclass(frozen=True)
class PlanningConfig:            # stage 4/5 search time per decision (look-ahead)
    # Planned during the robot motion: budget = ahead_ratio x expected duration of
    # the action being executed (stage-6 cycle time, buffer travel, pallet change),
    # scaled by the measured / expected duration of that action type so far.
    ahead_ratio: float = 0.8
    max_budget_s: float = 10.0
    min_budget_s: float = 0.3
    # Planned while the robot waits (first box, forecast did not match the result).
    wait_budget_s: float = 1.5
    # Weight of the newest measured / expected duration in the running correction.
    duration_smoothing: float = 0.3
    # Whole-decision monitoring target. Zero preserves the legacy profile.
    decision_budget_s: float = 0.0
    # Over decision_budget_s: True = still execute the command computed so far
    # (the search is anytime: it already holds the best option found in time);
    # False = discard it and WAIT (legacy, stops the order).
    use_late_command: bool = False


@dataclass(frozen=True)
class CapacityConfig:
    model: str = "team_mckee"
    site: str = "general"


@dataclass(frozen=True)
class RuntimeConfig:
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    validator: ValidatorConfig = field(default_factory=ValidatorConfig)
    supervisor: SupervisorConfig = field(default_factory=SupervisorConfig)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    verify: VerifyConfig = field(default_factory=VerifyConfig)
    planning: PlanningConfig = field(default_factory=PlanningConfig)
    capacity: CapacityConfig = field(default_factory=CapacityConfig)
    robot_checks_per_option: int = 12         # stage 6 tries the first N ranked candidates
    seed: int = 0


def _build(cls, data):
    names = {f.name for f in fields(cls)}
    unknown = set(data) - names
    if unknown:
        raise ValueError(f"unknown {cls.__name__} keys: {sorted(unknown)}")
    kwargs = {}
    for f in fields(cls):
        if f.name in data:
            default = f.default_factory() if callable(f.default_factory) else f.default
            value = data[f.name]
            kwargs[f.name] = _build(type(default), value or {}) if is_dataclass(default) else value
    return cls(**kwargs)


def config_from_dict(data):
    cfg = _build(RuntimeConfig, (data or {}).get("runtime", data or {}))
    if cfg.validator.damage_policy not in ("reject", "place_no_load"):
        raise ValueError("validator.damage_policy must be reject or place_no_load")
    for name in ("label_fail_probability", "damage_detect_probability", "base_view_recover_probability",
                 "uncertain_probability"):
        if not 0.0 <= getattr(cfg.perception, name) <= 1.0:
            raise ValueError(f"perception.{name} must be a probability")
    if not 0.0 <= cfg.execution.grip_fail_probability <= 1.0:
        raise ValueError("execution.grip_fail_probability must be a probability")
    if cfg.robot_checks_per_option < 1:
        raise ValueError("robot_checks_per_option must be >= 1")
    pl = cfg.planning
    if any(not math.isfinite(getattr(pl, f.name)) for f in fields(pl)):
        raise ValueError("planning parameters must be finite")
    if not 0.0 < pl.ahead_ratio <= 1.0 or not 0.0 < pl.duration_smoothing <= 1.0:
        raise ValueError("planning.ahead_ratio and planning.duration_smoothing must be in (0, 1]")
    if not 0.0 < pl.min_budget_s <= pl.max_budget_s or pl.wait_budget_s <= 0.0:
        raise ValueError("planning budgets must be positive and min_budget_s <= max_budget_s")
    if pl.decision_budget_s < 0:
        raise ValueError("planning.decision_budget_s must be nonnegative")
    if cfg.capacity.model not in ("team_mckee", "jaesung_site"):
        raise ValueError("capacity.model must be team_mckee or jaesung_site")
    from .capacity import SITES
    if cfg.capacity.site not in SITES:
        raise ValueError("unknown capacity.site")
    return cfg


def load_runtime_config(path):
    return config_from_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))
