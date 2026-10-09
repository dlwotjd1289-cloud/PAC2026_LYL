"""The single source of runtime dataclasses for all team modules."""

from .models import (
    BoxState,
    BoxStatus,
    ExecutionResult,
    FrozenDict,
    InventoryState,
    PalletState,
    PlacedBox,
    PlacementCandidate,
    Pose3D,
    RejectCode,
    Size3D,
    SystemState,
    ValidationResult,
    plain,
)
from .planning import (
    CandidateEvaluation,
    ConstraintEvidence,
    FeatureVector,
    FutureStats,
    PlanningContext,
    PlanningResult,
    SimulationSnapshot,
    SkuSpec,
    StateMode,
)
from .state_manager import CommitOutcome, CommitTolerance, StateManager
