"""Explicit input checks kept from the workcell scaffold.

Model construction already validates (models.__post_init__); these helpers
are for callers that receive objects from elsewhere and want a named check.
"""

from .models import PlacementCandidate, SystemState, validate_box  # noqa: F401


def validate_candidate(candidate: PlacementCandidate, state: SystemState):
    if candidate.target_pose.frame_id != "pallet":
        raise ValueError("target_pose.frame_id must be 'pallet'")
    if candidate.base_state_version != state.state_version:
        raise ValueError("STALE_PLAN: stale candidate")
