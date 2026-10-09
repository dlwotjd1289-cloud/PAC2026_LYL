from dataclasses import replace
from pac_common.models import BoxState

class GroundTruthObservationGenerator:
    """Initial sim boundary. Replace later with fixed-camera error/noise model."""
    def observe_box(self, ground_truth: BoxState, stamp_sec: float) -> BoxState:
        return replace(ground_truth, stamp_sec=stamp_sec, confidence=1.0,
                       source='sim_ground_truth_top_view')
