"""Contract test: our stage 1 output goes through the integrated stage 2 validator unchanged."""
from pac_common import BoxStatus
from pac_runtime.config import ValidatorConfig
from pac_runtime.state_validator import Anomaly, StateValidator

from pac_perception.depth_measurement import DepthConfig, measure_box
from pac_perception.raw_observation import build_raw_observation

from perception_testkit import CATALOG, WEIGHT_RANGES, camera, render

DEPTH = DepthConfig(roi_xy_m=(-0.6, 0.6, -0.45, 0.45))
VALIDATOR = StateValidator(CATALOG, ValidatorConfig(), WEIGHT_RANGES)


def observe(depth, weight_kg, label_sku=None):
    return build_raw_observation("B001", weight_kg, measure_box(depth, camera(), DEPTH), 12.5,
                                 label_sku=label_sku, catalog=CATALOG, weight_ranges=WEIGHT_RANGES)


def test_unlabelled_box_is_resolved_and_planned():
    obs = observe(render(0.34, 0.25, 0.21), 3.1)
    assert obs.label_sku == "K08"
    verdict = VALIDATOR.validate(obs)
    assert (verdict.kind, verdict.route) == (Anomaly.OK, "PLAN")
    assert verdict.box.status == BoxStatus.MEASURED and verdict.box.weight_kg == 3.1
    assert verdict.uncertain  # inferred label -> confidence < 1 -> delta


def test_read_label_keeps_full_confidence():
    verdict = VALIDATOR.validate(observe(render(0.34, 0.25, 0.21), 3.1, label_sku="K08"))
    assert (verdict.kind, verdict.uncertain) == (Anomaly.OK, False)


def test_dented_box_is_rejected_by_stage_2():
    verdict = VALIDATOR.validate(observe(render(0.34, 0.25, 0.21, dent_m=0.03), 3.1, label_sku="K08"))
    assert (verdict.kind, verdict.route) == (Anomaly.DAMAGED, "INSPECTION")


def test_unmatched_box_goes_to_inspection():
    obs = observe(render(0.60, 0.40, 0.30), 9.0)
    assert obs.label_sku is None
    verdict = VALIDATOR.validate(obs)
    assert (verdict.kind, verdict.route) == (Anomaly.RECOGNITION_FAIL, "INSPECTION")
