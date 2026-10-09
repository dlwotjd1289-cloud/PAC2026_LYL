"""End-to-end pipeline: synthetic depth camera + Donghan measurement, and one tiny order."""
from dataclasses import replace
import json

import numpy as np
import pytest

from pac_common import BoxState, BoxStatus, Pose3D, Size3D
from pac_perception.perception_config import load_perception_config
from pac_runtime import FieldBox
from pac_runtime.config import PerceptionConfig, ValidatorConfig
from pac_runtime.state_validator import Anomaly, StateValidator

from pipeline import REPO
from pipeline.depth_camera import CameraSpec, at_stopper, render
from pipeline.depth_perception import DepthPerception
from pipeline.run import load_pipeline_config, main

CFG = load_pipeline_config(REPO / "config/pipeline.yaml")
DEPTH, RESOLVER = load_perception_config(REPO / CFG["perception"]["config"])
DEPTH = replace(DEPTH, roi_xy_m=tuple(CFG["perception"]["roi_xy_m"]))
CATALOG_ORDER = {"K08": ((0.34, 0.25, 0.21), (1.7, 4.8)), "K13": ((0.52, 0.48, 0.40), (13.0, 28.5))}


def cell():
    from pac_common import SkuSpec

    class Cell:
        catalog = {k: SkuSpec(k, Size3D(*s), w[1], (0.0, 1.5707963), 400.0) for k, (s, w) in CATALOG_ORDER.items()}
        weight_ranges = {k: w for k, (_, w) in CATALOG_ORDER.items()}
        stream = ()
    return Cell()


def field(sku, damaged=False, box_id="B1"):
    size, (lo, hi) = CATALOG_ORDER[sku]
    truth = BoxState(box_id, sku, Size3D(*size), (lo + hi) / 2, Pose3D("conveyor", 0, 0, 0), (0.0, 1.5707963),
                     BoxStatus.MEASURED, 1.0, 0.0, "TRUTH")
    return FieldBox(truth, damaged)


@pytest.mark.parametrize("dims", [(0.22, 0.19, 0.09), (0.52, 0.48, 0.40), (0.598, 0.48, 0.40)])
def test_size_within_validator_tolerance(dims):
    from pac_perception.depth_measurement import measure_box

    cam = CameraSpec()
    m = measure_box(render(cam, [at_stopper(*dims, yaw=0.03)], np.random.default_rng(1)), cam.model(), DEPTH)
    assert m.in_full_view and not m.damage_reasons and m.confidence == 1.0
    assert max(abs(a - b) for a, b in zip((m.size.x, m.size.y, m.size.z), dims)) < 0.006


def test_shipped_roi_never_sees_a_box_at_the_stopper_whole():
    """Why config/pipeline.yaml widens the ROI: 0.232 < stopper face 0.235."""
    from pac_perception.depth_measurement import measure_box

    shipped, _ = load_perception_config(REPO / CFG["perception"]["config"])
    cam = CameraSpec()
    m = measure_box(render(cam, [at_stopper(0.34, 0.25, 0.21, dent_m=0.03)], np.random.default_rng(0)),
                    cam.model(), shipped)
    assert not m.in_full_view and not m.damage_reasons


@pytest.mark.parametrize("box_id", ["dent-A", "corner-B", "x1", "x2"])
def test_damaged_box_found_by_depth_and_rejected_by_stage2(box_id):
    perception = DepthPerception(cell(), DEPTH, RESOLVER, PerceptionConfig(label_fail_probability=0.0), 3)
    obs = perception.observe(field("K08", damaged=True, box_id=box_id), 1.0)
    assert obs.visual_damage
    verdict = StateValidator(cell().catalog, ValidatorConfig(), cell().weight_ranges).validate(obs)
    assert (verdict.kind, verdict.route) == (Anomaly.DAMAGED, "INSPECTION")


def test_unread_label_resolved_from_depth_size_and_weight():
    perception = DepthPerception(cell(), DEPTH, RESOLVER, PerceptionConfig(label_fail_probability=1.0), 5)
    obs = perception.observe(field("K13"), 1.0)
    assert obs.label_sku == "K13" and obs.confidence < 1.0
    assert perception.records[-1]["sku_resolved"]


def test_queued_box_upstream_is_split_off():
    c = cell()
    first, second = field("K08", box_id="A"), field("K13", box_id="B")
    c.stream = (first, second)
    perception = DepthPerception(c, DEPTH, RESOLVER, PerceptionConfig(), 0, queue_probability=1.0)
    obs = perception.observe(first, 0.0)
    assert perception.records[-1]["boxes_in_roi"] == 2
    assert abs(obs.size.x - 0.34) < 0.006 and abs(obs.size.y - 0.25) < 0.006


def test_end_to_end_tiny_order(tmp_path):
    main(["--output", str(tmp_path), "--boxes", "8", "--per-family", "1", "--ids", "S0001", "--no-dashboard"])
    summary = json.loads((tmp_path / "summary.json").read_text())
    (line,) = summary["orders"]
    assert line["status"] == "complete" and line["arrived"] == line["placed"] + line["inspection"]
    order = json.loads((tmp_path / "orders/S0001.json").read_text())
    assert order["evaluation"] and not any("error" in e for e in order["evaluation"])
    assert order["result"]["pallet_list"][0]["layout"]
