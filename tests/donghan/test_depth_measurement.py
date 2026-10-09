import math
from pathlib import Path

import pytest

from pac_perception.depth_measurement import DepthConfig, measure_box
from pac_perception.perception_config import load_perception_config

from perception_testkit import camera, render, render_scene

CONFIG = DepthConfig(roi_xy_m=(-0.6, 0.6, -0.45, 0.45))


def test_nominal_box_size_pose_and_no_damage():
    m = measure_box(render(0.34, 0.25, 0.21, x=0.05, y=-0.03), camera(), CONFIG)
    assert m.size.x == pytest.approx(0.34, abs=0.006)
    assert m.size.y == pytest.approx(0.25, abs=0.006)
    assert m.size.z == pytest.approx(0.21, abs=0.003)
    assert (m.pose.frame_id, m.pose.x, m.pose.y) == ("conveyor", pytest.approx(0.05, abs=0.005),
                                                     pytest.approx(-0.03, abs=0.005))
    assert not m.damaged and m.confidence == 1.0


def test_rotated_box_reports_long_side_yaw():
    m = measure_box(render(0.52, 0.48, 0.40, yaw=math.radians(30)), camera(), CONFIG)
    assert m.size.x == pytest.approx(0.52, abs=0.008)
    assert m.size.y == pytest.approx(0.48, abs=0.008)
    assert math.degrees(m.pose.yaw) == pytest.approx(30, abs=1.5)


def test_dented_top_is_damage():
    m = measure_box(render(0.34, 0.25, 0.21, dent_m=0.03), camera(), CONFIG)
    assert m.damage_reasons == ("top_dent",)
    assert m.size.z == pytest.approx(0.21, abs=0.003)


def test_crushed_corner_is_damage():
    m = measure_box(render(0.34, 0.25, 0.21, crushed_corner_m=0.12), camera(), CONFIG)
    assert "crushed_corner" in m.damage_reasons


def test_crushed_corner_on_a_large_box_is_damage():
    # area ratio stays above 0.92 here; the corner gap catches it
    m = measure_box(render(0.52, 0.48, 0.40, crushed_corner_m=0.10), camera(), CONFIG)
    assert m.rectangularity > CONFIG.min_rectangularity
    assert "crushed_corner" in m.damage_reasons


def test_intact_corners_have_small_gap():
    m = measure_box(render(0.52, 0.48, 0.40, yaw=0.3), camera(), CONFIG)
    assert m.corner_gap_m < CONFIG.max_corner_gap_m


def test_missing_depth_lowers_confidence():
    m = measure_box(render(0.34, 0.25, 0.21, hole_ratio=0.4), camera(), CONFIG)
    assert m.confidence == CONFIG.uncertain_confidence
    assert not m.damaged


def test_queued_box_behind_is_separated():
    # box at the stopper (x = +0.15) and the next one waiting 5 cm upstream
    scene = [dict(length=0.27, width=0.18, height=0.15, x=0.15),
             dict(length=0.34, width=0.25, height=0.21, x=0.15 - 0.135 - 0.05 - 0.17)]
    m = measure_box(render_scene(scene), camera(), CONFIG)
    assert m.size.x == pytest.approx(0.27, abs=0.006) and m.size.z == pytest.approx(0.15, abs=0.003)
    assert m.box_count == 2 and m.gap_downstream_m is None
    assert m.gap_upstream_m == pytest.approx(0.05, abs=0.01)

    upstream = measure_box(render_scene(scene), camera(), CONFIG, target_x=-0.2)
    assert upstream.size.x == pytest.approx(0.34, abs=0.006)
    assert upstream.gap_downstream_m == pytest.approx(0.05, abs=0.01)
    with pytest.raises(ValueError, match="target_x"):
        measure_box(render_scene(scene), camera(), CONFIG, target_x=-0.55)


def test_touching_boxes_stay_one_cluster():
    scene = [dict(length=0.27, width=0.18, height=0.15, x=0.135),
             dict(length=0.27, width=0.18, height=0.15, x=-0.135)]
    m = measure_box(render_scene(scene), camera(), CONFIG)
    assert m.box_count == 1 and m.size.x == pytest.approx(0.54, abs=0.008)


def test_box_cut_by_the_roi_or_image_edge_is_not_called_damaged():
    narrow = DepthConfig(roi_xy_m=(-0.6, 0.3, -0.45, 0.45))
    m = measure_box(render(0.52, 0.48, 0.40, x=0.15), camera(), narrow)   # box reaches x 0.41 > ROI 0.3
    assert not m.in_full_view and not m.damaged
    assert m.confidence == CONFIG.uncertain_confidence
    # image half width at the box top is 0.59 m: a box reaching 0.71 m is cut by the image border
    assert not measure_box(render(0.52, 0.48, 0.40, x=0.45), camera(), CONFIG).in_full_view
    assert measure_box(render(0.52, 0.48, 0.40), camera(), CONFIG).in_full_view


def test_empty_pick_zone_raises():
    with pytest.raises(ValueError, match="no box"):
        measure_box(render(0.34, 0.25, 0.21, x=2.0), camera(), CONFIG)


def test_yaml_config_loads():
    path = Path(__file__).resolve().parents[2] / "config" / "donghan" / "perception_depth.yaml"
    depth, resolver = load_perception_config(path)
    assert depth.max_dent_m == 0.012 and resolver.size_tolerance_m == 0.012
