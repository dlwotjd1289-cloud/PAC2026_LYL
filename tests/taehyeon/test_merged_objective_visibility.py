"""Independent objective and information-boundary unit checks; no order runs."""
from collections import Counter
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from pac_candidates import CandidateConfig
from pac_common import PlacedBox, Pose3D, Size3D, SkuSpec
from pac_highlevel import Arrival, HighLevelConfig, PalletizingWorld, from_index, to_index
from pac_highlevel.lookahead import LookaheadConfig, LookaheadPolicy, _child, leaf_score, window_clone
from pac_highlevel.world import BufferEntry
from th_helpers import make_box


def score_fixture():
    root = SimpleNamespace(closed=[SimpleNamespace(fill=.8)], ng=["old"], time_s=10.)
    node = SimpleNamespace(
        closed=[root.closed[0], SimpleNamespace(fill=.4)], ng=["old", "new"], time_s=25.,
        placed=[SimpleNamespace(size=Size3D(.5, .5, .2))],
        heightmap=lambda: np.array([[.2, .4], [.1, .3]]),
        config=SimpleNamespace(features=SimpleNamespace(heightmap_cell_m=.5)),
        pallet_volume=2., pallet_size=Size3D(1., 1., 2.),
        fill=lambda: .3, buffer=[object(), None, object()],
    )
    return node, root


def test_default_legacy_objective_keeps_original_numeric_terms(monkeypatch):
    from pac_highlevel import lookahead
    monkeypatch.setattr(lookahead, "dead_share", lambda node, cfg: .25)
    node, root = score_fixture()
    # Independent hand calculation: waste + void + roughness + dead + buffer + NG + time.
    expected = .6 + .1 + .3 * np.sqrt(.0125) / 2. + .25 * .7 + .02 + .5 + .0005 * 15.
    assert LookaheadConfig().objective == "legacy"
    assert leaf_score(node, root, LookaheadConfig()) == pytest.approx(expected)


@pytest.mark.parametrize("old_time_weight", [0., .0005, .5])
def test_j_proxy_scales_nontime_score_and_counts_elapsed_time_once(monkeypatch, old_time_weight):
    from pac_highlevel import lookahead
    monkeypatch.setattr(lookahead, "dead_share", lambda node, cfg: .25)
    node, root = score_fixture()
    cfg = LookaheadConfig(objective="j_proxy", pallet_cost_s=2368., w_time=old_time_weight)
    nontime = .6 + .1 + .3 * np.sqrt(.0125) / 2. + .25 * .7 + .02 + .5
    assert leaf_score(node, root, cfg) == pytest.approx(2368. * nontime + 15.)
    later = replace_namespace(node, time_s=32.)
    assert leaf_score(later, root, cfg) - leaf_score(node, root, cfg) == pytest.approx(7.)


def replace_namespace(value, **updates):
    return SimpleNamespace(**{**vars(value), **updates})


@pytest.mark.parametrize("objective", ["legacy", "j_proxy"])
@pytest.mark.parametrize("improvement_factor, expected_action", [(.5, 0), (1.5, 1)])
def test_override_margin_uses_the_same_units_as_the_objective(objective, improvement_factor, expected_action):
    cfg = LookaheadConfig(objective=objective, horizon=1, margin=.005, pallet_cost_s=2368.)
    policy = LookaheadPolicy(HighLevelConfig(), cfg)
    world = SimpleNamespace(next_arrival=1, arrivals=[None, None], slots=1,
                            action_mask=lambda: np.array([True, True, False]))
    policy.rule = lambda current: from_index(0, current.slots)
    scale = 2368. if objective == "j_proxy" else 1.
    improvement = improvement_factor * .005 * scale
    policy.scores = lambda *args, **kwargs: {0: 100., 1: 100. - improvement}
    assert to_index(policy(world)) == expected_action


@pytest.mark.parametrize("value", [0., -1., float("nan"), float("inf"), -float("inf")])
def test_invalid_pallet_cost_is_rejected(value):
    with pytest.raises(ValueError):
        LookaheadConfig(pallet_cost_s=value)


@pytest.mark.parametrize("value", [-1, 1.5, True, float("nan"), float("inf")])
def test_invalid_measured_count_is_rejected(value):
    with pytest.raises(ValueError):
        LookaheadConfig(measured_preview_count=value)


@pytest.mark.parametrize("name", ["margin", "w_void", "w_rough", "w_buffer", "w_ng", "w_time", "w_dead",
                                  "dead_order_weight", "time_budget_s", "time_budget_ratio"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), -1.])
def test_nonfinite_or_negative_score_or_budget_config_is_rejected(name, value):
    with pytest.raises(ValueError):
        LookaheadConfig(**{name: value})


def visible_world(camera_weights=(5., 6.), hidden_weight=7., with_truth=False):
    weights = [1., 2., 3., 4., *camera_weights, hidden_weight]
    arrivals = [Arrival(make_box(f"B{i}", (.2, .2, .2), w),
                        truth=object() if with_truth else None) for i, w in enumerate(weights)]
    spec = SkuSpec("K", Size3D(.2, .2, .2), 10., (0.,), 10000.)
    world = PalletizingWorld(arrivals, Size3D(2., 2., 1.5), {"K": spec}, CandidateConfig(),
                             HighLevelConfig(), measured_preview_count=3,
                             weight_ranges={"K": (1., 10.)}, start=False)
    world.current = arrivals[0]
    world.next_arrival = 1
    world.remaining = Counter({"K": 6})
    return world


def test_only_first_three_preview_weights_are_observed_and_camera_weights_are_masked():
    world = visible_world()
    before = [arrival.box.weight_kg for arrival in world.arrivals]
    clone = window_clone(world, 6)
    assert [arrival.box.weight_kg for arrival in clone.arrivals] == [1., 2., 3., 4., 10., 10.]
    assert clone.known_weight_ids == frozenset({"B0", "B1", "B2", "B3"})
    assert [arrival.box.weight_kg for arrival in world.arrivals] == before


def test_camera_assumptions_are_not_labelled_as_measured_observations():
    clone = window_clone(visible_world(), 6)
    assert clone.assumed_weight_ids == frozenset({"B4", "B5"})
    for arrival in clone.arrivals[4:]:
        assert arrival.box.source == "CATALOG_WEIGHT_UPPER_BOUND"
        assert arrival.box.box_id not in clone.known_weight_ids


def test_search_copy_has_no_arrival_current_or_buffer_truth():
    world = visible_world(with_truth=True)
    buffered = Arrival(make_box("BUFFER", (.2, .2, .2), 3.), truth=object())
    world.buffer[0] = BufferEntry(buffered, 0, 0)
    clone = window_clone(world, 6)
    assert all(arrival.truth is None for arrival in clone.arrivals)
    assert clone.current.truth is None
    assert clone.buffer[0].arrival.truth is None
    assert world.current.truth is not None and world.buffer[0].arrival.truth is not None


def test_search_diagnostic_layout_uses_observed_boxes_instead_of_hidden_truth():
    world = visible_world()
    placed = PlacedBox("ALREADY", "K", Size3D(.2, .2, .2), 3., Pose3D("pallet", .5, .5, 0.))
    hidden = object()
    world.placed = [placed]
    world.placed_truth = {"ALREADY": hidden}
    clone = window_clone(world, 6)
    assert clone.placed_truth == {"ALREADY": placed}
    assert world.placed_truth["ALREADY"] is hidden


def test_child_steps_preserve_original_weight_knowledge_without_promotion():
    clone = window_clone(visible_world(), 6)
    known = clone.known_weight_ids
    child = _child(clone)
    assert child.known_weight_ids == known
    policy = LookaheadPolicy(clone.config, LookaheadConfig(measured_preview_count=3))
    for _ in range(4):
        child = policy._step(child, 0)
    assert child.current.box.box_id == "B4" and child.current.box.weight_kg == 10.
    assert child.known_weight_ids == known
    recloned = window_clone(child, 6)
    assert recloned.known_weight_ids == known
    assert recloned.current.box.box_id not in known


def test_changed_unmeasured_weights_do_not_change_masked_planning_inputs():
    left = window_clone(visible_world((1.5, 2.5), 1.), 6)
    right = window_clone(visible_world((8.5, 9.5), 200.), 6)
    assert left.current.box == right.current.box
    assert [a.box for a in left.arrivals] == [a.box for a in right.arrivals]
    assert left.action_mask().tolist() == right.action_mask().tolist()
    assert left.known_weight_ids == right.known_weight_ids


def test_legacy_visibility_mode_keeps_all_observed_weights():
    world = visible_world()
    world.measured_preview_count = None
    clone = window_clone(world, 6)
    assert [a.box.weight_kg for a in clone.arrivals] == [1., 2., 3., 4., 5., 6.]


def test_merged_future_context_ignores_unweighed_and_hidden_true_weights():
    from pac_highlevel.merged_placement import future_context_from_world
    left, right = visible_world((1., 2.), 1.), visible_world((9., 10.), 200.)
    a = future_context_from_world(left, left.current.box)
    b = future_context_from_world(right, right.current.box)
    assert a == b
    assert a.known_weights == (2., 3., 4.)
    assert a.known_box_ids == ("B1", "B2", "B3")
    assert a.camera_box_ids == ("B4", "B5")
    assert dict(a.remaining_by_sku) == {"K": 3}


def test_merged_future_context_labels_catalog_fallback_as_an_assumption():
    from pac_highlevel.merged_placement import future_context_from_world
    world = visible_world()
    world.weight_ranges = {}
    future = future_context_from_world(world, world.current.box)
    assert dict(future.weight_ranges) == {"K": (10., 10.)}
    assert dict(future.range_sources) == {"K": "catalog_nominal_degenerate_range_assumption"}


def test_future_context_has_no_reference_to_mutable_world():
    from pac_highlevel.merged_placement import future_context_from_world
    world = visible_world()
    future = future_context_from_world(world, world.current.box)
    before = future.heavier_share(4.)
    world.remaining["K"] = 1000
    world.weight_ranges["K"] = (100., 200.)
    world.arrivals[1] = replace(world.arrivals[1], box=replace(world.arrivals[1].box, weight_kg=500.))
    assert future.heavier_share(4.) == before
