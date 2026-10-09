"""Local score/observation contracts; no order run or physics simulation."""
from types import SimpleNamespace as NS

import pytest

from th_helpers import candidate, make_box, make_context, make_state, placed
from pac_candidates import CandidateBackend, CandidateConfig
from pac_highlevel.merged_placement import (
    FutureContext, MergedConfig, MergedPlacer, future_context_from_world,
    load_merged_config, merged_config_from_dict, piecewise_reward,
)
from pac_highlevel.placement import LayerPlacer


def world_for(preview=(), current=None, buffer=(), counts=None, ranges=None,
              measured=3, limit=5, known_ids=None):
    world = NS(current=NS(box=current) if current is not None else None,
               buffer=[NS(arrival=NS(box=b)) if b is not None else None for b in buffer],
               arrivals=[NS(box=b) for b in preview], next_arrival=0,
               remaining_by_sku=lambda: dict(counts or {}),
               catalog=make_context().catalog, weight_ranges=ranges or {},
               measured_preview_count=measured, visible_horizon=limit)
    if known_ids is not None:
        world.known_weight_ids = frozenset(known_ids)
    return world


@pytest.mark.parametrize("boxes", [(), (placed("A", 0., 0., 0.),),
    (placed("A", 0., 0., 0.), placed("B", .4, 0., 0., size=(.4, .3, .4)))])
def test_zero_blend_exactly_preserves_pr9_score_and_tie_order(backend, boxes):
    box, state = make_box(), make_state(boxes)
    valid = backend.candidate_set(box, state).valid
    assert valid
    expected = LayerPlacer().scores(valid, box, state, backend)
    actual = MergedPlacer(MergedConfig(blend=0)).scores(valid, box, state, backend)
    assert actual == expected
    assert min(actual, key=lambda row: row[:5]) == min(expected, key=lambda row: row[:5])


@pytest.mark.parametrize("blend", [0., .25, 1.])
def test_static_hard_mask_is_required_even_if_caller_passes_bad_candidate(backend, blend):
    box, state = make_box(), make_state()
    good = candidate(box, .004, .004, 0., cid="good")
    bad = candidate(box, .004, .004, .4, cid="unsupported")
    assert backend.validate_constraints(box, good, state).success
    assert not backend.validate_constraints(box, bad, state).success
    rows = MergedPlacer(MergedConfig(blend=blend)).scores([bad, good], box, state, backend)
    assert [r[-1].candidate_id for r in rows] == ["good"]


def test_open_pallet_edge_does_not_act_as_lateral_wall(backend):
    box, state = make_box(size=(.1, .1, .8)), make_state()
    candidates = [candidate(box, .004, .004, 0., cid="edge"),
                  candidate(box, .5, .5, 0., cid="middle")]
    placer = MergedPlacer()
    assert len(placer.scores(candidates, box, state, backend)) == 2
    edge, middle = [placer.last_details[k]["raw"] for k in ("edge", "middle")]
    assert edge["side"] == middle["side"] == 0.
    assert edge["critical_acceleration_g"] == middle["critical_acceleration_g"]
    # Low soft acceleration is a penalty, never a reintroduced .3 g hard filter.
    assert 0 < edge["critical_acceleration_g"] < .3
    assert edge["tilt"] < 1.


def test_real_neighbor_can_receive_proxy_side_credit(backend):
    box = make_box(size=(.2, .2, .4))
    state = make_state((placed("A", .1, .1, 0., size=(.2, .2, .4)),))
    candidates = [candidate(box, .308, .1, 0., cid="touch"),
                  candidate(box, .6, .6, 0., cid="open")]
    placer = MergedPlacer()
    assert len(placer.scores(candidates, box, state, backend)) == 2
    assert placer.last_details["touch"]["raw"]["side"] > 0.
    assert placer.last_details["open"]["raw"]["side"] == 0.
    assert "proxy" in placer.last_details["touch"]["side_support_model"]


def test_piecewise_defaults_preserve_old_s_and_l_definitions():
    cfg = MergedConfig()
    assert (cfg.s_target, cfg.s_lo, cfg.s_hi, cfg.s_weights) == (.75, 20., 2., (.2, .25, .4, .15))
    assert (cfg.l_target, cfg.l_lo, cfg.l_hi) == (.5, 8., .5)
    assert piecewise_reward(.75, cfg.s_target, cfg.s_lo, cfg.s_hi) == 15.
    assert piecewise_reward(1., cfg.s_target, cfg.s_lo, cfg.s_hi) == 15.5
    assert piecewise_reward(1., cfg.l_target, cfg.l_lo, cfg.l_hi) == 4.25


@pytest.mark.parametrize("kwargs", [dict(blend=-.1), dict(blend=1.01), dict(blend=float("nan")),
    dict(s_lo=0., s_hi=0.), dict(a_target_g=0.), dict(mu=0.), dict(horizon=6),
    dict(s_weights=(.2, .2, .2, .2)), dict(s_lo=1., s_hi=2.)])
def test_invalid_normalization_and_config_are_rejected(kwargs):
    with pytest.raises(ValueError):
        MergedConfig(**kwargs)


def test_configuration_loaders_are_consistent_and_reject_unrecognized_keys(tmp_path):
    path = tmp_path / "merged.yaml"
    path.write_text("merged:\n  blend: 0.4\n  horizon: 4\n", encoding="utf-8")
    assert load_merged_config(path) == merged_config_from_dict(dict(blend=.4, horizon=4))
    with pytest.raises(ValueError):
        merged_config_from_dict(dict(stack_g=.3))


def test_score_details_explain_each_contribution_once(backend):
    box, state = make_box(), make_state()
    c = candidate(box, .2, .2, 0.)
    placer = MergedPlacer()
    rows = placer.scores([c], box, state, backend)
    detail = placer.last_details[c.candidate_id]
    assert rows[0][0] == sum(detail["contributions"].values())
    assert detail["blend_assumption"] == .25
    assert detail["contributions"]["height"] == 0.
    assert "top" not in detail["contributions"]
    assert set(detail["contributions"]) == {"height", "corner", "flat_nonlevel", "flat_step",
        "flat_flush", "flat_global", "stability_contact", "stability_support",
        "stability_jaesung", "load_margin", "void", "future_block", "future_floor"}


def test_less_load_margin_increases_score_without_changing_safety_thresholds():
    box = make_box()
    state = make_state((placed("A", .1, .1, 0.),))
    c = candidate(box, .1, .1, .2)
    low_capacity = CandidateBackend(make_context(capacity_n=100.), CandidateConfig())
    high_capacity = CandidateBackend(make_context(capacity_n=1000.), CandidateConfig())
    placer = MergedPlacer()
    low = placer.scores([c], box, state, low_capacity)[0][0]
    low_details = placer.last_details[c.candidate_id]
    high = placer.scores([c], box, state, high_capacity)[0][0]
    high_details = placer.last_details[c.candidate_id]
    assert low > high
    assert low_details["raw"]["L"] < high_details["raw"]["L"]
    assert low_details["raw"]["S"] == high_details["raw"]["S"]
    assert low_capacity.context.catalog["K"].top_load_capacity_n == 100.


class CameraOnlyBox:
    def __init__(self, box_id, sku_id="K"):
        self.box_id, self.sku_id = box_id, sku_id

    @property
    def weight_kg(self):
        raise AssertionError("Camera-only exact weight must never be read")


class BoundedArrivals:
    def __init__(self, visible, start=0, limit=5):
        self.visible, self.start, self.limit = visible, start, limit
        self.reads = []

    def __getitem__(self, key):
        assert isinstance(key, slice)
        assert key.start == self.start and key.stop <= self.start+self.limit
        self.reads.append(key)
        return [NS(box=b) for b in self.visible[:key.stop-key.start]]

    def __iter__(self):
        raise AssertionError("Full hidden arrival iteration forbidden")

    def __len__(self):
        raise AssertionError("Hidden arrival count cannot be inferred from stream")


def test_three_weighed_two_camera_counts_once_and_excludes_current_self():
    box = make_box("current", weight=4.)
    preview = [make_box(f"P{i}", weight=6.+i) for i in range(3)]
    preview += [CameraOnlyBox("P3"), CameraOnlyBox("P4")]
    world = world_for(preview, current=box, buffer=[make_box("buffer", weight=7.)],
                      counts={"K": 8}, ranges={"K": (2., 12.)})
    context = future_context_from_world(world, box)
    assert context.known_box_ids == ("buffer", "P0", "P1", "P2")
    assert context.known_weights == (7., 6., 7., 8.)
    assert context.camera_box_ids == ("P3", "P4")
    assert context.remaining_by_sku == (("K", 5),)
    assert context.visible_box_ids == tuple(f"P{i}" for i in range(5))
    assert context.heavier_share(4.) == pytest.approx((4+5*.75)/9)


def test_choosing_buffer_excludes_itself_and_keeps_other_current_weight():
    current, buffered = make_box("current", weight=9.), make_box("buffer", weight=3.)
    context = future_context_from_world(world_for(current=current, buffer=[buffered]), buffered)
    assert context.known_box_ids == ("current",)
    assert context.known_weights == (9.,)
    assert context.heavier_share(3.) == 1.


def test_only_explicit_visible_slice_is_read_and_limit_is_respected():
    world = world_for(counts={"K": 100}, ranges={"K": (2., 8.)}, limit=2)
    world.next_arrival = 10
    world.arrivals = BoundedArrivals([make_box("P0"), make_box("P1")], start=10, limit=2)
    context = future_context_from_world(world, horizon=5)
    assert context.visible_box_ids == ("P0", "P1")
    assert context.remaining_by_sku == (("K", 98),)
    assert len(world.arrivals.reads) == 1


def test_unknown_order_totals_still_count_observed_camera_boxes():
    context = future_context_from_world(world_for([CameraOnlyBox("camera")], measured=0))
    assert context.remaining_by_sku == (("K", 1),)
    assert context.weight_ranges == (("K", (5., 5.)),)
    assert context.range_sources == (("K", "catalog_nominal_degenerate_range_assumption"),)


def test_search_provenance_does_not_promote_camera_box_when_it_moves_forward():
    world = world_for([CameraOnlyBox("earlier_camera"), make_box("known", weight=8.)],
        current=CameraOnlyBox("camera_current"), buffer=[CameraOnlyBox("camera_buffer")],
        counts={"K": 4}, ranges={"K": (2., 10.)}, known_ids={"known"})
    context = future_context_from_world(world, box=make_box("different_candidate"))
    assert context.known_box_ids == ("known",)
    assert context.known_weights == (8.,)
    assert context.camera_box_ids == ("camera_current", "camera_buffer", "earlier_camera")
    assert context.remaining_by_sku == (("K", 5),)
    assert context.heavier_share(5.) == pytest.approx((1+5*(4.5/8))/6)


def test_candidate_seen_in_preview_is_subtracted_but_never_its_own_future():
    box = make_box("P0", weight=4.)
    world = world_for([box, make_box("P1", weight=8.)], counts={"K": 2}, measured=2)
    context = future_context_from_world(world, box)
    assert context.known_box_ids == ("P1",)
    assert not context.remaining_by_sku
    assert context.heavier_share(4.) == 1.


def test_heavier_share_uses_strict_half_kg_boundary_and_declared_range():
    context = FutureContext(known_weights=(5.5, 5.6), remaining_by_sku=(("K", 2),),
                            weight_ranges=(("K", (4., 8.)),))
    assert context.heavier_share(5.) == pytest.approx((1.+2.*.625)/4.)
    assert FutureContext().heavier_share(5.) == 0.


def test_context_is_explicit_and_does_not_bind_placer_to_previous_world(backend):
    box, state = make_box(), make_state()
    valid = [candidate(box, .004, .004, 0.)]
    light = world_for(current=box, preview=[make_box("light", weight=2.)])
    heavy = world_for(current=box, preview=[make_box("heavy", weight=10.)])
    placer = MergedPlacer()
    placer.choose_in_world(valid, box, state, backend, light)
    first = placer.last_details["T"]
    placer.choose_in_world(valid, box, state, backend, heavy)
    second = placer.last_details["T"]
    placer.choose_in_world(valid, box, state, backend, light)
    assert placer.last_details["T"] == first
    assert second["score"] > first["score"]
    assert not hasattr(placer, "world")


def test_context_snapshot_does_not_change_when_source_count_mapping_changes():
    counts = {"K": 5}
    world = world_for(ranges={"K": (2., 8.)})
    world.remaining_by_sku = lambda: counts
    context = future_context_from_world(world)
    counts["K"] = 100
    assert context.remaining_by_sku == (("K", 5),)
