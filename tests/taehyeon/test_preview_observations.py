"""Observation plumbing only: no order loop, search or robot execution."""
from dataclasses import replace

from pac_candidates import CandidateConfig
from pac_common import Size3D, SkuSpec
from pac_highlevel import HighLevelConfig, load_lookahead_config, load_policy
from pac_highlevel.lookahead import window_clone
from pac_highlevel.merged_placement import future_context_from_world
from pac_runtime import CellSpec, RawObservation, RuntimeConfig, RuntimeCore
from th_helpers import REPO


def core():
    catalog = {"K": SkuSpec("K", Size3D(.4, .3, .2), 20., (0.,), 2000.)}
    cell = CellSpec((), {"K": 10}, Size3D(1.1, 1.1, 1.5), catalog, {"K": (1., 20.)})
    config = load_lookahead_config(REPO / "config/jaesung_taehyeon/lookahead.yaml")
    hl = HighLevelConfig()
    return RuntimeCore(cell, CandidateConfig(), hl, RuntimeConfig(), object(),
                       load_policy("lookahead", config, config=hl))


def observation(index):
    return RawObservation(f"P{index}", "K", 2.+index,
                          Size3D(.30+.01*index, .22, .17), .9, False, "top", 12.)


def test_all_five_observed_sizes_and_weights_survive_preview_and_search_copy():
    runtime = core()
    observations = tuple(observation(i) for i in range(5))
    version = runtime.sm.version
    runtime.on_preview(observations)
    assert runtime.sm.version == version
    assert runtime._preview_obs == observations
    assert [(b.weight_kg, b.size) for b in runtime.preview] == [(o.weight_kg, o.size) for o in observations]
    world = runtime.decider.snapshot_world(runtime.sm.snapshot(), visible_boxes=runtime.preview)
    cloned = window_clone(world, 5)
    assert cloned.measured_preview_count == 5
    assert [(a.box.weight_kg, a.box.size) for a in cloned.arrivals] == [(o.weight_kg, o.size) for o in observations]
    assert all(a.truth is None for a in cloned.arrivals)
    future = future_context_from_world(cloned)
    assert future.known_box_ids == tuple(o.box_id for o in observations)
    assert future.known_weights == tuple(o.weight_kg for o in observations)
    assert future.camera_box_ids == ()
    assert dict(future.remaining_by_sku) == {"K": 5}
    assert dict(future.weight_ranges) == {"K": (1., 20.)}


def test_unknown_within_five_never_exposes_sixth_observation():
    class HiddenObservation:
        @property
        def label_sku(self):
            raise AssertionError("The sixth observation must not be read")
    runtime = core()
    visible = [replace(observation(0), label_sku=None)] + [observation(i) for i in range(1, 5)]
    runtime.on_preview(iter(visible+[HiddenObservation()]))
    assert [box.box_id for box in runtime.preview] == [f"P{i}" for i in range(1, 5)]


def test_invalid_measurements_are_not_replaced_with_catalogue_values():
    runtime = core()
    runtime.on_preview([
        replace(observation(0), weight_kg=float("nan")),
        replace(observation(1), weight_kg=-1.),
        observation(2),
    ])
    assert [box.box_id for box in runtime.preview] == ["P2"]
    assert runtime.counts["preview_invalid_observation"] == 2
