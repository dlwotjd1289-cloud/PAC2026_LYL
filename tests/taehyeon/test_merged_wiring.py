"""Shared candidate ordering on actual snapshots, lookahead copies and repacks.

These are bounded unit fixtures. They do not run an order simulation or ROS.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import pytest

from pac_candidates import CandidateConfig
from pac_common import Size3D, SkuSpec
from pac_highlevel import Arrival, HighLevelConfig, HighLevelDecider, PalletizingWorld, load_policy
from pac_highlevel.lookahead import LookaheadConfig, _child, window_clone
from pac_highlevel.merged_placement import MergedConfig, MergedPlacer, future_context_from_world
from pac_highlevel.placement import LayerPlacer, make_placer
from pac_highlevel.runtime import validate_merged_placer_contract
from pac_runtime.placer import RobotAwarePlacer, layer_ranker, merged_ranker
from pac_runtime.ros_node import make_runtime_ranker
from th_helpers import make_box, make_context, make_state


class RecordingRobot:
    def __init__(self, reject_first=False):
        self.calls = []
        self.reject_first = reject_first

    def first_executable(self, box, ordered, state):
        self.calls.append(tuple(candidate.candidate_id for candidate in ordered))
        if not ordered:
            return None, None, {}
        if self.reject_first and len(ordered) > 1:
            bad = SimpleNamespace(codes=(SimpleNamespace(value="ROBOT_TEST_REJECTION"),))
            return ordered[1], None, {ordered[0].candidate_id: bad}
        return ordered[0], None, {}


def world_for(placer):
    current = make_box("NOW", weight=4.)
    future = [make_box(f"V{i}", weight=float(i+1)) for i in range(1, 7)]
    catalog = {"K": SkuSpec("K", current.size, 8., current.allowed_yaws_rad, 3000.)}
    world = PalletizingWorld([Arrival(item) for item in future], Size3D(1.1, 1.1, 1.5),
        catalog, CandidateConfig(), HighLevelConfig(), placer=placer,
        weight_ranges={"K": (1., 8.)}, measured_preview_count=3, visible_horizon=5, start=False)
    world.current = Arrival(current)
    return world


def candidates(world):
    box, state, backend = world.current.box, world.state(), world.backend()
    return box, state, backend, list(backend.candidate_set(box, state).valid)


def test_alpha_zero_preserves_exact_layer_scores_and_order():
    world = world_for(MergedPlacer(MergedConfig(blend=0.)))
    box, state, backend, valid = candidates(world)
    assert len(valid) > 1
    original = LayerPlacer().scores(valid, box, state, backend)
    future = future_context_from_world(world, box)
    merged = world.placer.scores(valid, box, state, backend, future)
    assert [row[:5] for row in merged] == [row[:5] for row in original]
    assert world.choose_candidate(valid, box, state).candidate_id == LayerPlacer()(valid, box, state, backend).candidate_id


def test_runtime_and_plain_world_use_identical_merged_order():
    cfg = MergedConfig(blend=.25)
    plain = world_for(MergedPlacer(cfg))
    robot = RecordingRobot()
    ranker = merged_ranker(cfg)
    runtime = world_for(RobotAwarePlacer(robot, max_checks=100, ranker=ranker))
    box, state, backend, valid = candidates(plain)
    expected = sorted(plain.placer.scores(valid, box, state, backend,
        future_context_from_world(plain, box)), key=lambda row: row[:5])
    chosen = runtime.choose_candidate(valid, runtime.current.box, runtime.state())
    assert robot.calls[-1] == tuple(row[-1].candidate_id for row in expected)
    assert chosen.candidate_id == expected[0][-1].candidate_id


def test_robot_filter_still_rejects_top_merged_candidate():
    robot = RecordingRobot(reject_first=True)
    world = world_for(RobotAwarePlacer(robot, max_checks=2, ranker=merged_ranker()))
    box, state, backend, valid = candidates(world)
    selected = world.choose_candidate(valid, box, state)
    assert len(robot.calls[-1]) == 2
    assert selected.candidate_id == robot.calls[-1][1]
    assert world.placer.stats["robot_rejected"] == 1


def test_window_and_child_use_calling_world_context(monkeypatch):
    ranker = merged_ranker()
    world = world_for(RobotAwarePlacer(RecordingRobot(), ranker=ranker))
    seen = []
    original = ranker.placer.scores

    def record(valid, box, state, backend, future_context=None):
        seen.append(future_context)
        return original(valid, box, state, backend, future_context)

    monkeypatch.setattr(ranker.placer, "scores", record)
    clone = window_clone(world, 5)
    clone.next_arrival = 1
    clone.current = clone.arrivals[0]
    clone.remaining["K"] -= 1
    child = _child(clone)
    child.next_arrival = 2
    child.current = child.arrivals[1]
    child.remaining["K"] -= 1
    for current_world in (world, clone, child):
        box, state, _, valid = candidates(current_world)
        current_world.choose_candidate(valid, box, state)
    assert seen[0].visible_box_ids == ("V1", "V2", "V3", "V4", "V5")
    assert seen[1].visible_box_ids == ("V2", "V3", "V4", "V5")
    assert seen[2].visible_box_ids == ("V3", "V4", "V5")
    assert not hasattr(ranker, "world") and not hasattr(ranker.placer, "world")
    assert world.next_arrival == 0 and world.current.box.box_id == "NOW"


def test_repack_hypothetical_state_is_forwarded_unchanged(monkeypatch):
    world = world_for(MergedPlacer())
    box, state, _, valid = candidates(world)
    hypothetical = replace(state, state_version=state.state_version+1)
    captured = []
    def choose(supplied_valid, supplied_box, supplied_state, supplied_backend, supplied_world):
        captured.append((supplied_state, supplied_world))
        return supplied_valid[0]
    monkeypatch.setattr(world.placer, "choose_in_world", choose)
    assert world.choose_candidate(valid, box, hypothetical) is valid[0]
    assert captured == [(hypothetical, world)]


def test_merged_ranker_is_independently_copyable():
    ranker = merged_ranker(MergedConfig(blend=.1))
    copied = deepcopy(ranker)
    assert copied is not ranker and copied.placer is not ranker.placer
    assert copied.placer.cfg == ranker.placer.cfg
    assert copied.name == "merged"


def test_merged_factory_and_runtime_yaml_use_same_parameters(tmp_path):
    path = tmp_path / "merged.yaml"
    path.write_text("merged:\n  blend: 0.1\n  horizon: 5\n")
    ranker = make_runtime_ranker("merged", merged_config_path=str(path))
    direct = make_placer("merged", MergedConfig(blend=.1))
    assert ranker.placer.cfg == direct.cfg
    assert ranker.provenance()["future_context"] == "current_world_only"


@pytest.mark.parametrize("policy_mode,ranker", [("merged", layer_ranker()), ("layer", merged_ranker())])
def test_mismatched_runtime_and_search_placement_rejected(policy_mode, ranker):
    cfg = LookaheadConfig(placer=policy_mode)
    policy = load_policy("lookahead", cfg, config=HighLevelConfig())
    with pytest.raises(ValueError, match="merged placement mismatch"):
        HighLevelDecider(make_context(), policy=policy,
                        placer=RobotAwarePlacer(RecordingRobot(), ranker=ranker))


def test_snapshot_preserves_explicit_3_plus_2_and_weight_ranges():
    cfg = LookaheadConfig(placer="merged", horizon=5, measured_preview_count=3)
    policy = load_policy("lookahead", cfg, config=HighLevelConfig())
    decider = HighLevelDecider(make_context(), policy=policy,
        placer=RobotAwarePlacer(RecordingRobot(), ranker=merged_ranker()), weight_ranges={"K": (1., 10.)})
    current = make_box("NOW", weight=4.)
    future = tuple(make_box(f"V{i}", weight=float(i+1)) for i in range(1, 6))
    state = make_state(tracked={"NOW": current})
    state = replace(state, inventory=replace(state.inventory, remaining_by_sku={"K": 7}))
    world = decider.snapshot_world(state, current_box_id="NOW", visible_boxes=future)
    assert world.weight_ranges == {"K": (1., 10.)}
    assert world.measured_preview_count == 3 and world.visible_horizon == 5
    context = future_context_from_world(world, current)
    assert context.known_box_ids == ("V1", "V2", "V3")
    assert context.visible_box_ids == ("V1", "V2", "V3", "V4", "V5")


def test_legacy_placement_contract_remains_accepted():
    policy = load_policy("lookahead", LookaheadConfig(placer="layer"), config=HighLevelConfig())
    validate_merged_placer_contract(policy, RobotAwarePlacer(RecordingRobot(), ranker=layer_ranker()))
