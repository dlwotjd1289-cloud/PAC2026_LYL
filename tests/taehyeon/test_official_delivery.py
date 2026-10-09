"""Evidence survives lookahead and reaches the command; no order simulation."""
import json
from dataclasses import replace

import pytest

from pac_candidates import CandidateConfig
from pac_common import Size3D, SkuSpec, ValidationResult
from pac_highlevel import HighLevelConfig, load_policy
from pac_highlevel.merged_placement import MergedConfig
from pac_runtime import CellSpec, RawObservation, RuntimeConfig, RuntimeCore
from pac_runtime.placer import RobotAwarePlacer, merged_ranker
from pac_runtime.ros_node import command_to_dict
from test_merged_wiring import world_for


class FixedRobot:
    def validate_robot_motion(self, box, candidate, state):
        return ValidationResult(True, (), {"cycle_time_s": 4., "arm_clearance_m": .05})


def official_placer():
    return RobotAwarePlacer(FixedRobot(), ranker=merged_ranker(MergedConfig(official_evaluation=True)))


def test_option_evidence_is_owned_and_checked_time_replaces_assumed_duration():
    placer = official_placer()
    world = world_for(placer)
    option, _ = world.options()
    assert option.feasible and option.official_evaluation["rows"]
    assert world._option_place_time(option) == 4.
    assert option.official_evaluation["candidate_generation_elapsed_s"] >= 0
    placer.last_evaluation.clear()
    world._placement_evaluation.clear()
    assert option.official_evaluation["rows"]  # copy, not shared mutable diagnostics
    before = world.time_s
    world.step(0)
    assert world.time_s-before == pytest.approx(4.)


def test_geometry_only_option_keeps_original_time():
    from pac_highlevel.merged_placement import MergedPlacer
    world = world_for(MergedPlacer())
    option, _ = world.options()
    assert world._option_place_time(option) == world.config.timing.place_time_s
    assert option.official_evaluation == {}


def test_command_json_contains_all_five_scores_and_measured_decision_duration():
    size = Size3D(.4, .3, .2)
    cell = CellSpec((), {"K": 1}, Size3D(1.1, 1.1, 1.5),
                    {"K": SkuSpec("K", size, 6., (0.,), 2000.)}, {"K": (2., 6.)})
    hl = HighLevelConfig()
    core = RuntimeCore(cell, CandidateConfig(), hl, RuntimeConfig(), FixedRobot(),
        load_policy("rule", config=hl), ranker=merged_ranker(MergedConfig(official_evaluation=True)))
    core.on_observation(RawObservation("NOW", "K", 4., size, 1., False, "top", 0.))
    command = core.next_command()
    assert command.action == "PLACE_CURRENT"
    evidence = command.evaluation
    assert evidence["box_id"] == "NOW"
    row = next(r for r in evidence["rows"] if r["candidate_id"] == command.candidate.candidate_id)
    keys = ("space_cost", "time_cost", "balance_cost", "stability_cost", "execution_cost")
    assert row["score_total"] == pytest.approx(sum(row[k] for k in keys))
    assert evidence["decision_timing"]["elapsed_s"] >= 0
    wire = json.loads(json.dumps(command_to_dict(command), allow_nan=False))
    assert wire["official_evaluation"]["selected_candidate_id"] == command.candidate.candidate_id


def test_one_decision_with_existing_robot_model():
    """One model decision, no physical movement and no order performance claim."""
    from pac_robot_check import RobotFeasibility
    from pac_runtime.config import load_runtime_config
    from th_helpers import REPO
    size = Size3D(.4, .3, .2)
    cell = CellSpec((), {"K": 1}, Size3D(1.2, 1., 1.35),
        {"K": SkuSpec("K", size, 6., (0.,), 2000.)}, {"K": (2., 6.)})
    hl = HighLevelConfig()
    core = RuntimeCore(cell, CandidateConfig(), hl,
        load_runtime_config(REPO / "config/jaesung_taehyeon/runtime.yaml"),
        RobotFeasibility(), load_policy("rule", config=hl),
        ranker=merged_ranker(MergedConfig(official_evaluation=True)))
    core.on_observation(RawObservation("MODEL", "K", 4., size, 1., False, "top", 0.))
    command = core.next_command()
    assert command.action == "PLACE_CURRENT"
    selected = next(r for r in command.evaluation["rows"]
                    if r["candidate_id"] == command.candidate.candidate_id)
    assert selected["status"] == "complete"
    assert command.robot["cycle_time_s"] == selected["robot_seconds"]
    assert not command.evaluation["decision_timing"]["exceeded"]
