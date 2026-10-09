"""Bounded robot-aware official criteria; fake robot, no order simulation."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from pac_common import PlacementCandidate, Pose3D, RejectCode, ValidationResult
from pac_runtime.placer import RobotAwarePlacer, official_robot_assumptions
import pac_runtime.placer as module


def candidate(name):
    return PlacementCandidate(name, "NOW", Pose3D("pallet", .1, .1, 0.), 4)


class Ranking:
    name = "merged"

    def __init__(self, scores, enabled=True, clock=None):
        self.scores = scores
        self.clock = clock
        self.placer = NS(cfg=NS(official_evaluation=enabled), last_details={})

    def __call__(self, valid, box, state, backend):
        if self.clock is not None:
            self.clock[0] += .07
        self.placer.last_details = {
            c.candidate_id: {"score": self.scores[c.candidate_id], "official": {
                "space_cost": self.scores[c.candidate_id], "balance_cost": 0.,
                "stability_cost": 0., "time_cost": None, "execution_cost": None,
                "status": "geometry_only"}} for c in valid}
        return sorted(valid, key=lambda c: self.scores[c.candidate_id])


class Robot:
    def __init__(self, metrics, clock=None):
        self.metrics, self.clock, self.calls = metrics, clock, []

    def validate_robot_motion(self, box, c, state):
        self.calls.append(c.candidate_id)
        if self.clock is not None:
            self.clock[0] += .6
        value = self.metrics[c.candidate_id]
        return (ValidationResult(False, (RejectCode.INVALID_STATE,), {}) if value is False
                else ValidationResult(True, (), value))

    def first_executable(self, *args):
        pytest.fail("official mode must compare checked candidates directly")


def pick(scores, metrics, **kwargs):
    robot = Robot(metrics)
    placer = RobotAwarePlacer(robot, ranker=Ranking(scores), **kwargs)
    out = placer([candidate(k) for k in scores], NS(box_id="NOW"), NS(state_version=4), None)
    return out, placer


def test_faster_successful_position_can_win_over_first_geometry_position():
    chosen, placer = pick({"slow": 0., "fast": .02}, {
        "slow": {"cycle_time_s": 16., "arm_clearance_m": .05},
        "fast": {"cycle_time_s": 4., "arm_clearance_m": .05}})
    assert chosen.candidate_id == "fast"
    evaluation = placer.last_evaluation
    assert evaluation["box_id"] == "NOW" and evaluation["state_version"] == 4
    assert evaluation["checked_count"] == 2 and evaluation["unscored_count"] == 0
    for row in evaluation["rows"]:
        assert row["score_total"] == pytest.approx(sum(row[k] for k in (
            "space_cost", "balance_cost", "stability_cost", "time_cost", "execution_cost")))


def test_more_clearance_can_win_with_identical_cycle_time():
    chosen, _ = pick({"tight": 0., "wide": .02}, {
        "tight": {"cycle_time_s": 8., "arm_clearance_m": 0.},
        "wide": {"cycle_time_s": 8., "arm_clearance_m": .1}})
    assert chosen.candidate_id == "wide"


@pytest.mark.parametrize("metrics", [False, {}, {"cycle_time_s": 1.},
    {"cycle_time_s": float("nan"), "arm_clearance_m": .1},
    {"cycle_time_s": 1., "arm_clearance_m": float("inf")},
    {"cycle_time_s": 1., "arm_clearance_m": None},
    {"cycle_time_s": True, "arm_clearance_m": .1}])
def test_failed_or_incomplete_robot_evidence_never_selected(metrics):
    chosen, placer = pick({"bad": 0., "ok": 1.}, {
        "bad": metrics, "ok": {"cycle_time_s": 8., "arm_clearance_m": .05}})
    assert chosen.candidate_id == "ok"
    assert placer.last_evaluation["unscored_count"] == 1


def test_no_complete_robot_evidence_means_no_candidate():
    chosen, placer = pick({"bad": 0.}, {"bad": {}})
    assert chosen is None and placer.last_evaluation["selected_candidate_id"] is None


def test_shortlist_never_checks_more_than_twelve():
    scores = {f"C{i:02d}": i*.001 for i in range(14)}
    chosen, placer = pick(scores, {k: {"cycle_time_s": 8., "arm_clearance_m": .05}
                                   for k in scores}, max_checks=100)
    assert chosen is not None and len(placer.robot.calls) == 12
    assert placer.last_evaluation["unscored_count"] == 2


def test_soft_budget_attempts_one_then_excludes_unchecked(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    scores = {"bad": 0., "unchecked": .1}
    robot = Robot({"bad": {}, "unchecked": {"cycle_time_s": 1., "arm_clearance_m": .1}}, clock)
    placer = RobotAwarePlacer(robot, ranker=Ranking(scores, clock=clock))
    assert placer([candidate(k) for k in scores], NS(box_id="NOW"), NS(state_version=4), None) is None
    assert robot.calls == ["bad"]
    assert placer.last_evaluation["ranking_elapsed_s"] == pytest.approx(.07)
    assert placer.last_evaluation["unscored_count"] == 2
    assert placer.last_evaluation["rows"][-1]["status"] == "unchecked_time_budget"


def test_ranking_elapsed_is_common_recorded_time_component(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    ranker = Ranking({"one": 0.}, clock=clock)
    placer = RobotAwarePlacer(Robot({"one": {"cycle_time_s": 8., "arm_clearance_m": .05}}), ranker=ranker)
    placer([candidate("one")], NS(box_id="NOW"), NS(state_version=4), None)
    row = placer.last_evaluation["rows"][0]
    assert row["computation_time_cost"] == pytest.approx(.1*.07/7.)
    assert row["time_cost"] == pytest.approx(.101)
    assert row["scope"] == "ranking_robot_checks_candidate_generation_not_measured"
    clone = deepcopy(placer)
    assert clone.last_evaluation == placer.last_evaluation
    assert clone.last_evaluation is not placer.last_evaluation


def test_flag_off_keeps_first_executable_contract_and_clears_evaluation():
    class LegacyRobot:
        def first_executable(self, box, ranked, state):
            return ranked[0], None, {}
        def validate_robot_motion(self, *args):
            pytest.fail("legacy mode must keep first_executable")
    placer = RobotAwarePlacer(LegacyRobot(), ranker=Ranking({"one": 0.}, enabled=False))
    placer.last_evaluation = {"stale": True}
    chosen = placer([candidate("one")], NS(box_id="NOW"), NS(state_version=4), None)
    assert chosen.candidate_id == "one" and placer.last_evaluation == {}
    assert official_robot_assumptions()["status"] == "initial_soft_assumptions_not_physical_constraints"


def test_world_snapshot_stays_separate_after_shared_placer_next_call():
    scores = {"first": 0., "second": 0.}
    placer = RobotAwarePlacer(Robot({k: {"cycle_time_s": 8., "arm_clearance_m": .05}
                                   for k in scores}), ranker=Ranking(scores))
    first_world, second_world = NS(), NS()
    for cid, world in (("first", first_world), ("second", second_world)):
        placer.choose_in_world([candidate(cid)], NS(box_id="NOW"), NS(state_version=4), None, world)
    assert first_world._placement_evaluation["selected_candidate_id"] == "first"
    assert second_world._placement_evaluation["selected_candidate_id"] == "second"
    assert first_world._placement_evaluation is not placer.last_evaluation
    cloned = deepcopy(placer)
    assert cloned._lock is not placer._lock


def test_actual_merged_geometry_metadata_completes_all_five_fields():
    from pac_candidates import CandidateBackend, CandidateConfig
    from pac_highlevel.merged_placement import MergedConfig
    from th_helpers import make_box, make_context, make_state

    box, state = make_box("NOW"), make_state(version=4)
    backend = CandidateBackend(make_context(), CandidateConfig())
    selected = candidate("real_geometry")
    ranker = module.merged_ranker(MergedConfig(official_evaluation=True))
    placer = RobotAwarePlacer(Robot({selected.candidate_id: {
        "cycle_time_s": 8., "arm_clearance_m": .05}}), ranker=ranker)
    assert placer([selected], box, state, backend) is selected
    row = placer.last_evaluation["rows"][0]
    assert row["status"] == "complete"
    assert row["score_total"] == pytest.approx(sum(row[k] for k in (
        "space_cost", "balance_cost", "stability_cost", "time_cost", "execution_cost")))


def test_full_pattern_time_is_common_to_all_completed_rows(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    class TimedRobot(Robot):
        def validate_robot_motion(self, *args):
            clock[0] += .02
            return super().validate_robot_motion(*args)
    scores = {"one": 0., "two": .01}
    ranker = Ranking(scores, clock=clock)
    robot = TimedRobot({k: {"cycle_time_s": 8., "arm_clearance_m": .05} for k in scores})
    placer = RobotAwarePlacer(robot, ranker=ranker)
    world = NS(_candidate_generation_elapsed_s=.7)
    placer.choose_in_world([candidate(k) for k in scores], NS(box_id="NOW"), NS(state_version=4), None, world)
    for row in world._placement_evaluation["rows"]:
        assert row["pattern_evaluation_elapsed_s"] == pytest.approx(.81)
        assert row["computation_time_cost"] == pytest.approx(.1*.81/7.)
        assert row["time_cost"] == pytest.approx(.1 + .1*.81/7.)
        assert row["scope"] == "candidate_generation_hard_mask_ranking_robot_checks_excludes_highlevel_search"
        assert ranker.placer.last_details[row["candidate_id"]]["official"]["score_total"] == row["score_total"]
