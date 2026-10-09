"""Selected-pose replay checks; one synthetic command, no order simulation."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pac_candidates import CandidateConfig
from pac_common import Size3D, SkuSpec
from pac_highlevel import HighLevelConfig, load_policy
from pac_robot_check import RobotFeasibility
from pac_runtime import CellSpec, RawObservation, RuntimeConfig, RuntimeCore
from pac_runtime.core import Command
from pac_runtime.placer import layer_ranker
from selected_trace import capture_selected, evaluate_snapshot


@pytest.fixture(scope="module")
def selected():
    # Same one-box model fixture as test_official_delivery, using original layer selection.
    size = Size3D(.4, .3, .2)
    cell = CellSpec((), {"K": 1}, Size3D(1.2, 1., 1.35),
        {"K": SkuSpec("K", size, 6., (0.,), 2000.)}, {"K": (2., 6.)})
    hl = HighLevelConfig()
    core = RuntimeCore(cell, CandidateConfig(), hl, RuntimeConfig(),
        RobotFeasibility(), load_policy("rule", config=hl), ranker=layer_ranker())
    core.on_observation(RawObservation("MODEL", "K", 4., size, 1., False, "top", 0.))
    command = core.next_command()
    assert command.action == "PLACE_CURRENT"
    return core, command


def test_capture_is_read_only_and_does_not_run_jaesung_calculation(selected, monkeypatch):
    import pac_highlevel.merged_placement as module
    core, command = selected
    state, before_command = deepcopy(core.sm.snapshot()), deepcopy(command)
    counts, stats = deepcopy(core.counts), deepcopy(core.placer.stats)
    def forbidden(*args, **kwargs):
        raise AssertionError("Capture must not calculate the Jaesung score")
    monkeypatch.setattr(module, "_jaesung_metrics", forbidden)
    snap = capture_selected(core, command, .25)
    assert snap["candidate"] == command.candidate
    assert snap["state"] == state
    assert core.sm.snapshot() == state and command == before_command
    assert core.counts == counts and core.placer.stats == stats
    assert snap["record"]["provenance"]["role"] == "post_decision_audit_only"


def test_selected_model_replay_is_pure_and_keeps_original_choice(selected):
    core, command = selected
    snap = capture_selected(core, command, .25)
    before = deepcopy(snap)
    state, chosen = deepcopy(core.sm.snapshot()), deepcopy(command)
    result = evaluate_snapshot(snap)
    assert snap == before and core.sm.snapshot() == state and command == chosen
    assert result == evaluate_snapshot(snap)
    evaluated = result["evaluation"]
    assert evaluated["evaluation_only"] is True
    assert evaluated["legacy_cost"]["value"] is not None
    assert evaluated["execution_feasibility"]["result_ok"]["value"] is None
    assert result["input"]["provenance"]["static_replay_success"] is True
    assert isinstance(result["input"]["load"]["cog_xyz_m"], list)


def test_calculation_failure_cannot_change_command_or_capture(selected, monkeypatch):
    import pac_highlevel.merged_placement as module
    core, command = selected
    snap = capture_selected(core, command, .25)
    before = deepcopy(snap)
    state, chosen = deepcopy(core.sm.snapshot()), deepcopy(command)
    def failed(*args, **kwargs):
        raise RuntimeError("synthetic audit calculation failure")
    monkeypatch.setattr(module, "_jaesung_metrics", failed)
    with pytest.raises(RuntimeError, match="synthetic audit calculation failure"):
        evaluate_snapshot(snap)
    assert snap == before and core.sm.snapshot() == state and command == chosen


@pytest.mark.parametrize("result_ok", [True, False])
def test_explicit_execution_outcome_preserved_without_changing_planned_metrics(selected, result_ok):
    core, command = selected
    snap = capture_selected(core, command, .25)
    missing = evaluate_snapshot(snap)
    snap["record"]["execution"]["result_ok"] = result_ok
    supplied = evaluate_snapshot(snap)
    assert supplied["evaluation"]["execution_feasibility"]["result_ok"]["value"] is result_ok
    assert supplied["evaluation"]["legacy_cost"] == missing["evaluation"]["legacy_cost"]
    assert supplied["input"]["provenance"]["geometry"] == "selected_planned_pose_model_not_measured_layout"


def test_wait_has_no_fabricated_geometry_or_execution_success(selected):
    core, _ = selected
    snap = capture_selected(core, Command("WAIT", core.sm.version), .1)
    assert set(snap) == {"record"}
    result = evaluate_snapshot(snap)["evaluation"]
    assert result["legacy_cost"]["value"] is None
    assert result["execution_feasibility"]["robot_ok"]["value"] is None
    assert result["execution_feasibility"]["result_ok"]["value"] is None
