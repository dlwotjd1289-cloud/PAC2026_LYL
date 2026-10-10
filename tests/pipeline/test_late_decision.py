"""Over the 7 s decision budget: keep the computed command (profile) or WAIT (legacy)."""
from dataclasses import replace
import time

import pytest

from pac_runtime.config import RuntimeConfig
from pac_runtime.core import Command, RuntimeCore

from pipeline import REPO
from pac_runtime import load_runtime_config


def late_core(use_late):
    core = RuntimeCore.__new__(RuntimeCore)
    cfg = RuntimeConfig()
    core.cfg = replace(cfg, planning=replace(cfg.planning, decision_budget_s=0.01, use_late_command=use_late))
    core.counts, core.repacks_for_current = __import__("collections").Counter(), 0
    core.sm = type("SM", (), {"version": 3})()

    def slow(forecast=None, budget_s=None):
        time.sleep(0.03)
        return Command("PALLET_CLOSE", 3, reason="NO_FEASIBLE_ACTION")
    core._next_command = slow
    return core


@pytest.mark.parametrize("use_late, action", [(True, "PALLET_CLOSE"), (False, "WAIT")])
def test_late_command(use_late, action):
    core = late_core(use_late)
    cmd = core.next_command()
    assert cmd.action == action and core.last_decision_timing["exceeded"]
    key = "decision_budget_exceeded_used_best" if use_late else "decision_budget_exceeded"
    assert core.counts[key] == 1


def test_integration_profile_keeps_late_commands():
    planning = load_runtime_config(REPO / "config/jaesung_taehyeon/runtime.yaml").planning
    assert planning.decision_budget_s == 7.0 and planning.use_late_command is True
