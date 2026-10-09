"""Opt-in capacity/profile and whole-decision deadline result guard."""
from collections import Counter
from dataclasses import replace
from types import SimpleNamespace

import pytest

from pac_common import Size3D, SkuSpec
from pac_runtime.capacity import allowable_capacity_n, apply_site_capacity
from pac_runtime.config import RuntimeConfig, config_from_dict, load_runtime_config
from pac_runtime.core import Command, RuntimeCore
from pac_runtime.loop import CellSpec
from pac_runtime.order import configure_cell
from th_helpers import REPO


def test_merged_runtime_profile_keeps_fixed_capacity_and_seven_seconds():
    cfg = load_runtime_config(REPO / "config/jaesung_taehyeon/runtime.yaml")
    assert cfg.capacity.model == "jaesung_site" and cfg.capacity.site == "general"
    assert cfg.planning.max_budget_s == cfg.planning.decision_budget_s == 7
    size = Size3D(.4, .3, .2)
    cell = CellSpec((), {"K": 1}, Size3D(1.1, 1.1, 1.5),
                    {"K": SkuSpec("K", size, 7., (0.,), 123.)}, {"K": (2., 7.)})
    configured = configure_cell(cell, cfg)
    assert configured.catalog["K"].top_load_capacity_n == allowable_capacity_n(size, 7., "general")
    assert configured.pallet_size == cell.pallet_size
    assert configured.catalog["K"].weight_kg == cell.catalog["K"].weight_kg
    assert cell.catalog["K"].top_load_capacity_n == 123.
    assert configure_cell(cell, RuntimeConfig()) is cell


@pytest.mark.parametrize("name,value", [("decision_budget_s", -1), ("decision_budget_s", float("nan")),
                                       ("max_budget_s", float("inf"))])
def test_invalid_runtime_time_config_rejected(name, value):
    with pytest.raises(ValueError):
        config_from_dict({"planning": {name: value}})


@pytest.mark.parametrize("limit,elapsed,expected", [(7., 6.9, "PLACE_CURRENT"),
                                                    (7., 7.1, "WAIT"),
                                                    (0., 8., "PLACE_CURRENT")])
def test_complete_decision_guard_drops_late_command_without_committing_geometry(monkeypatch, limit, elapsed, expected):
    core = object.__new__(RuntimeCore)
    core.cfg = replace(RuntimeConfig(), planning=replace(RuntimeConfig().planning, decision_budget_s=limit))
    core.sm = SimpleNamespace(version=42, placed=())
    core.counts = Counter()
    core.repacks_for_current = 0
    times = iter((100., 100.+elapsed))
    monkeypatch.setattr("pac_runtime.core.time.perf_counter", lambda: next(times))
    passed_budgets = []

    def decide(forecast=None, budget_s=None):
        passed_budgets.append(budget_s)
        core.counts["PLACE_CURRENT"] += 1
        core.repacks_for_current += 1
        return Command("PLACE_CURRENT", 42, box_id="B")

    core._next_command = decide
    result = core.next_command(budget_s=10.)
    assert result.action == expected
    assert core.sm.placed == () and core.sm.version == 42
    assert core.last_decision_timing["elapsed_s"] == pytest.approx(elapsed)
    assert passed_budgets == [limit if limit else 10.]
    if expected == "WAIT":
        assert result.reason == "DECISION_BUDGET_EXCEEDED"
        assert core.counts["PLACE_CURRENT"] == 0 and core.counts["decision_budget_exceeded"] == 1
        assert core.repacks_for_current == 0
