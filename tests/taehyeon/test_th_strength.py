import random

import pytest

from th_helpers import make_state, placed
from pac_candidates import CandidateConfig
from virtual_data.scenario_source import load_dataset
from virtual_data.strength import PROFILE_ORDER, STRENGTH_PROFILES, draw, true_overloads

from test_th_virtual_data import FIXTURE


def test_draw_is_deterministic_and_consistent():
    arrivals = load_dataset(FIXTURE).scenarios[0].arrivals
    for profile in PROFILE_ORDER:
        a = draw(arrivals, profile, random.Random(5))
        b = draw(arrivals, profile, random.Random(5))
        assert a == b
        assert set(a.true_capacity_n) == {x.box_id for x in arrivals}
        assert all(v > 0 for v in a.true_capacity_n.values())
        assert set(a.detected) <= set(a.damaged)
        assert len({a.sku_grade[x.sku_id] for x in arrivals}) >= 1


def test_profiles_are_ordered_by_strength():
    arrivals = load_dataset(FIXTURE).scenarios[0].arrivals

    def median(profile):
        values = []
        for seed in range(20):
            values += list(draw(arrivals, profile, random.Random(seed)).true_capacity_n.values())
        values.sort()
        return values[len(values) // 2]

    assert median("strong") > median("nominal") > median("extreme")
    assert median("weak") > median("extreme")


def test_true_overloads_detects_crushed_box():
    from virtual_data.strength import StrengthScenario

    a = placed("A", 0.002, 0.002, 0.0, weight=2)
    b = placed("B", 0.002, 0.002, 0.2, weight=20)
    state = make_state([a, b])
    weak = StrengthScenario("extreme", {}, {"A": 50.0, "B": 1000.0}, (), ())
    over, worst = true_overloads(state, weak, CandidateConfig())
    assert over == ["A"]
    assert worst == pytest.approx(20 * 9.80665 / 50.0)
    strong = StrengthScenario("strong", {}, {"A": 5000.0, "B": 5000.0}, (), ())
    assert true_overloads(state, strong, CandidateConfig())[0] == []


def test_profiles_registered():
    assert "extreme" in STRENGTH_PROFILES and len(PROFILE_ORDER) == len(STRENGTH_PROFILES)


def test_all_zero_profile_weights_rejected():
    from virtual_data.config import StrengthSection

    with pytest.raises(ValueError):
        StrengthSection(profiles={"strong": 0, "weak": 0})


def test_dblf_policy_ignores_output_order():
    from types import SimpleNamespace

    from pac_common import PlacementCandidate, Pose3D
    from virtual_data.policies import choose

    high = PlacementCandidate("C000", "N", Pose3D("pallet", 0.0, 0.0, 0.4), 0)
    low = PlacementCandidate("C001", "N", Pose3D("pallet", 0.5, 0.5, 0.0), 0)
    cset = SimpleNamespace(valid=(high, low))
    chosen, decision = choose("dblf", cset, random.Random(0), None)
    assert chosen is low and decision == "dblf"
