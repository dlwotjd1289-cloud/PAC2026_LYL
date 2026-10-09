"""Official geometry terms only; no episode or robot execution."""
import math
from types import SimpleNamespace

import pytest

from th_helpers import candidate, make_box, make_context, make_state, placed
from pac_candidates import CandidateBackend, CandidateConfig
from pac_highlevel.merged_placement import (
    MergedConfig, MergedPlacer, _capacity_utilization_concentration,
)
from pac_highlevel.placement import LayerPlacer


def test_unequal_strength_gets_equal_utilization_without_equal_force():
    assert _capacity_utilization_concentration([(50., 100.), (500., 1000.)]) == 0.
    assert _capacity_utilization_concentration([(90., 100.), (100., 1000.)]) > 0.
    # Unloaded boxes do not receive artificial force just to equalize a count.
    assert _capacity_utilization_concentration([(50., 100.), (0., 10000.)]) == 0.


@pytest.mark.parametrize("pairs", [[(2., 0.)], [(float('nan'), 10.)],
                                     [(-1., 10.)], [(12., 10.)]])
def test_invalid_load_evidence_is_rejected(pairs):
    with pytest.raises(ValueError):
        _capacity_utilization_concentration(pairs)


def test_pallet_center_is_preferred_within_existing_load_budget(backend):
    box, state = make_box(), make_state()
    candidates = [candidate(box, .004, .004, 0., cid='edge'),
                  candidate(box, .35, .40, 0., cid='center')]
    placer = MergedPlacer(MergedConfig(official_evaluation=True))
    assert len(placer.scores(candidates, box, state, backend)) == 2
    edge, center = [placer.last_details[c]["contributions"] for c in ('edge', 'center')]
    assert edge['pallet_cog'] > center['pallet_cog']
    maximum_budget = .25*(.6/15.5)*4.25
    for detail in placer.last_details.values():
        official = detail['official']
        assert 0 <= official['balance_cost'] <= maximum_budget
        assert official['space_cost']+official['balance_cost']+official['stability_cost'] == pytest.approx(detail['score'])
        assert official['time_cost'] is official['execution_cost'] is None
        assert official['status'] == 'geometry_only'


def test_real_lever_propagation_is_used_for_capacity_utilization():
    box = make_box(size=(.4, .2, .2))
    bases = (placed('A', .1, .1, 0., size=(.2, .2, .2)),
             placed('B', .3, .1, 0., size=(.2, .2, .2)))
    state = make_state(bases)
    backend = CandidateBackend(make_context(overrides={'A': 100., 'B': 1000.}), CandidateConfig())
    c = candidate(box, .1, .1, .2)
    placer = MergedPlacer(MergedConfig(official_evaluation=True))
    assert len(placer.scores([c], box, state, backend)) == 1
    raw = placer.last_details[c.candidate_id]['raw']
    # Two equal force shares but unequal strength give unequal utilization.
    assert raw['load_concentration'] > 0.
    assert raw['load_concentration_model'] == 'active_top_load_capacity_utilization_dispersion'
    assert raw['balance_weights_assumption'] == (.5, .25, .25)


def test_missing_balance_evidence_fails_closed(backend):
    box, state = make_box(), make_state()
    c = candidate(box, .2, .2, 0.)
    original = backend.validate_constraints
    def without_cog(box, cand, state):
        result = original(box, cand, state)
        metrics = dict(result.details['metrics'])
        metrics.pop('pallet_cog_margin_ratio')
        return SimpleNamespace(success=result.success, details={'metrics': metrics})
    backend.validate_constraints = without_cog
    placer = MergedPlacer(MergedConfig(official_evaluation=True))
    assert placer.scores([c], box, state, backend) == []
    assert placer.last_details[c.candidate_id]['fail_closed']
    assert placer.last_details[c.candidate_id]['official']['status'] == 'balance_evidence_unavailable'


def test_flag_off_preserves_original_layer_contract(backend):
    box, state = make_box(), make_state((placed('A', .1, .1, 0.),))
    valid = backend.candidate_set(box, state).valid
    assert valid
    scorer = MergedPlacer(MergedConfig(blend=0., official_evaluation=False))
    assert scorer.scores(valid, box, state, backend) == LayerPlacer().scores(valid, box, state, backend)


def test_official_flag_requires_boolean():
    with pytest.raises(ValueError):
        MergedConfig(official_evaluation='false')


@pytest.mark.parametrize("ratio", [None, float("nan"), float("inf"), -0.1, True])
def test_missing_or_invalid_load_ratio_fails_closed(backend, ratio):
    box, state = make_box(), make_state()
    c = candidate(box, .2, .2, 0.)
    original = backend.validate_constraints
    def invalid_ratio(box, cand, state):
        result = original(box, cand, state)
        metrics = dict(result.details["metrics"])
        if ratio is None:
            metrics.pop("max_load_ratio")
        else:
            metrics["max_load_ratio"] = ratio
        return SimpleNamespace(success=result.success, details={"metrics": metrics})
    backend.validate_constraints = invalid_ratio
    placer = MergedPlacer(MergedConfig(official_evaluation=True))
    assert placer.scores([c], box, state, backend) == []
    assert placer.last_details[c.candidate_id]["fail_closed"]
