"""Original Taehyeon candidates with the optional weight-order rule disabled.

Capacity-only balance anchors were removed from the default selection path.
These bounded fixtures check the restored behavior without order simulation.
"""
from dataclasses import replace

from pac_candidates import CandidateBackend, CandidateConfig
from pac_candidates.candidate_generation import raw_candidates
from th_helpers import make_box, make_context, make_state, placed


def fixture(balance=True, capacities=(120., 120.)):
    config = CandidateConfig()
    config = replace(config,
        generation=replace(config.generation, balance_anchors=balance, use_extreme_points=False),
        constraints=replace(config.constraints,
                            heavy_on_light=replace(config.constraints.heavy_on_light, enabled=False)))
    state = make_state([
        placed("L", .002, .002, 0., (.54, 1.096, .2), weight=30.),
        placed("R", .55, .002, 0., (.548, 1.096, .2), weight=30.),
    ])
    box = make_box("N", (.4, .4, .2), weight=20., yaws=(0.,))
    backend = CandidateBackend(make_context(overrides=dict(zip(("L", "R"), capacities))), config)
    return backend, state, box


def test_original_generator_skips_balance_scan_without_weight_order_rule(monkeypatch):
    import pac_candidates.candidate_generation as generation

    def unexpected_scan(*args, **kwargs):
        raise AssertionError("Original generator must not scan balance anchors with the rule off")

    monkeypatch.setattr(generation, "_balance_raw", unexpected_scan)
    backend, state, box = fixture()
    raw = raw_candidates(backend.model_for(state), box, backend.config)
    assert raw and all(item.anchor != "balance" for item in raw)
    assert not backend.config.constraints.heavy_on_light.enabled


def test_balance_flag_does_not_change_raw_or_valid_candidates_when_rule_is_off():
    off, state, box = fixture(balance=False)
    on, _, _ = fixture(balance=True)
    plain = raw_candidates(off.model_for(state), box, off.config)
    restored = raw_candidates(on.model_for(state), box, on.config)
    assert restored == plain
    # Compare IDs, order and poses, not just an unordered position subset.
    assert on.candidate_set(box, state).valid == off.candidate_set(box, state).valid


def test_original_candidates_still_use_existing_exact_hard_mask():
    backend, state, box = fixture(capacities=(300., 300.))
    candidates = backend.candidate_set(box, state).valid
    assert candidates
    for candidate in candidates:
        verdict = backend.validate_constraints(box, candidate, state)
        assert verdict.success
        assert verdict.details["metrics"]["max_load_ratio"] <= 1.


def test_original_candidates_do_not_relax_allowable_load():
    backend, state, box = fixture(capacities=(1., 1.))
    assert not backend.candidate_set(box, state).valid


def test_no_balance_scan_on_empty_pallet():
    backend, _, box = fixture()
    raw = raw_candidates(backend.model_for(make_state()), box, backend.config)
    assert raw and all(item.anchor != "balance" for item in raw)
