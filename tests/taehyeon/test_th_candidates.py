import math
import random
import re

import pytest

from th_helpers import HALF_PI, make_box, make_context, make_state, placed
from pac_common import plain
from pac_candidates import CandidateBackend, CandidateConfig
from pac_candidates.config import GenerationConfig
from pac_candidates.geometry import rotated_dims, same_yaw


def test_empty_pallet_corners_respect_tolerance(backend):
    box = make_box()
    state = make_state()
    report = backend.generate_with_report(box, state)
    poses = [c.target_pose for c in report.candidates]
    tol = backend.config.uncertainty.size_tolerance_m
    assert any(
        p.x == pytest.approx(tol) and p.y == pytest.approx(tol) and p.yaw == 0 for p in poses
    )
    # far corner keeps the same tolerance to the pallet edge
    assert any(
        p.x == pytest.approx(1.1 - 0.4 - tol) and p.y == pytest.approx(1.1 - 0.3 - tol)
        for p in poses
    )
    assert all(p.z == 0.0 for p in poses)
    assert report.ems_count == 1


def test_candidate_ids_unique_and_contract_format(backend):
    box = make_box("B007")
    state = make_state([placed("A", 0.002, 0.002, 0)], version=12)
    cands = backend.generate_candidates(box, state)
    ids = [c.candidate_id for c in cands]
    assert len(ids) == len(set(ids))
    assert all(re.fullmatch(r"S0012-B007-C\d{3}", i) for i in ids)
    assert all(c.base_state_version == 12 and c.box_id == "B007" for c in cands)
    assert all(c.target_pose.frame_id == "pallet" for c in cands)


def test_yaws_follow_box_and_config():
    backend = CandidateBackend(make_context())
    only0 = make_box(yaws=(0.0,))
    assert {c.target_pose.yaw for c in backend.generate_candidates(only0, make_state())} == {0.0}
    square = make_box(size=(0.3, 0.3, 0.2))
    yaws = {c.target_pose.yaw for c in backend.generate_candidates(square, make_state())}
    assert yaws == {0.0}  # 90 deg gives the same AABB -> kept once
    cfg = CandidateConfig(generation=GenerationConfig(yaw_set_rad=(HALF_PI,)))
    b2 = CandidateBackend(make_context(), cfg)
    assert {c.target_pose.yaw for c in b2.generate_candidates(make_box(), make_state())} == {
        HALF_PI
    }


def _random_state(seed, n=12):
    rng = random.Random(seed)
    backend = CandidateBackend(make_context())
    boxes = []
    for i in range(n):
        size = rng.choice([(0.4, 0.3, 0.2), (0.3, 0.2, 0.15), (0.5, 0.4, 0.3), (0.25, 0.2, 0.15)])
        box = make_box(f"P{i}", size, weight=rng.uniform(1, 10))
        state = make_state(boxes)
        valid = backend.candidate_set(box, state).valid
        if not valid:
            continue
        pose = rng.choice(valid[:5]).target_pose
        boxes.append(placed(box.box_id, pose.x, pose.y, pose.z, size, box.weight_kg, pose.yaw))
    return backend, make_state(boxes)


@pytest.mark.parametrize("seed", range(6))
def test_dedup_and_resting_invariants(seed):
    backend, state = _random_state(seed)
    box = make_box("N", (0.3, 0.25, 0.2))
    model = backend.model_for(state)
    cands = backend.generate_candidates(box, state)
    assert cands, "no candidates generated"
    d = backend.config.generation.dedup_distance_m
    for i, a in enumerate(cands):
        pa = a.target_pose
        dx, dy, _ = rotated_dims(box.size, pa.yaw)
        tol = backend.config.uncertainty.size_tolerance_m
        exp = model.half_gap + tol
        from pac_candidates.geometry import Rect

        expanded = Rect(pa.x - exp, pa.y - exp, pa.x + dx + exp, pa.y + dy + exp)
        assert pa.z == pytest.approx(model.resting_z(expanded))  # rests on something
        for b in cands[i + 1 :]:
            pb = b.target_pose
            if same_yaw(pa.yaw, pb.yaw) and abs(pa.z - pb.z) <= 0.003:
                assert max(abs(pa.x - pb.x), abs(pa.y - pb.y)) >= d - 1e-9


def test_generation_is_deterministic_and_non_mutating():
    backend, state = _random_state(1)
    before = plain(state)
    box = make_box("N")
    a = backend.generate_candidates(box, state)
    b = CandidateBackend(make_context()).generate_candidates(box, state)
    assert a == b
    assert plain(state) == before


def test_extreme_point_next_to_placed_box(backend):
    state = make_state([placed("A", 0.002, 0.002, 0.0)])
    box = make_box("N")
    report = backend.generate_with_report(box, state)
    gap = 0.004 + 2 * 0.002
    xs = {
        round(c.target_pose.x, 6)
        for c in report.candidates
        if c.target_pose.z == 0 and c.target_pose.y < 0.01 and c.target_pose.yaw == 0
    }
    assert round(0.002 + 0.4 + gap, 6) in xs  # flush with clearance to box A
    sources = {report.infos[c.candidate_id].source for c in report.candidates}
    assert sources <= {"EMS", "EP"}


def test_top_of_box_candidates_exist(backend):
    state = make_state([placed("A", 0.002, 0.002, 0.0, size=(0.6, 0.5, 0.2), weight=20)])
    cands = backend.candidate_set(make_box("N", (0.3, 0.25, 0.2), weight=2), state).valid
    assert any(c.target_pose.z == pytest.approx(0.2) for c in cands)


def test_max_candidates_cap():
    cfg = CandidateConfig(generation=GenerationConfig(max_candidates=3))
    backend = CandidateBackend(make_context(), cfg)
    _, state = _random_state(2)
    assert len(backend.generate_candidates(make_box("N"), state)) <= 3


def test_mask_aware_dedup_never_loses_valid_cluster():
    _, state = _random_state(3)
    box = make_box("N", (0.3, 0.25, 0.2), weight=1)
    geo = CandidateBackend(
        make_context(), CandidateConfig(generation=GenerationConfig(dedup_mode="geometric"))
    )
    aware = CandidateBackend(
        make_context(), CandidateConfig(generation=GenerationConfig(dedup_mode="mask_aware"))
    )
    off = CandidateBackend(
        make_context(), CandidateConfig(generation=GenerationConfig(dedup_mode="off"))
    )
    n_geo = geo.candidate_set(box, state).valid_count
    n_aware = aware.candidate_set(box, state).valid_count
    n_off = off.candidate_set(box, state).valid_count
    assert n_aware >= n_geo
    assert n_off >= n_aware
    if n_off:
        assert n_aware > 0


def test_ems_context_augmentation(backend):
    state = make_state([placed("A", 0.002, 0.002, 0.0)])
    box = make_box("N")
    report = backend.generate_with_report(box, state)
    ctx = backend.context_with_ems(make_context(), report)
    assert ctx.ems_upper_by_candidate
    for cid, upper in ctx.ems_upper_by_candidate.items():
        cand = next(c for c in report.candidates if c.candidate_id == cid)
        dx, dy, dz = rotated_dims(box.size, cand.target_pose.yaw)
        assert upper.x - cand.target_pose.x >= dx - 1e-9
        assert upper.y - cand.target_pose.y >= dy - 1e-9
        assert upper.z - cand.target_pose.z >= dz - 1e-9


def test_uncertain_box_gets_wider_clearance():
    ctx = make_context(uncertain=("N",))
    backend = CandidateBackend(ctx)
    cands = backend.generate_candidates(make_box("N"), make_state())
    assert min(c.target_pose.x for c in cands) == pytest.approx(0.004)


def test_full_pallet_has_no_floor_candidates(backend):
    boxes = []
    k = 0
    for i in range(2):
        for j in range(3):
            boxes.append(placed(f"F{k}", 0.002 + i * 0.55, 0.002 + j * 0.366, 0.0, (0.54, 0.358, 0.2)))
            k += 1
    state = make_state(boxes)
    cands = backend.candidate_set(make_box("N", (0.3, 0.2, 0.2), weight=1), state).valid
    assert cands
    assert all(c.target_pose.z == pytest.approx(0.2) for c in cands)
    assert math.isclose(min(c.target_pose.z for c in cands), 0.2)


@pytest.mark.parametrize("seed", range(5))
def test_likely_valid_first_order_serves_first_n_consumers(seed):
    """donghan's inventory probe validates only the first 16 candidates."""
    backend, state = _random_state(10 + seed, n=16)
    for size, weight in (((0.3, 0.25, 0.2), 1.0), ((0.4, 0.3, 0.2), 4.0), ((0.25, 0.2, 0.1), 0.5)):
        box = make_box("N", size, weight=weight)
        cset = backend.candidate_set(box, state)
        if cset.valid:
            first = backend.generate_candidates(box, state)[:16]
            assert any(backend.validate_constraints(box, c, state).success for c in first)


def test_priority_order_option():
    cfg = CandidateConfig(generation=GenerationConfig(order="priority"))
    backend = CandidateBackend(make_context(), cfg)
    _, state = _random_state(4)
    cands = backend.generate_candidates(make_box("N"), state)
    keys = [(round(c.target_pose.z, 6), round(c.target_pose.y, 6), round(c.target_pose.x, 6)) for c in cands]
    assert keys == sorted(keys)


def test_balance_scan_runs_when_the_low_estimate_fails_the_mask():
    """Review 2026-10-08: the lowest estimate-passing spot (on X, z=0.1) is
    rejected by the mask (X's capacity override 1 N), so the level above the
    light boxes must still be scanned; it used to be cut off (0 valid)."""
    boxes = [placed("X", 0.002, 0.002, 0.0, size=(0.3, 1.096, 0.1), weight=20.0)]
    x = 0.31
    for i, w in enumerate((0.35, 0.43)):
        boxes.append(placed(f"L{i}", x, 0.002, 0.0, size=(w, 1.096, 0.2), weight=2.0))
        x += w + 0.008
    backend = CandidateBackend(make_context(overrides={"X": 1.0}), CandidateConfig())
    cset = backend.candidate_set(make_box("H", (0.3, 0.3, 0.2), weight=4.0), make_state(boxes))
    assert cset.valid
    assert all(c.target_pose.z == pytest.approx(0.2) for c in cset.valid)


@pytest.mark.parametrize("yaws", [(math.pi,), (-math.pi / 2,), (3 * math.pi / 2,), (0.0, math.pi)])
def test_allowed_yaws_are_matched_by_footprint(backend, yaws):
    box = make_box("N", (0.4, 0.3, 0.2), yaws=yaws)
    cands = backend.generate_candidates(box, make_state())
    assert cands
    for c in cands:
        assert any(abs(c.target_pose.yaw - a) < 1e-9 for a in yaws)  # value from the box's own list
    assert backend.candidate_set(box, make_state()).valid


def test_dedup_readmits_candidates_hidden_by_a_replaced_representative():
    """Review 2026-10-08: R0 (invalid) hides R1; R2 (valid) replaces R0 but
    is too far from R1 to represent it -> R1 must come back."""
    from types import SimpleNamespace

    from pac_candidates.candidate_generation import deduplicate

    raws = [SimpleNamespace(x=x, y=0.0, z=0.0, yaw=0.0) for x in (0.10, 0.03, 0.17)]
    valid = {id(raws[0]): False, id(raws[1]): False, id(raws[2]): True}
    kept, _ = deduplicate(raws, 0.08, 0.003, lambda r: valid[id(r)])
    assert [r.x for r in kept] == [0.03, 0.17]


def test_uncertain_reject_policy_generates_nothing():
    from pac_candidates.config import UncertaintyConfig

    cfg = CandidateConfig(uncertainty=UncertaintyConfig(uncertain_policy="reject"))
    backend = CandidateBackend(make_context(uncertain=("N",)), cfg)
    assert backend.generate_candidates(make_box("N"), make_state()) == []
    assert backend.generate_candidates(make_box("M"), make_state())  # certain box unaffected
