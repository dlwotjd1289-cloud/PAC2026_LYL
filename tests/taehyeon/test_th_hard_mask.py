import math
import random

import pytest

from th_helpers import (
    HALF_PI,
    candidate,
    make_box,
    make_context,
    make_state,
    placed,
)
from pac_common import ConstraintEvidence, PlacementCandidate, Pose3D, RejectCode as R, Size3D
from pac_candidates import CandidateBackend, CandidateConfig, EVIDENCE_SOURCE
from pac_candidates.config import (
    ConstraintConfig,
    PalletCogConfig,
    UncertaintyConfig,
)
from pac_candidates.geometry import rotated_dims

TOL = 0.002  # default size tolerance
GAP = 0.004 + 2 * TOL  # required free gap between two boxes


def reasons(result):
    return {r.split(":", 1)[0] for r in result.details.get("reasons", ())}


def check(backend, box, state, x, y, z, yaw=0.0):
    return backend.validate_constraints(
        box, candidate(box, x, y, z, yaw, version=state.state_version), state
    )


def test_valid_floor_placement_has_typed_evidence(backend):
    box = make_box()
    result = check(backend, box, make_state(), TOL, TOL, 0.0)
    assert result.success, result.details
    ev = result.details["evidence"]
    assert isinstance(ev, ConstraintEvidence)
    assert ev.source == EVIDENCE_SOURCE
    assert ev.support_ratio == 1.0
    assert ev.support_centering == 0.5
    assert ev.dependency_count == 0
    assert 0 < ev.cog_margin_ratio <= 1
    assert ev.pallet_load_margin_ratio == pytest.approx(1 - 5 / 1000)


def test_out_of_bound(backend):
    box = make_box()
    assert R.OUT_OF_BOUND in check(backend, box, make_state(), 0.8, 0.0, 0.0).codes
    # touching the edge without the size tolerance is also rejected
    assert R.OUT_OF_BOUND in check(backend, box, make_state(), 0.0, TOL, 0.0).codes


def test_height_limit():
    backend = CandidateBackend(make_context())
    state = make_state([placed("A", TOL, TOL, 0.0)], pallet=Size3D(1.1, 1.1, 0.35))
    result = check(backend, make_box(weight=1), state, TOL, TOL, 0.2)
    assert R.HEIGHT_LIMIT in result.codes


def test_overlap_and_clearance(backend):
    state = make_state([placed("A", TOL, TOL, 0.0)])
    box = make_box()
    overlap = check(backend, box, state, 0.1, 0.1, 0.0)
    assert R.BOX_COLLISION in overlap.codes and "OVERLAP" in reasons(overlap)
    tight = check(backend, box, state, TOL + 0.4 + 0.002, TOL, 0.0)
    assert R.BOX_COLLISION in tight.codes and "CLEARANCE" in reasons(tight)
    ok = check(backend, box, state, TOL + 0.4 + GAP, TOL, 0.0)
    assert ok.success, ok.details


def test_overhead_box_blocks_descent(backend):
    state = make_state(
        [
            placed("A", TOL, TOL, 0.0),
            placed("B", 0.2, TOL, 0.2, size=(0.45, 0.3, 0.1), weight=1),
        ]
    )
    result = check(backend, make_box(size=(0.1, 0.1, 0.1), weight=0.5), state, 0.55, TOL, 0.0)
    assert R.APPROACH_FAIL in result.codes


def test_floating_rejected(backend):
    result = check(backend, make_box(), make_state(), TOL, TOL, 0.1)
    assert R.LOW_SUPPORT in result.codes and "FLOATING" in reasons(result)


def full_backend(ctx=None):
    return CandidateBackend(ctx or make_context(), CandidateConfig(collect_all_reasons=True))


def test_low_support_ratio():
    backend = full_backend()
    state = make_state([placed("A", TOL, TOL, 0.0, weight=10)])
    result = check(backend, make_box(weight=1), state, 0.25, TOL, 0.2)
    assert R.LOW_SUPPORT in result.codes and "SUPPORT_RATIO" in reasons(result)
    assert R.COG_VIOLATION in result.codes


def test_lbcp_rejects_load_on_overhang_even_with_full_contact(backend):
    """B overhangs A; only B's part above A can bear load (its LBCP)."""
    a = placed("A", TOL, TOL, 0.0, size=(0.4, 0.3, 0.2), weight=10)
    b = placed("B", 0.15, TOL, 0.2, size=(0.4, 0.3, 0.2), weight=5)
    state = make_state([a, b])
    small = make_box("N", (0.15, 0.25, 0.1), weight=1)
    on_overhang = check(backend, small, state, 0.40, TOL, 0.4)
    assert on_overhang.details["metrics"]["support_ratio"] == pytest.approx(1.0)
    assert R.COG_VIOLATION in on_overhang.codes
    assert "LBCP_UNSTABLE" in reasons(on_overhang)
    over_a = check(backend, small, state, 0.16, TOL, 0.4)
    assert over_a.success, over_a.details
    assert over_a.details["evidence"].dependency_count == 2


def test_bridge_over_two_supporters_is_stable(backend):
    a = placed("A", TOL, TOL, 0.0, size=(0.3, 0.3, 0.2), weight=10)
    b = placed("B", TOL + 0.3 + GAP, TOL, 0.0, size=(0.3, 0.3, 0.2), weight=10)
    state = make_state([a, b])
    bridge = make_box("N", (0.5, 0.3, 0.1), weight=4)
    result = check(backend, bridge, state, 0.06, TOL, 0.2)
    assert result.success, result.details
    shares = result.details["metrics"]["supporter_shares"]
    assert set(shares) == {"A", "B"}
    assert sum(shares.values()) == pytest.approx(1.0)


def test_heavy_on_light(backend):
    state = make_state([placed("A", TOL, TOL, 0.0, weight=2.0)])
    heavy = check(backend, make_box(weight=10), state, TOL, TOL, 0.2)
    assert R.LOAD_VIOLATION in heavy.codes and "HEAVY_ON_LIGHT" in reasons(heavy)
    similar = check(backend, make_box(weight=2.4), state, TOL, TOL, 0.2)
    assert similar.success, similar.details


def test_box_capacity_propagates_down():
    ctx = make_context(overrides={"A": 60.0})
    backend = CandidateBackend(ctx)
    a = placed("A", TOL, TOL, 0.0, weight=20)
    b = placed("B", TOL, TOL, 0.2, weight=5)  # 49 N on A < 60 N
    state = make_state([a, b])
    result = check(backend, make_box(weight=2), state, TOL, TOL, 0.4)
    assert R.LOAD_VIOLATION in result.codes
    assert any(r == "BOX_CAPACITY:A" for r in result.details["reasons"])
    light = check(backend, make_box(weight=0.5), state, TOL, TOL, 0.4)
    assert light.success, light.details
    assert light.details["evidence"].max_load_ratio == pytest.approx((5 + 0.5) * 9.80665 / 60)


def test_pallet_max_weight():
    backend = CandidateBackend(make_context(pallet_max=10.0))
    state = make_state([placed("A", TOL, TOL, 0.0, weight=8)])
    result = check(backend, make_box(weight=5), state, 0.6, 0.6, 0.0)
    assert "PALLET_MAX_WEIGHT" in reasons(result)


def test_pallet_cog_region_and_improving_rule():
    strict = ConstraintConfig(pallet_cog=PalletCogConfig(ramp_mass_kg=0.0))
    backend = CandidateBackend(make_context(), CandidateConfig(constraints=strict))
    box = make_box(weight=5)
    corner = check(backend, box, make_state(), TOL, TOL, 0.0)
    assert R.COG_VIOLATION in corner.codes and "PALLET_COG" in reasons(corner)
    center = check(backend, box, make_state(), 0.35, 0.4, 0.0)
    assert center.success, center.details
    # A placement that pulls an off-centre CoG back toward the middle is allowed.
    lopsided = make_state([placed("A", TOL, TOL, 0.0, weight=30)])
    toward = check(backend, make_box(weight=5), lopsided, 0.69, 0.79, 0.0)
    assert toward.success, toward.details


def test_progressive_cog_region_allows_first_corner_box(backend):
    assert check(backend, make_box(weight=5), make_state(), TOL, TOL, 0.0).success


def test_orientation_rules(backend):
    box = make_box(yaws=(0.0,))
    tilted = check(backend, box, make_state(), 0.3, 0.3, 0.0, yaw=math.pi / 4)
    assert R.INVALID_STATE in tilted.codes
    forbidden = check(backend, box, make_state(), 0.3, 0.3, 0.0, yaw=HALF_PI)
    assert "ORIENTATION_NOT_ALLOWED" in reasons(forbidden)
    upside = PlacementCandidate(
        "U", box.box_id, Pose3D("pallet", 0.3, 0.3, 0.0, roll=math.pi), 0
    )
    assert R.INVALID_STATE in backend.validate_constraints(box, upside, make_state()).codes


def test_identity_checks(backend):
    box = make_box("N")
    state = make_state(version=4)
    stale = backend.validate_constraints(box, candidate(box, TOL, TOL, 0, version=3), state)
    assert stale.codes == (R.STALE_PLAN,)
    other = candidate(make_box("X"), TOL, TOL, 0, version=4)
    assert backend.validate_constraints(box, other, state).codes == (R.INVALID_STATE,)
    done = make_state([placed("N", TOL, TOL, 0)], version=4)
    again = backend.validate_constraints(box, candidate(box, 0.6, 0.6, 0, version=4), done)
    assert "ALREADY_PLACED" in reasons(again)


def test_uncertain_policies():
    ctx = make_context(uncertain=("N",))
    reject = CandidateBackend(
        ctx, CandidateConfig(uncertainty=UncertaintyConfig(uncertain_policy="reject"))
    )
    box = make_box("N")
    assert check(reject, box, make_state(), 0.01, 0.01, 0).codes == (R.SENSOR_UNCERTAIN,)
    robust = CandidateBackend(ctx)
    assert R.OUT_OF_BOUND in check(robust, box, make_state(), TOL, TOL, 0).codes
    assert check(robust, box, make_state(), 2 * TOL, 2 * TOL, 0).success


def test_all_reasons_reported_together():
    backend = full_backend()
    state = make_state([placed("A", TOL, TOL, 0.0, weight=1)])
    result = check(backend, make_box(weight=20), state, 0.3, TOL, 0.2)
    assert {"SUPPORT_RATIO", "LBCP_UNSTABLE", "HEAVY_ON_LIGHT"} <= reasons(result)


def test_snapshot_issues_detect_bad_state(backend):
    a = placed("A", TOL, TOL, 0.0, weight=1)
    b = placed("B", 0.3, TOL, 0.2, weight=1)  # mostly hanging
    model = backend.model_for(make_state([a, b]))
    assert "UNSTABLE" in model.snapshot_issues()["B"]


# --------------------------------------------------------------------------
# Independent brute-force oracle for the geometric guarantees.
# --------------------------------------------------------------------------


def _aabb(pose, size):
    dx, dy, dz = rotated_dims(size, pose.yaw)
    return (pose.x, pose.y, pose.z), (pose.x + dx, pose.y + dy, pose.z + dz)


def _overlap(a, b, axis):
    return min(a[1][axis], b[1][axis]) - max(a[0][axis], b[0][axis])


@pytest.mark.parametrize("seed", range(10))
def test_valid_candidates_satisfy_independent_oracle(seed):
    rng = random.Random(seed)
    backend = CandidateBackend(make_context())
    boxes = []
    sizes = [(0.4, 0.3, 0.2), (0.3, 0.2, 0.15), (0.5, 0.4, 0.3), (0.25, 0.2, 0.1), (0.6, 0.4, 0.2)]
    for i in range(25):
        size = rng.choice(sizes)
        box = make_box(f"P{i}", size, weight=round(rng.uniform(1, 12), 2))
        state = make_state(boxes, version=i)
        result = backend.candidate_set(box, state)
        for cand in result.valid:
            lo, hi = _aabb(cand.target_pose, box.size)
            assert lo[0] >= TOL - 1e-9 and lo[1] >= TOL - 1e-9
            assert hi[0] <= 1.1 - TOL + 1e-9 and hi[1] <= 1.1 - TOL + 1e-9
            assert hi[2] + TOL <= 1.35 + 1e-9
            contact = 0.0
            for other in boxes:
                o = _aabb(other.pose, other.size)
                ox, oy, oz = (_overlap((lo, hi), o, k) for k in range(3))
                # no interpenetration and lateral clearance respected
                if oz > 0.003:
                    assert max(-ox, -oy) >= GAP - 1e-7
                if abs(o[1][2] - lo[2]) <= 0.003 and ox > 0 and oy > 0:
                    contact += ox * oy
            area = (hi[0] - lo[0]) * (hi[1] - lo[1])
            if lo[2] > 0.003:
                assert contact / area >= 0.7 - 1e-9
            else:
                assert lo[2] == 0.0
        if result.valid:
            pose = rng.choice(result.valid[:4]).target_pose
            boxes.append(placed(box.box_id, pose.x, pose.y, pose.z, size, box.weight_kg, pose.yaw))
    final = backend.model_for(make_state(boxes))
    assert final.snapshot_issues() == {}


@pytest.mark.parametrize("seed", range(4))
def test_fail_fast_and_full_modes_agree(seed):
    rng = random.Random(100 + seed)
    fast = CandidateBackend(make_context())
    full = full_backend()
    boxes = []
    for i in range(18):
        size = rng.choice([(0.4, 0.3, 0.2), (0.3, 0.2, 0.15), (0.5, 0.4, 0.3)])
        box = make_box(f"P{i}", size, weight=round(rng.uniform(1, 12), 2))
        state = make_state(boxes, version=i)
        cands = full.generate_candidates(box, state)
        for cand in cands:
            a = fast.validate_constraints(box, cand, state)
            b = full.validate_constraints(box, cand, state)
            assert a.success == b.success
            assert set(a.codes) <= set(b.codes)
            if a.success:
                assert a.details["evidence"] == b.details["evidence"]
        valid = [c for c in cands if full.validate_constraints(box, c, state).success]
        if valid:
            pose = rng.choice(valid[:3]).target_pose
            boxes.append(placed(box.box_id, pose.x, pose.y, pose.z, size, box.weight_kg, pose.yaw))


def test_heavy_on_light_share_mode_allows_bridging():
    from pac_candidates.config import HeavyOnLightConfig

    a = placed("A", TOL, TOL, 0.0, size=(0.3, 0.3, 0.2), weight=4)
    b = placed("B", TOL + 0.3 + GAP, TOL, 0.0, size=(0.3, 0.3, 0.2), weight=4)
    state = make_state([a, b])
    bridge = make_box("N", (0.5, 0.3, 0.1), weight=7)
    per_box = ConstraintConfig(heavy_on_light=HeavyOnLightConfig(mode="per_box"))
    strict = CandidateBackend(make_context(), CandidateConfig(constraints=per_box))
    assert "HEAVY_ON_LIGHT" in reasons(check(strict, bridge, state, 0.06, TOL, 0.2))
    share_cfg = ConstraintConfig(heavy_on_light=HeavyOnLightConfig(mode="share"))
    share = CandidateBackend(make_context(), CandidateConfig(constraints=share_cfg))
    assert check(share, bridge, state, 0.06, TOL, 0.2).success


# --------------------------------------------------------------------------
# Regressions found by the self-review (2026-10-08)
# --------------------------------------------------------------------------


def test_zero_weight_box_on_empty_pallet_does_not_crash(backend):
    box = make_box(weight=0.0)
    result = check(backend, box, make_state(), 0.3, 0.3, 0.0)
    assert result.success, result.details


def test_loaded_zero_capacity_box_keeps_evidence_finite():
    # A dented box (capacity override 0 N) that already carries a box: every
    # unrelated candidate must still be judged normally, with finite evidence.
    ctx = make_context(overrides={"D": 0.0})
    backend = CandidateBackend(ctx)
    d = placed("D", TOL, TOL, 0.0, weight=10)
    top = placed("T", TOL, TOL, 0.2, weight=1)
    state = make_state([d, top])
    elsewhere = check(backend, make_box(weight=2), state, 0.6, 0.6, 0.0)
    assert elsewhere.success, elsewhere.details
    assert math.isfinite(elsewhere.details["evidence"].max_load_ratio)
    on_top = check(backend, make_box(weight=0.5), state, TOL, TOL, 0.4)
    assert any(r == "BOX_CAPACITY:D" for r in on_top.details["reasons"])


def test_grid_maps_nearly_coincident_edges_consistently(backend):
    # Two edges 5e-8 apart are merged in the compressed grid; the second box
    # must still mark exactly its own cells (no fake or hidden EMS).
    a = placed("A", TOL, TOL, 0.0, size=(0.3, 0.3, 0.2))
    b = placed("B", TOL + 0.3 + GAP + 5e-8, TOL, 0.0, size=(0.3, 0.3, 0.2))
    model = backend.model_for(make_state([a, b]))
    xs, ys, heights = model.grid()
    centers = (xs[:-1] + xs[1:]) / 2
    row = heights[0]
    for cx, h in zip(centers, row):
        inside = any(g.expanded.x0 <= cx <= g.expanded.x1 for g in model.boxes)
        assert (h > 0) == inside


def test_incremental_loads_match_a_full_rebuild_with_lever_shares():
    """Regression: the mask must predict exactly the loads a re-check of the
    resulting pallet computes. With lever shares the split moves with the
    resultant, so re-using the old shares under-estimated loads (found as an
    OVERLOADED re-check on a heavy stack, 2026-10-08)."""
    from dataclasses import replace as dc_replace

    from pac_candidates.config import HeavyOnLightConfig
    from pac_candidates.pallet_model import G, PalletModel

    cfg = CandidateConfig()
    cfg = dc_replace(
        cfg, constraints=dc_replace(cfg.constraints, heavy_on_light=HeavyOnLightConfig(enabled=False))
    )
    ctx = make_context(capacity_n=1e6)
    rng = random.Random(5)
    boxes = []
    checked = 0
    for i in range(40):
        size = rng.choice([(0.4, 0.3, 0.2), (0.3, 0.2, 0.15), (0.5, 0.4, 0.25), (0.2, 0.2, 0.2)])
        box = make_box(f"N{i}", size, weight=rng.uniform(1, 40))
        state = make_state(boxes, version=i)
        backend = CandidateBackend(ctx, cfg)
        cset = backend.candidate_set(box, state)
        if not cset.valid:
            continue
        cand = rng.choice(cset.valid[:6])
        model = backend.model_for(state)
        verdict = backend.validate_constraints(box, cand, state)
        p = cand.target_pose
        new = placed(box.box_id, p.x, p.y, p.z, size, box.weight_kg, p.yaw)
        after = PalletModel(make_state([*boxes, new], version=i + 1), ctx, cfg)
        contacts = after.contacts[box.box_id]
        extra = model.propagate(contacts, box.weight_kg * G)
        for g in model.boxes:
            expected = after.top_load_n[g.box_id]
            got = model.top_load_n[g.box_id] + extra.get(g.box_id, 0.0)
            assert got == pytest.approx(expected, rel=1e-9, abs=1e-7), (i, g.box_id)
        assert verdict.success
        boxes.append(new)
        checked += 1
    assert checked >= 20
    assert any(len(PalletModel(make_state(boxes), ctx, cfg).contacts[b.box_id]) > 1 for b in boxes)


def test_off_centre_load_on_a_bridge_shifts_the_split():
    from pac_candidates.pallet_model import G, PalletModel

    ctx = make_context(capacity_n=1e6)
    cfg = CandidateConfig()
    a = placed("A", TOL, TOL, 0.0, size=(0.3, 0.3, 0.2), weight=30)
    b = placed("B", TOL + 0.3 + GAP, TOL, 0.0, size=(0.3, 0.3, 0.2), weight=30)
    c = placed("C", TOL, TOL, 0.2, size=(0.6 + GAP, 0.3, 0.1), weight=10)
    state = make_state([a, b, c], version=1)
    model = PalletModel(state, ctx, cfg)
    d = placed("D", TOL + 0.4, TOL, 0.3, size=(0.2, 0.3, 0.2), weight=10)  # over B's side
    after = PalletModel(make_state([a, b, c, d], version=2), ctx, cfg)
    extra = model.propagate(after.contacts["D"], d.weight_kg * G)
    for box_id in ("A", "B", "C"):
        assert model.top_load_n[box_id] + extra.get(box_id, 0.0) == pytest.approx(after.top_load_n[box_id])
    assert after.top_load_n["B"] > after.top_load_n["A"] + 50  # D's weight goes mostly to B


def test_loads_on_a_supporters_overhang_do_not_tip_it():
    """Review 2026-10-08: C rests on B's unsupported overhang and on D. Load
    must reach B only through B's load-bearing part, so B's resultant stays
    on A (it used to end 2 cm past A's edge, unnoticed)."""
    from pac_candidates.pallet_model import PalletModel

    ctx = make_context(capacity_n=1e5)
    cfg = CandidateConfig()
    boxes = [
        placed("A", TOL, TOL, 0.0, size=(0.70, 0.3, 0.2), weight=10),
        placed("B", TOL, TOL, 0.2, size=(1.0, 0.3, 0.1), weight=1.0),
        placed("D", 0.6, 0.312, 0.0, size=(0.49, 0.3, 0.3), weight=10),
    ]
    plan = [("C", 0.6, 0.1, 0.3, (0.49, 0.4, 0.1), 1.5), ("E0", 0.604, 0.104, 0.4, (0.2, 0.18, 0.1), 2.0)]
    for v, (bid, x, y, z, size, w) in enumerate(plan, start=1):
        b = make_box(bid, size, weight=w)
        r = CandidateBackend(ctx, cfg).validate_constraints(
            b, candidate(b, x, y, z, version=v), make_state(boxes, version=v)
        )
        if r.success:
            boxes.append(placed(bid, x, y, z, size=size, weight=w))
    after = PalletModel(make_state(boxes, version=9), ctx, cfg)
    assert after.snapshot_issues() == {}
    total = after._total_n["B"]
    assert after._moment["B"][0] / total <= 0.702 + 1e-9  # resultant on A


def test_share_mode_checks_supporters_below_min_share():
    from dataclasses import replace as dc_replace

    s = placed("S", TOL, TOL, 0.0, size=(0.848, 0.3, 0.2), weight=50)
    light = placed("L", 0.858, TOL, 0.0, size=(0.2, 0.3, 0.2), weight=0.5)
    c0 = CandidateConfig()
    cfg = dc_replace(c0, constraints=dc_replace(c0.constraints, pallet_cog=PalletCogConfig(enabled=False)))
    backend = CandidateBackend(make_context(capacity_n=1e5), cfg)
    h = make_box("H", (0.928, 0.3, 0.2), weight=40)
    result = check(backend, h, make_state([s, light], version=0), TOL, TOL, 0.2)
    shares = result.details["metrics"]["supporter_shares"]
    assert shares["L"] < cfg.constraints.heavy_on_light.min_share
    assert "HEAVY_ON_LIGHT" in reasons(result)  # 40 kg x 8.5 % = 3.4 kg on a 0.5 kg box


def test_lever_split_with_collinear_contacts_is_exact():
    from pac_candidates.loads import lever_shares

    shares = lever_shares([1.0, 1.0], [(0.1, 0.15), (0.5, 0.15)], (0.2, 0.2))
    assert shares == pytest.approx([0.75, 0.25])


def test_mask_accepted_stacks_always_pass_a_full_recheck():
    """Property: whatever the mask accepts, a from-scratch snapshot re-check
    finds no instability, tipping supporter or overload (random stacks)."""
    from pac_candidates.pallet_model import PalletModel

    for seed in range(6):
        rng = random.Random(100 + seed)
        ctx = make_context(capacity_n=rng.choice([60.0, 150.0, 1e5]))
        cfg = CandidateConfig()
        boxes = []
        for i in range(30):
            size = rng.choice([(0.4, 0.3, 0.2), (0.3, 0.2, 0.15), (0.5, 0.4, 0.25), (0.6, 0.25, 0.1)])
            box = make_box(f"N{i}", size, weight=rng.uniform(0.5, 30))
            state = make_state(boxes, version=i)
            valid = CandidateBackend(ctx, cfg).candidate_set(box, state).valid
            if not valid:
                continue
            p = rng.choice(valid[:8]).target_pose
            boxes.append(placed(box.box_id, p.x, p.y, p.z, size, box.weight_kg, p.yaw))
            assert PalletModel(make_state(boxes, version=i + 1), ctx, cfg).snapshot_issues() == {}, (seed, i)
