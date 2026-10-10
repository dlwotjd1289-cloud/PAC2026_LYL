#!/usr/bin/env python3
"""PAC2026 LYL placement for the Gazebo V4.6 closed loop (drop-in for ``ahead_planner_bridge_v44.py plan``).

    PLANNER_BRIDGE=<this file> bash scripts/run_generator_cycle_v44.sh DATASET SCENARIO COUNT

Same CLI, output JSON and exit codes as the workcell bridge (exit 20 + "PLAN FAIL: NO_SLOT" lets
the cycle put the box on the buffer table). Only the placement decision changes:

    workcell bridge : Donghan PlacementPlanner + ReferenceBackend
    this bridge     : Taehyeon pac_candidates (5-1 generation, 5-2 hard mask: support, load, CoG,
                      per-box top-load capacity) -> LayerPlacer order (the integration default)
                      -> the workcell's own reach check (TCP height over the PICK -> slot corridor)

The workcell geometry (pallet frame <-> Gazebo world, clearance, reach, SKU catalog, state file)
is reused from the workcell bridge (``PAC_WORKCELL_REPO``), so the cycle, ``commit`` and the
buffer logic stay exactly as they are. Not used here: the N=5 look-ahead (buffer / close
decisions stay the cycle's rules) and the virtual-cell robot model (pac_robot_check).
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

LYL = Path(__file__).resolve().parents[1]
WORKCELL = Path(os.environ.get("PAC_WORKCELL_REPO", LYL.parent / "pac2026_integrated")).resolve()

# LYL packages first: the workcell bridge then reuses the already imported pac_common.
sys.path[:0] = [str(p) for p in sorted((LYL / "ros2_ws/src").glob("pac_*")) if (p / p.name).is_dir()]
from pac_common.adapters import context_from_json, state_from_json  # noqa: E402
from pac_candidates import CandidateBackend, load_candidate_config  # noqa: E402
from pac_candidates.geometry import rotated_dims  # noqa: E402
from pac_runtime.placer import layer_ranker  # noqa: E402

_spec = importlib.util.spec_from_file_location("workcell_bridge", WORKCELL / "scripts/ahead_planner_bridge_v44.py")
wb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(wb)

CANDIDATES = LYL / "config/jaesung_taehyeon/candidates.yaml"


def centre(box, cand):
    p = cand.target_pose
    dx, dy, dz = rotated_dims(box.size, p.yaw)
    return p.x + dx / 2, p.y + dy / 2, p.z + dz / 2


def cmd_plan(a) -> int:
    st = wb.load_state(Path(a.state))
    per = json.loads(Path(a.perception).read_text())
    if per.get("status") != "OK":
        raise SystemExit(f"PLAN FAIL: perception status {per.get('status')}")
    if a.catalog:
        wb.load_generator_catalog(a.catalog)
    sku = a.sku or wb.match_sku(per["length"], per["width"])
    if a.sku:
        ref = sorted((wb.CATALOG[sku]["size"]["x"], wb.CATALOG[sku]["size"]["y"]))
        meas = sorted((per["length"], per["width"]))
        if any(abs(m - r) > wb.FOOTPRINT_TOL_M for m, r in zip(meas, ref)):
            raise SystemExit(f"PLAN FAIL: camera footprint {meas[1]:.3f} x {meas[0]:.3f} m does not match SKU "
                             f"{sku} {ref[1]:.3f} x {ref[0]:.3f} m")
    remaining = json.loads(a.remaining_json) if a.remaining_json else a.remaining
    pick_xy = (per.get("x", -1.03), per.get("y", 1.20)) if a.pick_xy is None else tuple(a.pick_xy)
    true_size = wb.CATALOG[sku]["size"]
    cand_cfg = load_candidate_config(CANDIDATES)
    rank = layer_ranker()
    started = time.perf_counter()
    chosen, notes, n_valid, n_rejected = None, [], 0, 0
    for height, mode in ((wb.NOMINAL_STACK_HEIGHT_M, "nominal"), (wb.MAX_STACK_HEIGHT_M, "allowance")):
        data = wb.scenario(st, a.box_id, sku, a.mass, per["yaw"], remaining, height)
        state, context = state_from_json(data["state"]), context_from_json(data["context"])
        box = state.inventory.tracked_boxes[a.box_id]
        backend = CandidateBackend(context, cand_cfg)
        cset = backend.candidate_set(box, state)
        n_valid, n_rejected = len(cset.valid), len(cset.rejected)
        unreachable = 0
        for order, cand in enumerate(rank(list(cset.valid), box, state, backend)):
            c = centre(box, cand)
            w = wb.pallet_to_world(*c)
            tz = wb.transfer_tcp_z(st, pick_xy, w[:2], true_size)
            if tz <= wb.REACH_TCP_MAX_Z:
                chosen = (cand, c, w, tz, mode, order)
                break
            unreachable += 1
        if unreachable:
            notes.append(f"{mode} {height:.2f} m: {unreachable} candidate(s) out of reach")
        if chosen:
            break
        if not cset.valid:
            notes.append(f"{mode} {height:.2f} m: no valid candidate ({n_rejected} rejected: {cset.code_counts})")
        if wb.MAX_STACK_HEIGHT_M <= wb.NOMINAL_STACK_HEIGHT_M:
            break
    if chosen is None:
        print("PLAN FAIL: NO_SLOT (" + "; ".join(notes) + ")", file=sys.stderr)
        return 20
    cand, c, world, tz, mode, order = chosen
    out = {"box_id": a.box_id, "sku_id": sku, "mass_kg": a.mass, "state_version": st["state_version"],
           "candidate_id": cand.candidate_id,
           # the cycle prints a number: LayerPlacer rank of the chosen slot (0 = best), not a cost
           "score": float(order), "score_basis": "LayerPlacer rank (0 = best)",
           "target_corner_pallet": [cand.target_pose.x, cand.target_pose.y, cand.target_pose.z],
           "yaw_rad": cand.target_pose.yaw, "center_pallet": list(c),
           "slot_world": [round(v, 4) for v in world], "n_ranked": n_valid, "n_rejected": n_rejected,
           "height_mode": mode,
           "stack_limit_m": wb.NOMINAL_STACK_HEIGHT_M if mode == "nominal" else wb.MAX_STACK_HEIGHT_M,
           "transfer_tcp_z": round(tz, 3), "notes": notes,
           "planning_time_sec": round(time.perf_counter() - started, 3), "perception": per,
           "planner": "PAC2026_LYL pac_candidates + LayerPlacer (Taehyeon) + workcell reach check",
           "model": "rule (LayerPlacer)"}
    Path(a.out).write_text(json.dumps(out, indent=2, default=float))
    print(f"{world[0]:.4f} {world[1]:.4f} {world[2]:.4f} {cand.target_pose.yaw:.4f}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan")
    p.add_argument("--state", required=True)
    p.add_argument("--box-id", required=True)
    p.add_argument("--mass", type=float, required=True)
    p.add_argument("--perception", required=True)
    p.add_argument("--remaining", type=int, default=0)
    p.add_argument("--remaining-json")
    p.add_argument("--sku")
    p.add_argument("--catalog")
    p.add_argument("--model", default=None, help="ignored (kept for CLI compatibility)")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--pick-xy", type=float, nargs=2, default=None)
    p.add_argument("--out", required=True)
    a, rest = ap.parse_known_args()
    if a.cmd != "plan":
        return 2
    return cmd_plan(a)


if __name__ == "__main__":
    sys.exit(main())
