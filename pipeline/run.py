"""End-to-end virtual run: data generation -> stacking finished.

    python -m pipeline.run --output runs/demo                # generate + run every order
    python -m pipeline.run --output runs/demo --ids S0001    # one order
    python -m pipeline.run --output runs/x --dataset runs/demo/dataset   # reuse a dataset

Stages per box (one RuntimeCore, as on the ROS node):
  0  tools/ahead_dataset_generator      orders (arrival sequence, SKU, weight)
  1  pipeline.depth_perception          scale + label + Donghan depth algorithm
  2  pac_runtime.state_validator        OK / SPEC_MISMATCH / DAMAGED / RECOGNITION_FAIL
  3  pac_runtime.supervisor             pallet change, missing boxes, buffer
  4  pac_highlevel (lookahead N=5)      place / buffer / retrieve / close / repack
  5  pac_candidates + LayerPlacer       Taehyeon candidate generation and ranking
  6  pac_robot_check                    HDR50-22 reach / collision / cycle time
  7  pac_runtime.executor (virtual)     placement noise, grip failures, post check
  8  pac_runtime.state_manager          measured state -> next box
  +  tools/algorithm_integration        Jaesung 5-criteria evaluation (after the order)
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import statistics
import sys
import time
import traceback

from . import REPO, bootstrap

bootstrap()

import yaml  # noqa: E402

from pac_candidates import load_candidate_config  # noqa: E402
from pac_highlevel import load_highlevel_config, load_lookahead_config, load_policy  # noqa: E402
from pac_perception.perception_config import load_perception_config  # noqa: E402
from pac_robot_check import RobotFeasibility, load_robot_check_config  # noqa: E402
from pac_runtime import RuntimeCore, RuntimeLoop, load_runtime_config  # noqa: E402
from pac_runtime.order import configure_cell  # noqa: E402
from pac_runtime.ros_node import make_runtime_ranker  # noqa: E402
from selected_trace import capture_selected, evaluate_snapshot  # noqa: E402
from virtual_data import load_virtual_config  # noqa: E402
from virtual_data.runtime_cell import runtime_cell  # noqa: E402
from virtual_data.scenario_source import load_dataset, run_generator  # noqa: E402

from .depth_camera import CameraSpec  # noqa: E402
from .depth_perception import DepthPerception  # noqa: E402
from .dashboard_export import export_order, run_id_for  # noqa: E402

PROFILE = REPO / "config/jaesung_taehyeon"


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")


def load_pipeline_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError(f"unsupported schema_version: {data.get('schema_version')}")
    return data


def generate(cfg, out_dir):
    g = cfg["generator"]
    print(f"[0] generating dataset -> {out_dir}", flush=True)
    print(run_generator(REPO / "tools/ahead_dataset_generator", REPO / "ros2_ws/src/pac_common", out_dir,
                        g["mode"], g["sample_per_family"], seed=g.get("seed"),
                        boxes_per_scenario=g["boxes_per_scenario"]), flush=True)
    return out_dir


def perception_factory(cell, cfg, rt, seed):
    p = cfg["perception"]
    depth_cfg, resolver_cfg = load_perception_config(REPO / p["config"])
    if p.get("roi_xy_m"):
        depth_cfg = replace(depth_cfg, roi_xy_m=tuple(p["roi_xy_m"]))
    camera = CameraSpec(**p.get("camera", {}))
    made = {}

    def make(stream):
        made[stream] = DepthPerception(cell, depth_cfg, resolver_cfg, rt.perception, f"{seed}:{stream}",
                                       camera=camera, queue_probability=p.get("queue_probability", 0.5),
                                       **p.get("damage", {}))
        return made[stream]
    return make, made


def perception_summary(records, stream):
    """Depth stage scored against the hidden truth (after the run)."""
    first = {}
    for r in records:  # the preview camera re-reads queued boxes; score each (box, view) once
        first.setdefault((r["box_id"], r["view"]), r)
    records = list(first.values())
    measured = [r for r in records if "measured_size" in r]
    errors = []
    for r in measured:
        t, m = sorted(r["true_size"][:2], reverse=True) + [r["true_size"][2]], r["measured_size"]
        errors.append(max(abs(a - b) for a, b in zip(t, m)) * 1000)
    top = {}
    for r in measured:
        if r["view"] == "top":
            top.setdefault(r["box_id"], r)
    damaged = [r for r in top.values() if r["true_damaged"]]
    intact = [r for r in top.values() if not r["true_damaged"]]
    unlabelled = [r for r in top.values() if not r["label_read"]]
    return {
        "measurements": len(records),
        "failed": len(records) - len(measured),
        "size_error_mm_max_side": {"mean": round(statistics.mean(errors), 2) if errors else None,
                                   "max": round(max(errors), 2) if errors else None},
        "damage_true": len(damaged),
        "damage_detected": sum(bool(r["damage_reasons"]) for r in damaged),
        "damage_false_alarm": sum(bool(r["damage_reasons"]) for r in intact),
        "damage_kinds": dict(Counter(k for r in damaged for k in r["damage_reasons"])),
        "label_unread": len(unlabelled),
        "label_resolved_by_size_weight": sum(r["sku_resolved"] for r in unlabelled),
        "label_resolved_wrong": sum(r["sku_resolved"] and r["sku"] != r["true_sku"] for r in unlabelled),
        "boxes_in_roi_gt1": sum(r["boxes_in_roi"] > 1 for r in top.values()),
        "stream": stream,
    }


def decision_summary(decisions, budget_s):
    """Report the active deadline; retain the historical 7 s field for readers."""
    times = [d["elapsed_s"] for d in decisions]
    return {"count": len(times), "max_s": max(times, default=None),
            "mean_s": round(statistics.mean(times), 4) if times else None,
            "budget_s": budget_s,
            "over_budget": sum(t > budget_s for t in times) if budget_s else 0,
            "over_7s": sum(t > 7.0 for t in times), "log": decisions}


def run_order(spec, index, ds, cfg, configs, out_dir, dashboard=None):
    cc, hl, la, rt, vc = configs
    c = cfg["cell"]
    decisions, snapshots, pending, holder = [], [], {}, []

    class RecordingCore(RuntimeCore):
        """Times every decision and keeps the selected step for the offline evaluation."""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            holder.append(self)

        def next_command(self, *a, **kw):
            started = time.perf_counter()
            cmd = super().next_command(*a, **kw)
            decisions.append({"elapsed_s": round(time.perf_counter() - started, 4), "action": cmd.action,
                              "reason": cmd.reason, "box_id": cmd.box_id})
            snap = capture_selected(self, cmd, decisions[-1]["elapsed_s"])
            snapshots.append(snap)
            pending[id(cmd)] = snap
            return cmd

        def on_result(self, cmd, report=None):
            level = super().on_result(cmd, report)
            snap = pending.pop(id(cmd), None)
            if snap is not None and report is not None:
                snap["record"].setdefault("execution", {})["result_ok"] = bool(report.ok)
                snap["record"].setdefault("provenance", {})["postcheck_level"] = level
            return level

    cell = configure_cell(runtime_cell(spec, ds, cc, vc, scenario_index=index, family_index=0, seed=c["seed"],
                                       spec_mismatch_probability=c["spec_mismatch_probability"],
                                       missing_probability=c["missing_probability"],
                                       pallet_xy=tuple(c["pallet_xy_m"])), rt)
    make, made = perception_factory(cell, cfg, rt, f"{c['seed']}:{spec.scenario_id}")
    ranker = make_runtime_ranker(cfg["run"]["ranker"], merged_config_path=str(PROFILE / "merged.yaml"))
    robot = RobotFeasibility(load_robot_check_config(PROFILE / "robot_check.yaml"))
    loop = RuntimeLoop(cell, cc, hl, rt, robot, load_policy("lookahead", la, config=hl),
                       ranker=ranker, log_events=True, perception_factory=make, core_class=RecordingCore)

    row = {"scenario_id": spec.scenario_id, "family": spec.family, "ordered": len(spec.arrivals),
           "stream_boxes": len(cell.stream), "pallet_size": asdict(cell.pallet_size)}
    limit = int(cfg["run"].get("order_wall_limit_s") or 0)

    def timeout(*_):
        raise TimeoutError("ORDER_WALL_LIMIT")
    previous = signal.signal(signal.SIGALRM, timeout) if limit else None
    signal.alarm(limit) if limit else None
    started = time.perf_counter()
    try:
        out = loop.run()
        rejected = {x["box_id"] for x in out["inspection"]}
        row.update(status="complete", result=out,
                   unprocessed=len(cell.stream) - out["placed"] - len(rejected))
        if row["unprocessed"]:
            row["status"] = "runtime_stopped"
    except Exception as error:  # noqa: BLE001 - recorded, the next order still runs
        row.update(status="timeout" if isinstance(error, TimeoutError) else "error", error=str(error),
                   traceback=traceback.format_exc())
    finally:
        if limit:
            signal.alarm(0)
            signal.signal(signal.SIGALRM, previous)
    row["wall_s"] = round(time.perf_counter() - started, 2)
    row["decisions"] = decision_summary(decisions, rt.planning.decision_budget_s)
    row["perception"] = {name: perception_summary(p.records, name) for name, p in made.items()}
    row["perception_records"] = {name: p.records for name, p in made.items()}
    t0 = time.perf_counter()
    evals = []
    for snap in snapshots:
        try:
            evals.append(evaluate_snapshot(snap))
        except Exception as error:  # noqa: BLE001
            evals.append({"input": snap["record"], "error": str(error)})
    row["evaluation"] = evals
    row["offline_evaluation_s"] = round(time.perf_counter() - t0, 3)
    write(out_dir / f"{spec.scenario_id}.json", row)
    if dashboard:
        row["dashboard_run"] = str(export_order(
            row, run_id_for(spec.scenario_id, dashboard), {f.truth.box_id: (f.truth.size.x, f.truth.size.y,
                                                                             f.truth.size.z) for f in cell.stream},
            start_epoch=time.time() - row["wall_s"]))
    return row


def order_line(row):
    r = row.get("result") or {}
    p = row["perception"].get("perception", {})
    return {
        "order": row["scenario_id"], "family": row["family"], "status": row["status"],
        "ordered": row["ordered"], "arrived": row["stream_boxes"], "placed": r.get("placed"),
        "inspection": len(r.get("inspection", [])), "unprocessed": row.get("unprocessed"),
        # after an early stop the loop books every box still to come as MISSING
        "missing": sum((r.get("missing") or {}).values()) if row["status"] == "complete" else None,
        "stop_reason": (row["decisions"]["log"][-1]["reason"] if row["status"] == "runtime_stopped"
                        and row["decisions"]["log"] else row.get("error")),
        "pallets": r.get("pallets"), "fill_mean": round(r["fill_true_mean"], 4) if r else None,
        "cell_time_s": r.get("time_s"), "decision_max_s": row["decisions"]["max_s"],
        "decision_budget_s": row["decisions"].get("budget_s"),
        "decision_over_budget": row["decisions"].get("over_budget"),
        "decision_over_7s": row["decisions"]["over_7s"], "wall_s": row["wall_s"],
        "damage_detected": f"{p.get('damage_detected', 0)}/{p.get('damage_true', 0)}",
        "damage_false_alarm": p.get("damage_false_alarm", 0),
        "size_err_mm_mean": p.get("size_error_mm_max_side", {}).get("mean"),
    }


def report(lines, out_dir, meta):
    done = [x for x in lines if x["status"] == "complete"]
    md = ["# PAC2026 LYL pipeline run", "",
          f"- created: {meta['created_utc']}", f"- dataset: `{meta['dataset']}`",
          f"- orders: {len(lines)} (complete {len(done)})",
          "- virtual cell only: depth images are synthetic, robot motion is the kinematic model, "
          "not a physical test.", ""]
    keys = list(lines[0]) if lines else []
    md += ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
    md += ["| " + " | ".join(str(x[k]) for k in keys) + " |" for x in lines]
    if done:
        md += ["", f"Mean pallet fill over complete orders: "
               f"{statistics.mean(x['fill_mean'] for x in done):.3f} "
               "(mean of order-level pallet fill, including the final used pallet)."]
    (out_dir / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    write(out_dir / "summary.json", {"meta": meta, "orders": lines})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--config", type=Path, default=REPO / "config/pipeline.yaml")
    ap.add_argument("--dataset", type=Path, help="existing generator output (skip stage 0)")
    ap.add_argument("--ids", nargs="+", help="scenario ids to run (default: all)")
    ap.add_argument("--boxes", type=int, help="override generator.boxes_per_scenario")
    ap.add_argument("--per-family", type=int, help="override generator.sample_per_family")
    ap.add_argument("--wall-limit", type=int, help="override run.order_wall_limit_s")
    ap.add_argument("--visible", type=int,
                    help="boxes seen ahead on the conveyor (look-ahead horizon and measured previews; profile: 5)")
    ap.add_argument("--no-dashboard", action="store_true",
                    help="do not write logs/pipeline/<run> nor register it with the dashboard")
    args = ap.parse_args(argv)
    cfg = load_pipeline_config(args.config)
    if args.boxes:
        cfg["generator"]["boxes_per_scenario"] = args.boxes
    if args.per_family:
        cfg["generator"]["sample_per_family"] = args.per_family
    if args.wall_limit is not None:
        cfg["run"]["order_wall_limit_s"] = args.wall_limit
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    dataset_dir = args.dataset or generate(cfg, out / "dataset")
    ds = load_dataset(dataset_dir)
    configs = (load_candidate_config(PROFILE / "candidates.yaml"), load_highlevel_config(PROFILE / "highlevel.yaml"),
               load_lookahead_config(PROFILE / "lookahead.yaml"), load_runtime_config(PROFILE / "runtime.yaml"),
               load_virtual_config(REPO / "config/taehyeon/virtual_data.yaml"))
    if args.visible:
        # conveyor boxes seen ahead = look-ahead horizon = measured previews (one setting)
        cc, hl, la, rt, vc = configs
        hl = replace(hl, close=replace(hl.close, visible_boxes=args.visible))
        la = replace(la, horizon=args.visible, measured_preview_count=args.visible)
        configs = (cc, hl, la, rt, vc)
        cfg["run"]["visible_boxes"] = args.visible
    chosen = [(i, s) for i, s in enumerate(ds.scenarios) if not args.ids or s.scenario_id in args.ids]
    if args.ids and len(chosen) != len(args.ids):
        sys.exit(f"unknown ids: {set(args.ids) - {s.scenario_id for _, s in chosen}}")
    meta = {"created_utc": datetime.now(timezone.utc).isoformat(), "dataset": str(dataset_dir),
            "config": cfg, "profile": str(PROFILE.relative_to(REPO)),
            "effective_configs": {name: asdict(value) for name, value in zip(
                ("candidates", "highlevel", "lookahead", "runtime", "virtual_data"), configs)},
            "robot_check": asdict(load_robot_check_config(PROFILE / "robot_check.yaml"))}
    (out / "orders").mkdir(exist_ok=True)
    lines = []
    for index, spec in chosen:
        print(f"[order] {spec.scenario_id} {spec.family} boxes={len(spec.arrivals)}", flush=True)
        row = run_order(spec, index, ds, cfg, configs, out / "orders",
                        dashboard=None if args.no_dashboard else meta["created_utc"])
        lines.append(order_line(row))
        print("        " + json.dumps(lines[-1], ensure_ascii=False), flush=True)
        if row.get("dashboard_run"):
            print(f"        dashboard run: {row['dashboard_run']} (active)", flush=True)
        report(lines, out, meta)
    print(f"report: {out / 'summary.md'}")


if __name__ == "__main__":
    main()
