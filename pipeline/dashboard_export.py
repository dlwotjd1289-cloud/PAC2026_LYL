"""Hand one finished order to the failure-analysis dashboard (read-only side car).

The dashboard (``tools/pac_failure_analysis_dashboard``, contract in its
``INTEGRATION_GUIDE_KO.md``) reads only files under ``<repo>/logs``:

    logs/pipeline/<run_id>/
        pipeline_cycle.log   timeline; FAIL / ERROR / WARN lines feed the failure analysis
        boxes.json           {box_id: {"size_m": [x, y, z]}}  (true carton sizes)
        events.jsonl         executor claims: on_conveyor, pallet_placed, buffer_stored,
                             ng_diverted, failed (one line per event, simulated clock)
        world.sdf            static scene: pallets and the final virtual layout
    logs/pac_dashboard_integration/active_run.json   -> this run

There is no Gazebo here: the 3D boxes are the *planned and virtually executed*
layout drawn as static geometry, not observed poses.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys
from xml.sax.saxutils import escape

from . import REPO

DASHBOARD = REPO / "tools/pac_failure_analysis_dashboard"
PALLET_GAP_M = 0.4          # pallets of one order drawn side by side along x
DECK_M = 0.15

EVENT_MAP = {               # loop event -> dashboard event (success flag)
    "ARRIVE": ("on_conveyor", True),
    "PLACE": ("pallet_placed", True),
    "BUFFER": ("buffer_stored", True),
    "INSPECTION": ("ng_diverted", True),
    "NG": ("ng_diverted", True),
    "L3_GRIP_FAIL": ("failed", False),
}


def run_id_for(scenario_id, created_utc):
    stamp = re.sub(r"[^0-9]", "", created_utc)[:14]
    return f"{scenario_id}_{stamp}"


def _log_lines(row):
    """Timeline in the words the dashboard's failure scan looks for."""
    out = [f"[pipeline] order {row['scenario_id']} family={row['family']} ordered={row['ordered']} "
           f"arrived={row['stream_boxes']} status={row['status']}"]
    result = row.get("result") or {}
    for e in result.get("events", []):
        kind, box = e["event"], e.get("box", "")
        t = f"t={e['t']:.1f}s"
        if kind == "PLACE":
            level = e.get("level")
            tag = "PASS" if level == "L0" else "WARN"
            out.append(f"[executor] {tag} place {box} {e.get('action')} post-check {level} "
                       f"dxy={e.get('dxy_mm')}mm cycle={e.get('cycle_s')}s {t}")
        elif kind == "INSPECTION":
            out.append(f"[PERCEPTION] WARN {box} -> inspection ({e.get('reason')}) {t}")
        elif kind == "L3_GRIP_FAIL":
            out.append(f"[executor] FAIL grip {box} (L3, retried) {t}")
        elif kind == "ARRIVE" and e.get("anomaly") not in (None, "OK"):
            out.append(f"[PERCEPTION] WARN {box} sku={e.get('sku')} anomaly={e.get('anomaly')} {t}")
        elif kind in ("CLOSE", "MISSING", "REPACK_ABORT", "NG", "BUFFER", "REPACK_MOVE"):
            out.append(f"[cycle] {kind} " + json.dumps({k: v for k, v in e.items() if k not in ("event",)},
                                                     ensure_ascii=False))
    for d in row["decisions"]["log"]:
        if d["elapsed_s"] > 7.0:
            out.append(f"[planner] WARN decision {d['elapsed_s']:.2f}s over the 7 s budget "
                       f"action={d['action']} reason={d['reason']}")
    for name, p in row.get("perception", {}).items():
        if p.get("damage_false_alarm"):
            out.append(f"[PERCEPTION] WARN {name}: {p['damage_false_alarm']} intact boxes called damaged")
        if p.get("label_resolved_wrong"):
            out.append(f"[PERCEPTION] WARN {name}: {p['label_resolved_wrong']} SKUs inferred wrongly")
    if row["status"] == "runtime_stopped":
        last = row["decisions"]["log"][-1] if row["decisions"]["log"] else {}
        out.append(f"[planner] ERROR order stopped: {last.get('reason')} after {last.get('elapsed_s')}s; "
                   f"unprocessed={row.get('unprocessed')}")
    elif row["status"] in ("error", "timeout"):
        out.append(f"[pipeline] ERROR {row['status']}: {row.get('error')}")
        out += (row.get("traceback") or "").splitlines()
    else:
        out.append(f"[pipeline] PASS order complete placed={result.get('placed')} pallets={result.get('pallets')}")
    return out


def _box_model(name, size, centre, yaw, colour):
    sx, sy, sz = size
    x, y, z = centre
    return (f'<model name="{escape(name)}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>'
            f'<link name="link"><visual name="visual"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>'
            f'</geometry><material><diffuse>{colour}</diffuse></material></visual></link></model>')


def _colour(box_id):
    h = sum(map(ord, box_id)) * 0.618 % 1.0
    r, g, b = (0.45 + 0.4 * abs(math.sin(math.tau * (h + k / 3))) for k in range(3))
    return f"{r:.2f} {g:.2f} {b:.2f} 1"


def world_sdf(world, pallets):
    """Pallet decks + the final layout of every pallet (pose = AABB min corner, z=0 on the deck)."""
    models, x0 = [], 0.0
    for p in pallets:
        px, py, pz = p["pallet_size"]
        models.append(_box_model(p["pallet_id"], (px, py, DECK_M), (x0 + px / 2, py / 2, DECK_M / 2), 0.0,
                                 "0.55 0.40 0.25 1"))
        for b in p.get("layout", []):
            sx, sy, sz = b["size"]
            x, y, z, yaw = b["pose"]
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            dx, dy = c * sx + s * sy, s * sx + c * sy
            models.append(_box_model(b["box_id"], (sx, sy, sz),
                                     (x0 + x + dx / 2, y + dy / 2, DECK_M + z + sz / 2), yaw, _colour(b["box_id"])))
        x0 += px + PALLET_GAP_M
    return (f'<?xml version="1.0"?>\n<sdf version="1.8"><world name="{world}">'
            + "".join(models) + "</world></sdf>\n")


def export_order(row, run_id, stream_sizes, start_epoch, repo=REPO, register=True):
    """Write the dashboard files for one order; returns the run folder."""
    folder = Path(repo) / "logs/pipeline" / run_id
    folder.mkdir(parents=True, exist_ok=True)
    world = "pac_lyl_virtual"
    result = row.get("result") or {}
    pallets = result.get("pallet_list", [])
    lines = _log_lines(row)
    (folder / "pipeline_cycle.log").write_text("\n".join(lines) + "\n", encoding="utf-8")
    # the dashboard scans only the tail of each log: keep the alerts in their own file
    alerts = [x for x in lines[:-1] if re.search(r"FAIL|ERROR|WARN|Traceback", x)] + lines[-1:]
    (folder / "executor_alerts.log").write_text("\n".join(alerts[-90:]) + "\n", encoding="utf-8")
    (folder / "boxes.json").write_text(json.dumps({k: {"size_m": list(v)} for k, v in stream_sizes.items()},
                                                  ensure_ascii=False), encoding="utf-8")
    (folder / "world.sdf").write_text(world_sdf(world, pallets), encoding="utf-8")
    with (folder / "events.jsonl").open("w", encoding="utf-8") as f:
        for e in result.get("events", []):
            mapped = EVENT_MAP.get(e["event"])
            if not mapped or not e.get("box"):
                continue
            name, ok = mapped
            f.write(json.dumps({"run_id": run_id, "world": world, "box_id": e["box"], "event": name,
                                "success": ok, "timestamp_epoch_s": round(start_epoch + e["t"], 3),
                                "reason": str(e.get("reason") or e.get("anomaly") or e.get("level") or "")[:200]},
                               ensure_ascii=False, separators=(",", ":")) + "\n")
    if register:
        rel = folder.relative_to(Path(repo).resolve())
        width = sum(p["pallet_size"][0] for p in pallets) + PALLET_GAP_M * max(0, len(pallets) - 1)
        height = max((p["pallet_size"][2] for p in pallets), default=1.5) + DECK_M
        sys.path.insert(0, str(DASHBOARD))
        from pac_run_contract_writer import register_active_run
        register_active_run(Path(repo), run_id=run_id, world=world, world_sdf=str(rel / "world.sdf"),
                            log_dir=str(rel), box_catalog=str(rel / "boxes.json"),
                            events_file=str(rel / "events.jsonl"), topics={},
                            zones={"pallet": {"x": [0.0, max(width, 1.1)], "y": [0.0, 1.1],
                                              "z": [0.0, round(height + 0.1, 3)]}})
    return folder
