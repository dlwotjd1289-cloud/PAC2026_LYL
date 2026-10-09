"""Pipeline -> failure-analysis dashboard contract (files only, no server, no ROS)."""
import json
import sys

from pipeline.dashboard_export import DASHBOARD, export_order, run_id_for, world_sdf

sys.path.insert(0, str(DASHBOARD))
from pac_diagnostic_console import LogEvidence, failure_stage  # noqa: E402
from pac_integration import load_binding, load_catalog, load_events  # noqa: E402
from pac_scene3d import scene_for_repo  # noqa: E402

LAYOUT = [{"box_id": "S9-B001", "size": [0.4, 0.3, 0.2], "mass_kg": 5.0, "pose": [0.0, 0.0, 0.0, 0.0]},
          {"box_id": "S9-B002", "size": [0.4, 0.3, 0.2], "mass_kg": 5.0, "pose": [0.4, 0.0, 0.0, 1.5707963]}]


def stopped_row():
    events = [{"t": 0.0, "v": 1, "event": "ARRIVE", "box": "S9-B001", "sku": "K", "anomaly": "OK"},
              {"t": 8.0, "v": 2, "event": "PLACE", "box": "S9-B001", "action": "PLACE_CURRENT", "level": "L0",
               "dxy_mm": 1.0, "cycle_s": 8.0},
              {"t": 8.0, "v": 3, "event": "INSPECTION", "box": "S9-B003", "reason": "DAMAGED"},
              {"t": 9.0, "v": 4, "event": "L3_GRIP_FAIL", "box": "S9-B002"}]
    return {"scenario_id": "S9", "family": "normal", "ordered": 4, "stream_boxes": 4, "status": "runtime_stopped",
            "unprocessed": 1, "perception": {},
            "decisions": {"log": [{"elapsed_s": 7.4, "action": "WAIT", "reason": "DECISION_BUDGET_EXCEEDED"}]},
            "result": {"events": events, "placed": 2, "pallets": 1,
                       "pallet_list": [{"pallet_id": "P001", "pallet_size": [1.1, 1.1, 1.5], "layout": LAYOUT}]}}


def test_stopped_order_is_bound_and_its_failure_found(tmp_path):
    run_id = run_id_for("S9", "2026-10-10T09:00:00+00:00")
    sizes = {"S9-B001": (0.4, 0.3, 0.2), "S9-B002": (0.4, 0.3, 0.2), "S9-B003": (0.3, 0.2, 0.1)}
    export_order(stopped_row(), run_id, sizes, start_epoch=1_000_000.0, repo=tmp_path)

    binding = load_binding(tmp_path)
    assert binding["run_id"] == run_id and binding["world"] == "pac_lyl_virtual"
    assert set(load_catalog(binding, tmp_path)) == set(sizes)
    events = load_events(binding)
    assert events["S9-B003"]["event"] == "ng_diverted" and events["S9-B003"]["reason"] == "DAMAGED"
    assert events["S9-B002"] == {**events["S9-B002"], "event": "failed", "success": False}

    context = LogEvidence(tmp_path).snapshot()
    assert context["observation"] == "failure_logged"
    assert "DECISION_BUDGET_EXCEEDED" in context["alert"]["text"]
    assert failure_stage(context)["stage"] == "planner"


def test_world_sdf_draws_pallet_and_final_layout(tmp_path):
    (tmp_path / "logs").mkdir()
    path = tmp_path / "logs/world.sdf"
    path.write_text(world_sdf("pac_lyl_virtual", stopped_row()["result"]["pallet_list"]))
    scene = scene_for_repo(tmp_path, path, "pac_lyl_virtual")
    assert scene["error"] == "" and len(scene["primitives"]) == 3
    turned = next(p for p in scene["primitives"] if p["model"] == "S9-B002")
    # AABB min corner (0.4, 0) of a 90-degree box 0.4 x 0.3 -> centre (0.55, 0.2) on the 0.15 m deck
    assert turned["position"] == [0.55, 0.2, 0.25]


def test_events_keep_the_dashboard_line_format(tmp_path):
    run_id = run_id_for("S9", "2026-10-10T09:00:00+00:00")
    folder = export_order(stopped_row(), run_id, {}, start_epoch=0.0, repo=tmp_path, register=False)
    for line in (folder / "events.jsonl").read_text().splitlines():
        assert set(json.loads(line)) == {"run_id", "world", "box_id", "event", "success", "timestamp_epoch_s",
                                         "reason"}
    assert not (tmp_path / "logs/pac_dashboard_integration").exists()
