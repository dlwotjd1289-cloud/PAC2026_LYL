import json
from pathlib import Path

from pac_common import BoxState, SystemState

from .serialization import box_from_dict, system_state_from_dict


def validate_dataset(dataset_dir: Path) -> list[str]:
    failures = []
    required = (
        "manifest.json",
        "splits.json",
        "analysis/coverage_report.json",
        "analysis/scenario_metadata.csv",
        "logs/dataset_generation.jsonl",
    )
    for relative in required:
        if not (dataset_dir / relative).exists():
            failures.append(f"missing {relative}")

    if failures:
        return failures

    manifest = json.loads(
        (dataset_dir / "manifest.json").read_text(encoding="utf-8")
    )
    scenario_count = int(manifest["scenario_count"])
    test_files = sorted((dataset_dir / "test_data").glob("S*.json"))
    gt_files = sorted((dataset_dir / "ground_truth").glob("S*.json"))
    obs_files = sorted(
        (dataset_dir / "simulation_observations").glob("S*.jsonl")
    )
    if len(test_files) != scenario_count:
        failures.append("test_data scenario count mismatch")
    if len(gt_files) != scenario_count:
        failures.append("ground_truth scenario count mismatch")
    if len(obs_files) != scenario_count:
        failures.append("simulation_observations scenario count mismatch")

    for path in test_files:
        payload = json.loads(path.read_text(encoding="utf-8"))
        raw_text = path.read_text(encoding="utf-8")
        if "arrival_events" in payload or "true_" in raw_text:
            failures.append(f"future/ground-truth leakage in {path.name}")
        try:
            state = system_state_from_dict(payload["initial_system_state"])
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"invalid SystemState in {path.name}: {exc}")
            continue
        if not isinstance(state, SystemState):
            failures.append(f"SystemState adapter failure in {path.name}")
        if state.pallet.size.x <= 0.0 or state.pallet.size.y <= 0.0:
            failures.append(f"invalid pallet SI dimensions in {path.name}")

    for path in gt_files:
        payload = json.loads(path.read_text(encoding="utf-8"))
        for event in payload.get("arrival_events", []):
            try:
                box = box_from_dict(event["box_state"])
            except (KeyError, TypeError, ValueError) as exc:
                failures.append(f"invalid BoxState in {path.name}: {exc}")
                continue
            if not isinstance(box, BoxState):
                failures.append(f"BoxState adapter failure in {path.name}")
            if not box.pose.frame_id:
                failures.append(f"empty frame_id in {path.name}")
            if min(box.size.x, box.size.y, box.size.z) <= 0.0:
                failures.append(f"non-positive SI size in {path.name}")
            if box.weight_kg < 0.0:
                failures.append(f"negative mass in {path.name}")
            if not 0.0 <= box.confidence <= 1.0:
                failures.append(f"invalid confidence in {path.name}")

    splits = json.loads((dataset_dir / "splits.json").read_text(encoding="utf-8"))
    split_sets = {name: set(values) for name, values in splits.items()}
    if split_sets["train"] & split_sets["val"]:
        failures.append("train/val overlap")
    if split_sets["train"] & split_sets["test"]:
        failures.append("train/test overlap")
    if split_sets["val"] & split_sets["test"]:
        failures.append("val/test overlap")
    all_split_ids = set().union(*split_sets.values())
    scenario_ids = {path.stem for path in test_files}
    if all_split_ids != scenario_ids:
        failures.append("split IDs do not exactly cover scenario IDs")

    report = json.loads(
        (dataset_dir / "analysis" / "coverage_report.json").read_text(
            encoding="utf-8"
        )
    )
    if int(report["parcel_domain_violations"]) != 0:
        failures.append("parcel-domain violations detected")
    return failures
