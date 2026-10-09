from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import statistics
import subprocess
from typing import Any

from pac_common import (
    BoxState,
    BoxStatus,
    InventoryState,
    PalletState,
    Pose3D,
    Size3D,
    SystemState,
)

from .config import SCENARIO_FAMILIES, SkuSpec, load_sku_catalog, resolve_pallet, validate_config
from .serialization import to_primitive, write_json, write_jsonl

GENERATOR_VERSION = "1.4.0"
HALF_PI = 1.5707963267948966
MAX_DEDUP_ATTEMPTS = 50

WEIGHT_BANDS_KG = (
    (0.0, 3.0, "W_00_03"),
    (3.0, 5.0, "W_03_05"),
    (5.0, 7.0, "W_05_07"),
    (7.0, 10.0, "W_07_10"),
    (10.0, 15.0, "W_10_15"),
    (15.0, 20.0, "W_15_20"),
    (20.0, 25.0, "W_20_25"),
    (25.0, 30.0, "W_25_30"),
)


@dataclass(frozen=True)
class GeneratedBox:
    truth: BoxState
    observation: BoxState
    arrival_index: int
    weight_profile_component: str

    @property
    def volume_m3(self) -> float:
        size = self.truth.size
        return size.x * size.y * size.z

    @property
    def dimension_sum_m(self) -> float:
        size = self.truth.size
        return size.x + size.y + size.z


def _weighted_choice(
    rng: random.Random,
    items: list[Any],
    weights: list[float],
) -> Any:
    total = sum(weights)
    value = rng.random() * total
    running = 0.0
    for item, weight in zip(items, weights):
        running += weight
        if value <= running:
            return item
    return items[-1]


def _sample_profile(rng: random.Random, cfg: dict[str, Any]) -> str:
    dist = cfg["generation"]["weight_profile_distribution"]
    names = list(dist)
    return str(_weighted_choice(rng, names, [float(dist[name]) for name in names]))


def _stress_range(cfg: dict[str, Any], catalog: list[SkuSpec]) -> tuple[float, float]:
    raw = cfg["generation"].get("stress_weight_range_kg")
    if raw is not None:
        return float(raw[0]), float(raw[1])
    return (
        min(sku.weight_min_kg for sku in catalog),
        max(sku.weight_max_kg for sku in catalog),
    )


def _sample_atomic_weight(
    rng: random.Random,
    sku: SkuSpec,
    profile: str,
    stress_range: tuple[float, float],
) -> float:
    low = sku.weight_min_kg
    high = sku.weight_max_kg
    if profile == "normal":
        value = rng.uniform(low, high)
    elif profile == "light_biased":
        value = rng.triangular(low, high, low + 0.20 * (high - low))
    elif profile == "heavy_biased":
        value = rng.triangular(low, high, low + 0.80 * (high - low))
    elif profile == "stress":
        value = rng.uniform(stress_range[0], stress_range[1])
    else:
        raise ValueError(f"unsupported atomic profile: {profile}")
    return round(value, 6)


def _sample_weight(
    rng: random.Random,
    sku: SkuSpec,
    profile: str,
    cfg: dict[str, Any],
    stress_range: tuple[float, float],
) -> tuple[float, str]:
    if profile != "mixed":
        return _sample_atomic_weight(rng, sku, profile, stress_range), profile

    dist = cfg["generation"]["mixed_component_distribution"]
    names = list(dist)
    component = str(
        _weighted_choice(rng, names, [float(dist[name]) for name in names])
    )
    return (
        _sample_atomic_weight(rng, sku, component, stress_range),
        component,
    )


def _scenario_seed(base_seed: int, index: int, attempt: int) -> int:
    """Independent per-scenario seed derived by hashing.

    The old linear rule (base + index * 1009) made two runs whose base seeds
    differ by a multiple of 1009 emit the same scenarios at shifted indices.
    """
    key = f"ahead-scenario:{base_seed}:{index}:{attempt}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(key).digest()[:8], "big")


def _inventory_key(boxes: list["GeneratedBox"]) -> tuple[str, ...]:
    """SKU multiset of a scenario; same key = same boxes in another order."""
    return tuple(sorted(box.truth.sku_id for box in boxes))


def _sku_by_volume(catalog: list[SkuSpec]) -> list[SkuSpec]:
    return sorted(catalog, key=lambda item: item.volume_m3)


def _select_skus(
    rng: random.Random,
    catalog: list[SkuSpec],
    family: str,
    count: int,
) -> list[SkuSpec]:
    weights = [sku.sampling_weight for sku in catalog]
    if family == "size_mixed":
        ordered = _sku_by_volume(catalog)
        third = max(1, len(ordered) // 3)
        groups = (
            ordered[:third],
            ordered[third:2 * third] or ordered,
            ordered[2 * third:] or ordered,
        )
        selected = [rng.choice(groups[index % 3]) for index in range(count)]
        rng.shuffle(selected)
        return selected

    if family == "repeated_sku":
        distinct = min(4, len(catalog))
        block_skus = rng.sample(catalog, distinct)
        # At least two blocks, and never the same SKU in back-to-back blocks;
        # otherwise short scenarios collapse into one SKU and repeat a lot.
        max_block = max(1, min(7, count // 2))
        min_block = min(3, max_block)
        selected = []
        previous = None
        while len(selected) < count:
            choices = [sku for sku in block_skus if sku is not previous]
            sku = rng.choice(choices or block_skus)
            block_size = rng.randint(min_block, max_block)
            selected.extend([sku] * block_size)
            previous = sku
        return selected[:count]

    return [
        _weighted_choice(rng, catalog, weights)
        for _ in range(count)
    ]


def _make_box(
    scenario_id: str,
    index: int,
    sku: SkuSpec,
    weight_kg: float,
    component: str,
) -> GeneratedBox:
    box_id = f"{scenario_id}-B{index + 1:03d}"
    stamp_sec = float(index)
    pose = Pose3D(
        frame_id="conveyor",
        x=0.0,
        y=0.0,
        z=sku.size.z / 2.0,
    )
    truth = BoxState(
        box_id=box_id,
        sku_id=sku.sku_id,
        size=sku.size,
        weight_kg=weight_kg,
        pose=pose,
        allowed_yaws_rad=sku.allowed_yaws_rad,
        status=BoxStatus.READY_FOR_PICK,
        confidence=1.0,
        stamp_sec=stamp_sec,
        source="simulation_ground_truth",
    )
    observation = BoxState(
        box_id=box_id,
        sku_id=sku.sku_id,
        size=sku.size,
        weight_kg=weight_kg,
        pose=pose,
        allowed_yaws_rad=sku.allowed_yaws_rad,
        status=BoxStatus.DETECTED,
        confidence=1.0,
        stamp_sec=stamp_sec,
        source="simulation_identity_observation",
    )
    return GeneratedBox(
        truth=truth,
        observation=observation,
        arrival_index=index,
        weight_profile_component=component,
    )


def _generate_boxes(
    scenario_id: str,
    family: str,
    count: int,
    seed: int,
    catalog: list[SkuSpec],
    cfg: dict[str, Any],
) -> list[GeneratedBox]:
    rng = random.Random(seed)
    selected = _select_skus(rng, catalog, family, count)
    stress_range = _stress_range(cfg, catalog)

    draws = []
    for index, sku in enumerate(selected):
        if family == "weight_mixed":
            profile = "light_biased" if index % 2 == 0 else "heavy_biased"
        elif family == "late_heavy":
            profile = "mixed"
        else:
            profile = _sample_profile(rng, cfg)
        weight, component = _sample_weight(
            rng,
            sku,
            profile,
            cfg,
            stress_range,
        )
        draws.append((sku, weight, component))

    # Stress families: larger / heavier boxes arrive at the end of the stream.
    if family == "late_large":
        draws.sort(key=lambda item: item[0].volume_m3)
    elif family == "late_heavy":
        draws.sort(key=lambda item: item[1])

    return [
        _make_box(scenario_id, index, sku, weight, component)
        for index, (sku, weight, component) in enumerate(draws)
    ]


def _entropy(values: list[str]) -> float:
    counts = Counter(values)
    if len(counts) <= 1:
        return 0.0
    total = sum(counts.values())
    probabilities = [count / total for count in counts.values()]
    entropy = -sum(prob * math.log(prob) for prob in probabilities)
    return entropy / math.log(len(counts))


def _pearson(xs: list[float], ys: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    dx = [value - mean_x for value in xs]
    dy = [value - mean_y for value in ys]
    denominator = math.sqrt(
        sum(value * value for value in dx)
        * sum(value * value for value in dy)
    )
    if denominator <= 1e-15:
        return 0.0
    return sum(x * y for x, y in zip(dx, dy)) / denominator


def _weight_band(weight_kg: float) -> str:
    for index, (low, high, label) in enumerate(WEIGHT_BANDS_KG):
        if index == 0 and low <= weight_kg <= high:
            return label
        if index > 0 and low < weight_kg <= high:
            return label
    return "OUT_OF_DOMAIN"


def _git_commit(repo_root: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except (subprocess.SubprocessError, FileNotFoundError):
        return "UNKNOWN"


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _scenario_metadata(
    scenario_id: str,
    family: str,
    seed: int,
    boxes: list[GeneratedBox],
) -> dict[str, Any]:
    volumes = [box.volume_m3 for box in boxes]
    weights = [box.truth.weight_kg for box in boxes]
    indices = list(range(len(boxes)))
    sku_ids = [box.truth.sku_id for box in boxes]
    return {
        "scenario_id": scenario_id,
        "scenario_family": family,
        "seed": seed,
        "num_boxes": len(boxes),
        "num_skus": len(set(sku_ids)),
        "sku_diversity": round(_entropy(sku_ids), 6),
        "weight_band_diversity": round(
            _entropy([_weight_band(value) for value in weights]),
            6,
        ),
        "total_volume_m3": round(sum(volumes), 9),
        "total_weight_kg": round(sum(weights), 6),
        "arrival_volume_corr": round(_pearson(indices, volumes), 6),
        "arrival_weight_corr": round(_pearson(indices, weights), 6),
    }


def _initial_state(
    pallet_cfg: dict[str, Any],
    boxes: list[GeneratedBox],
) -> SystemState:
    pallet = PalletState(
        pallet_id=str(pallet_cfg["pallet_id"]),
        size=Size3D(
            x=float(pallet_cfg["size_x_m"]),
            y=float(pallet_cfg["size_y_m"]),
            # Cargo height above the deck, not the wood thickness.
            z=float(pallet_cfg["max_stack_height_m"]),
        ),
        boxes=(),
    )
    remaining = Counter(box.truth.sku_id for box in boxes)
    inventory = InventoryState(
        tracked_boxes={},
        remaining_by_sku=dict(sorted(remaining.items())),
    )
    return SystemState(
        state_version=0,
        stamp_sec=0.0,
        pallet=pallet,
        inventory=inventory,
    )


def _scenario_definition(
    cfg: dict[str, Any],
    scenario_id: str,
    family: str,
    seed: int,
    boxes: list[GeneratedBox],
) -> dict[str, Any]:
    pallet_cfg = resolve_pallet(cfg)
    return {
        "schema_version": int(cfg["schema_version"]),
        "generator_version": GENERATOR_VERSION,
        "scenario_id": scenario_id,
        "scenario_family": family,
        "state_kind": "SIMULATED",
        "seed": seed,
        "initial_system_state": _initial_state(pallet_cfg, boxes),
        "constraints": {
            # Explicit references so consumers never guess what 1.5 m means.
            "max_stack_height_m": float(pallet_cfg["max_stack_height_m"]),
            "height_reference": "above_deck",
            "deck_height_m": float(pallet_cfg["deck_height_m"]),
            "max_load_kg": float(pallet_cfg["max_load_kg"]),
        },
        "planner_contract_note": (
            "This file intentionally contains no exact future arrival order. "
            "The simulator owns ground-truth arrival events."
        ),
    }


def _ground_truth_definition(
    cfg: dict[str, Any],
    scenario_id: str,
    family: str,
    seed: int,
    boxes: list[GeneratedBox],
) -> dict[str, Any]:
    return {
        "schema_version": int(cfg["schema_version"]),
        "scenario_id": scenario_id,
        "scenario_family": family,
        "state_kind": "SIMULATED",
        "seed": seed,
        "arrival_events": [
            {
                "arrival_index": box.arrival_index,
                "weight_profile_component": box.weight_profile_component,
                "box_state": box.truth,
            }
            for box in boxes
        ],
    }


def _observation_rows(
    cfg: dict[str, Any],
    scenario_id: str,
    boxes: list[GeneratedBox],
) -> list[dict[str, Any]]:
    return [
        {
            "schema_version": int(cfg["schema_version"]),
            "scenario_id": scenario_id,
            "state_kind": "SIMULATED",
            "arrival_index": box.arrival_index,
            "observation": box.observation,
        }
        for box in boxes
    ]


def _split_ids(
    scenario_ids: list[str],
    cfg: dict[str, Any],
    seed: int,
) -> dict[str, list[str]]:
    ids = list(scenario_ids)
    random.Random(seed + 909).shuffle(ids)
    total = len(ids)
    train_end = int(round(total * float(cfg["split"]["train_ratio"])))
    val_count = int(round(total * float(cfg["split"]["val_ratio"])))
    val_end = min(total, train_end + val_count)
    return {
        "train": ids[:train_end],
        "val": ids[train_end:val_end],
        "test": ids[val_end:],
    }


def _family_plan(
    cfg: dict[str, Any],
    mode: str,
    sample_per_family: int,
) -> list[str]:
    if mode == "benchmark":
        plan = []
        counts = cfg["generation"]["benchmark"]["family_counts"]
        for family in SCENARIO_FAMILIES:
            plan.extend([family] * int(counts.get(family, 0)))
        return plan
    return [
        family
        for family in SCENARIO_FAMILIES
        for _ in range(sample_per_family)
    ]


def _coverage_report(
    cfg: dict[str, Any],
    all_boxes: list[GeneratedBox],
    metadata_rows: list[dict[str, Any]],
    duplicate_inventories: int,
) -> dict[str, Any]:
    domain = cfg["parcel_domain"]
    violations = 0
    weight_bands = Counter()
    sku_counts = Counter()
    for box in all_boxes:
        size = box.truth.size
        if (
            box.truth.weight_kg > float(domain["max_weight_kg"]) + 1e-12
            or size.x + size.y + size.z
            > float(domain["max_dimension_sum_m"]) + 1e-12
            or max(size.x, size.y, size.z)
            > float(domain["max_single_side_m"]) + 1e-12
        ):
            violations += 1
        weight_bands[_weight_band(box.truth.weight_kg)] += 1
        sku_counts[box.truth.sku_id] += 1

    family_counts = Counter(row["scenario_family"] for row in metadata_rows)
    return {
        "schema_version": int(cfg["schema_version"]),
        "generator_version": GENERATOR_VERSION,
        "scenario_count": len(metadata_rows),
        "box_count": len(all_boxes),
        "sku_count": len(sku_counts),
        "family_counts": dict(sorted(family_counts.items())),
        "sku_counts": dict(sorted(sku_counts.items())),
        "weight_band_counts": dict(sorted(weight_bands.items())),
        "parcel_domain_violations": violations,
        "duplicate_inventory_scenarios": duplicate_inventories,
        "units": {
            "length": "m",
            "mass": "kg",
            "time": "s",
            "angle": "rad",
        },
    }


def _write_report_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# AHEAD Dataset Coverage Report",
        "",
        f"- Scenarios: {report['scenario_count']}",
        f"- Boxes: {report['box_count']}",
        f"- SKU types used: {report['sku_count']}",
        f"- Parcel-domain violations: {report['parcel_domain_violations']}",
        "- Duplicate-inventory scenarios: "
        f"{report['duplicate_inventory_scenarios']}",
        "- Internal units: m, kg, s, rad",
        "",
        "## Scenario families",
    ]
    for key, value in report["family_counts"].items():
        lines.append(f"- {key}: {value}")
    lines.extend(["", "## Weight bands (analysis only)"])
    for key, value in report["weight_band_counts"].items():
        lines.append(f"- {key}: {value}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_dataset(
    cfg: dict[str, Any],
    output_dir: Path,
    mode: str = "sample",
    sample_per_family: int = 2,
    seed_override: int | None = None,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    validate_config(cfg)
    catalog = load_sku_catalog(cfg)
    output_dir.mkdir(parents=True, exist_ok=True)

    benchmark_cfg = cfg["generation"]["benchmark"]
    base_seed = (
        int(seed_override)
        if seed_override is not None
        else int(benchmark_cfg["seed"])
    )
    boxes_per_scenario = int(benchmark_cfg["boxes_per_scenario"])
    plan = _family_plan(cfg, mode, sample_per_family)
    scenario_ids = [f"S{index + 1:04d}" for index in range(len(plan))]

    all_boxes = []
    metadata_rows = []
    generation_logs = []
    scenario_index_rows = []

    seen_inventories = set()
    duplicate_inventories = 0

    for index, (scenario_id, family) in enumerate(zip(scenario_ids, plan)):
        # Resample until the SKU multiset is new to this dataset, so no two
        # scenarios differ only in arrival order (train/test leakage). Small
        # box counts can exhaust the space; those are counted, not hidden.
        for attempt in range(MAX_DEDUP_ATTEMPTS):
            scenario_seed = _scenario_seed(base_seed, index, attempt)
            boxes = _generate_boxes(
                scenario_id,
                family,
                boxes_per_scenario,
                scenario_seed,
                catalog,
                cfg,
            )
            inventory = _inventory_key(boxes)
            if inventory not in seen_inventories:
                break
        else:
            duplicate_inventories += 1
        seen_inventories.add(inventory)
        all_boxes.extend(boxes)
        metadata = _scenario_metadata(
            scenario_id,
            family,
            scenario_seed,
            boxes,
        )
        metadata_rows.append(metadata)
        scenario_index_rows.append(
            {
                "scenario_id": scenario_id,
                "scenario_family": family,
                "seed": scenario_seed,
            }
        )

        write_json(
            output_dir / "test_data" / f"{scenario_id}.json",
            _scenario_definition(
                cfg,
                scenario_id,
                family,
                scenario_seed,
                boxes,
            ),
        )
        write_json(
            output_dir / "ground_truth" / f"{scenario_id}.json",
            _ground_truth_definition(
                cfg,
                scenario_id,
                family,
                scenario_seed,
                boxes,
            ),
        )
        write_jsonl(
            output_dir / "simulation_observations" / f"{scenario_id}.jsonl",
            _observation_rows(cfg, scenario_id, boxes),
        )
        generation_logs.append(
            {
                "schema_version": int(cfg["schema_version"]),
                "run_id": "DATASET_GENERATION",
                "scenario_id": scenario_id,
                "git_commit": "PENDING",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "state_version": 0,
                "seed": scenario_seed,
                "event": "scenario_generated",
            }
        )

    split_payload = _split_ids(scenario_ids, cfg, base_seed)
    write_json(output_dir / "splits.json", split_payload)
    _write_csv(output_dir / "analysis" / "scenario_metadata.csv", metadata_rows)
    _write_csv(output_dir / "analysis" / "scenario_index.csv", scenario_index_rows)

    catalog_rows = []
    for sku in catalog:
        catalog_rows.append(
            {
                "sku_id": sku.sku_id,
                "size_x_m": sku.size.x,
                "size_y_m": sku.size.y,
                "size_z_m": sku.size.z,
                "weight_min_kg": sku.weight_min_kg,
                "weight_max_kg": sku.weight_max_kg,
                "allowed_yaws_rad": json.dumps(list(sku.allowed_yaws_rad)),
                "source_group": sku.source_group,
            }
        )
    _write_csv(output_dir / "analysis" / "catalog.csv", catalog_rows)

    report = _coverage_report(cfg, all_boxes, metadata_rows, duplicate_inventories)
    write_json(output_dir / "analysis" / "coverage_report.json", report)
    _write_report_markdown(
        output_dir / "analysis" / "coverage_report.md",
        report,
    )

    repo_root = repo_root or Path.cwd()
    git_commit = _git_commit(repo_root)
    for row in generation_logs:
        row["git_commit"] = git_commit
    write_jsonl(output_dir / "logs" / "dataset_generation.jsonl", generation_logs)

    manifest = {
        "schema_version": int(cfg["schema_version"]),
        "generator_version": GENERATOR_VERSION,
        "dataset_name": str(cfg["dataset"]["name"]),
        "mode": mode,
        "run_id": f"{cfg['dataset']['name']}-{mode}-{base_seed}",
        "seed": base_seed,
        "git_commit": git_commit,
        "scenario_count": len(scenario_ids),
        "box_count": len(all_boxes),
        "units": {
            "length": "m",
            "mass": "kg",
            "time": "s",
            "angle": "rad",
        },
        "contract": {
            "runtime_model": "pac_common dataclass",
            "storage_config": "YAML",
            "storage_test_data": "JSON/JSONL",
            "planner_future_order_exposed": False,
            "robot_feasibility_in_generator": False,
        },
    }
    write_json(output_dir / "manifest.json", manifest)

    hash_targets = []
    for folder in ("test_data", "ground_truth", "simulation_observations"):
        hash_targets.extend(sorted((output_dir / folder).glob("*")))
    hashes = {
        str(path.relative_to(output_dir)): _hash_file(path)
        for path in hash_targets
        if path.is_file()
    }
    write_json(output_dir / "file_hashes.json", hashes)
    return manifest
