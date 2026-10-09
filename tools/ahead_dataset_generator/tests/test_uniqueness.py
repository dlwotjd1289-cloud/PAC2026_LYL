from pathlib import Path
import json
import random

from ahead_dataset_generator.config import load_sku_catalog, load_yaml
from ahead_dataset_generator.generator import _select_skus, generate_dataset

ROOT = Path(__file__).resolve().parents[1]


def _inventories(dataset: Path) -> list[tuple[str, ...]]:
    result = []
    for path in sorted((dataset / "ground_truth").glob("S*.json")):
        events = json.loads(path.read_text(encoding="utf-8"))["arrival_events"]
        result.append(tuple(event["box_state"]["sku_id"] for event in events))
    return result


def test_no_duplicate_inventory_with_few_boxes(tmp_path: Path) -> None:
    cfg = load_yaml(ROOT / "config" / "default.yaml")
    cfg["generation"]["benchmark"]["boxes_per_scenario"] = 5
    generate_dataset(cfg, tmp_path, mode="sample", sample_per_family=20,
                     repo_root=ROOT)

    multisets = [tuple(sorted(seq)) for seq in _inventories(tmp_path)]
    assert len(set(multisets)) == len(multisets)
    report = json.loads(
        (tmp_path / "analysis" / "coverage_report.json").read_text(encoding="utf-8")
    )
    assert report["duplicate_inventory_scenarios"] == 0


def test_shifted_base_seed_does_not_replay_scenarios(tmp_path: Path) -> None:
    # The old base + index * 1009 rule replayed scenarios for this shift.
    cfg = load_yaml(ROOT / "config" / "default.yaml")
    base = int(cfg["generation"]["benchmark"]["seed"])
    generate_dataset(cfg, tmp_path / "a", sample_per_family=2,
                     seed_override=base, repo_root=ROOT)
    generate_dataset(cfg, tmp_path / "b", sample_per_family=2,
                     seed_override=base + 1009, repo_root=ROOT)

    assert not set(_inventories(tmp_path / "a")) & set(_inventories(tmp_path / "b"))


def test_repeated_sku_uses_at_least_two_blocks() -> None:
    catalog = load_sku_catalog(load_yaml(ROOT / "config" / "default.yaml"))
    for count in (4, 5, 8, 24):
        for seed in range(200):
            selected = _select_skus(random.Random(seed), catalog,
                                    "repeated_sku", count)
            assert len(selected) == count
            assert len({sku.sku_id for sku in selected}) >= 2


def test_zero_boxes_per_scenario_is_rejected(tmp_path: Path) -> None:
    import pytest

    cfg = load_yaml(ROOT / "config" / "default.yaml")
    cfg["generation"]["benchmark"]["boxes_per_scenario"] = 0
    with pytest.raises(ValueError, match="boxes_per_scenario"):
        generate_dataset(cfg, tmp_path, repo_root=ROOT)
