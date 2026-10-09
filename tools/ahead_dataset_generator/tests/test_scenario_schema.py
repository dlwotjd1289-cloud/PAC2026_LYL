from pathlib import Path
import json

from ahead_dataset_generator.config import load_yaml
from ahead_dataset_generator.generator import generate_dataset
from ahead_dataset_generator.validation import validate_dataset


def test_planner_safe_scenario_contains_no_future_order(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    cfg = load_yaml(root / "config" / "default.yaml")
    output = tmp_path / "dataset"
    generate_dataset(cfg, output, sample_per_family=1, repo_root=root)

    for path in (output / "test_data").glob("S*.json"):
        text = path.read_text(encoding="utf-8")
        payload = json.loads(text)
        assert "arrival_events" not in payload
        assert "true_" not in text

    assert validate_dataset(output) == []
