from pathlib import Path
import json

from ahead_dataset_generator.config import load_yaml
from ahead_dataset_generator.generator import generate_dataset


def test_same_seed_reproduces_core_files(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    cfg = load_yaml(root / "config" / "default.yaml")
    first = tmp_path / "first"
    second = tmp_path / "second"
    generate_dataset(cfg, first, sample_per_family=1, repo_root=root)
    generate_dataset(cfg, second, sample_per_family=1, repo_root=root)

    first_hashes = json.loads(
        (first / "file_hashes.json").read_text(encoding="utf-8")
    )
    second_hashes = json.loads(
        (second / "file_hashes.json").read_text(encoding="utf-8")
    )
    assert first_hashes == second_hashes
