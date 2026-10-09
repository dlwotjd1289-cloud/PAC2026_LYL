#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
# Team pac_common (monorepo ros2_ws/src) when not already on PYTHONPATH.
sys.path.append(str(PROJECT_ROOT.parents[1] / "ros2_ws" / "src" / "pac_common"))

from ahead_dataset_generator.config import load_yaml
from ahead_dataset_generator.generator import generate_dataset


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "default.yaml",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--mode",
        choices=("sample", "benchmark"),
        default="sample",
    )
    parser.add_argument("--sample-per-family", type=int, default=2)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    manifest = generate_dataset(
        cfg=cfg,
        output_dir=args.output,
        mode=args.mode,
        sample_per_family=args.sample_per_family,
        seed_override=args.seed,
        repo_root=PROJECT_ROOT,
    )
    print(
        f"Generated {manifest['scenario_count']} scenarios / "
        f"{manifest['box_count']} boxes -> {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
