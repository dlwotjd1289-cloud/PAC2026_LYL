#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
# Team pac_common (monorepo ros2_ws/src) when not already on PYTHONPATH.
sys.path.append(str(PROJECT_ROOT.parents[1] / "ros2_ws" / "src" / "pac_common"))

from ahead_dataset_generator.validation import validate_dataset


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    failures = validate_dataset(args.dataset)
    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}")
        return 1
    print("[PASS] dataset contract validation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
