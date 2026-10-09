"""Repo-wide pytest setup: put every team package on sys.path (no install needed).

One copy of each package exists in the monorepo, so the order does not matter
for correctness; it only mirrors the ROS 2 workspace layout.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PATHS = [
    *(p for p in sorted((ROOT / "ros2_ws" / "src").glob("pac_*")) if (p / p.name).is_dir()),
    ROOT / "tools" / "virtual_data",
    ROOT / "tools" / "ahead_dataset_generator" / "src",
    ROOT / "tools" / "algorithm_integration",
    ROOT,  # pipeline
]
for path in reversed(PATHS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
