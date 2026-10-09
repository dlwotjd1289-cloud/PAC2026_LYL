"""PAC2026 LYL end-to-end virtual pipeline (see ``pipeline/run.py``)."""
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def bootstrap():
    """Put every package of the repo on ``sys.path`` (no install needed)."""
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(key, "1")
    paths = sorted(p for p in (REPO / "ros2_ws/src").glob("pac_*") if (p / p.name).is_dir())
    paths += [REPO / "tools/virtual_data", REPO / "tools/algorithm_integration"]
    for path in reversed(paths):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
