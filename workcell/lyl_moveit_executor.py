#!/usr/bin/env python3
"""Run the existing V4.6 executor with a bounded commissioning press depth.

The original executor and its motion, collision, and descent guards stay intact.
1 mm is an initial contact-depth setting, not a physically validated success.
"""
from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
import sys


def main() -> int:
    try:
        press_m = float(os.environ.get("PAC_PRESS_M", "0.001"))
        if not math.isfinite(press_m) or not 0.0 <= press_m <= 0.002:
            raise ValueError("expected a finite value between 0 and 0.002 m")
    except ValueError as exc:
        print(f"Invalid PAC_PRESS_M: {exc}; robot executor not started", file=sys.stderr)
        return 2

    default_root = Path(__file__).resolve().parents[1].parent / "pac2026_integrated"
    workcell = Path(os.environ.get("PAC_WORKCELL_REPO", str(default_root))).expanduser().resolve()
    source = workcell / "scripts" / "moveit_pick_place_v44.py"
    if not source.is_file():
        print(f"Original V4.6 executor missing: {source}", file=sys.stderr)
        return 2
    # The original script imports its sibling vertical_descent_v46 module.
    sys.path.insert(0, str(source.parent))
    spec = importlib.util.spec_from_file_location("_lyl_original_moveit_v46", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.PRESS_M = press_m
    print(f"[LYL executor] press depth {press_m * 1000:.3f} mm; "
          "original motion and safety checks retained (commissioning setting)", flush=True)
    return module.main()


if __name__ == "__main__":
    raise SystemExit(main())
