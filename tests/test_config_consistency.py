"""config/default.yaml `pallet:` is the single source; everything else must agree.

Change the value there; the generator and the planner both read it.
"""
from pathlib import Path

import pytest
from pac_common.config import load_common_config

ROOT = Path(__file__).resolve().parents[1]
PALLET = load_common_config(ROOT / "config" / "default.yaml").pallet


def test_common_values_are_the_team_decision_shape():
    assert PALLET.cargo_size.z == PALLET.max_stack_height_m   # PalletState.size.z = above deck
    assert PALLET.max_total_height_m == pytest.approx(PALLET.deck_height_m + PALLET.max_stack_height_m)


def test_generator_reads_common_pallet():
    from ahead_dataset_generator.config import load_yaml, resolve_pallet

    cfg = load_yaml(ROOT / "tools/ahead_dataset_generator/config/default.yaml")
    got = resolve_pallet(cfg)
    assert (got["size_x_m"], got["size_y_m"], got["deck_height_m"], got["max_stack_height_m"],
            got["max_load_kg"]) == pytest.approx((PALLET.size_x_m, PALLET.size_y_m, PALLET.deck_height_m,
                                                  PALLET.max_stack_height_m, PALLET.max_load_kg))

