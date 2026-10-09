"""Order list (known in advance: SKU sizes, weight ranges, yaws, counts) ->
``CellSpec`` for ``RuntimeCore``. JSON layout::

    {"pallet": {"size_m": [1.2, 1.0, 1.35], "max_load_kg": 1000, "id_prefix": "P001"},
     "skus": {"K01": {"size_m": [0.22, 0.19, 0.09], "weight_kg": [0.2, 3.0],
                      "yaws_rad": [0.0, 1.5708], "count": 12}, ...}}

``size_m[2]`` of the pallet is the stacking height (deck excluded), as in
pac_common. Top-load capacities follow the McKee model of ``pac_candidates``.
"""

import json
from dataclasses import replace
from pathlib import Path

from pac_common import Size3D, SkuSpec
from pac_candidates.loads import mckee_capacity_n

from .loop import CellSpec


def cell_from_order(order, cand_config, nominal_weight="max"):
    lm = cand_config.constraints.load_model
    catalog, ranges, expected = {}, {}, {}
    for sku, s in sorted(order["skus"].items()):
        size = Size3D(*s["size_m"])
        lo, hi = s["weight_kg"]
        cap = mckee_capacity_n(size.x, size.y, lm.ect_n_per_m, lm.board_thickness_m, lm.safety_factor)
        weight = hi if nominal_weight == "max" else 0.5 * (lo + hi)
        catalog[sku] = SkuSpec(sku, size, float(weight), tuple(s.get("yaws_rad", (0.0, 1.5707963267948966))),
                               round(cap, 3))
        ranges[sku] = (float(lo), float(hi))
        expected[sku] = int(s["count"])
    p = order["pallet"]
    return CellSpec(stream=(), expected_by_sku=expected, pallet_size=Size3D(*p["size_m"]), catalog=catalog,
                    weight_ranges=ranges, pallet_max_weight_kg=float(p.get("max_load_kg", 1000.0)),
                    pallet_prefix=p.get("id_prefix", "PALLET"))


def configure_cell(cell, runtime_config):
    """Apply the selected existing capacity model without changing cell geometry."""
    if runtime_config is None or runtime_config.capacity.model == "team_mckee":
        return cell
    from .capacity import apply_site_capacity
    return replace(cell, catalog=apply_site_capacity(
        cell.catalog, cell.weight_ranges, runtime_config.capacity.site))


def load_order(path, cand_config, runtime_config=None):
    cell = cell_from_order(json.loads(Path(path).read_text(encoding="utf-8")), cand_config)
    return configure_cell(cell, runtime_config)
