from pac_common import Size3D, SkuSpec

from pac_perception.sku_resolver import resolve_sku

from perception_testkit import CATALOG, WEIGHT_RANGES, YAWS


def test_unique_match_in_either_orientation():
    r = resolve_sku(Size3D(0.25, 0.34, 0.21), 3.2, CATALOG, WEIGHT_RANGES)
    assert (r.sku_id, r.reason) == ("K08", "unique")
    assert r.confidence < 1.0


def test_close_sizes_are_told_apart():
    # K03 and K04 differ by 2 cm, outside the 1.2 cm tolerance
    assert resolve_sku(Size3D(0.27, 0.18, 0.15), 1.39, CATALOG, WEIGHT_RANGES).sku_id == "K03"
    assert resolve_sku(Size3D(0.27, 0.20, 0.13), 0.86, CATALOG, WEIGHT_RANGES).sku_id == "K04"


def test_weight_outside_range_is_no_match():
    r = resolve_sku(Size3D(0.34, 0.25, 0.21), 8.0, CATALOG, WEIGHT_RANGES)
    assert (r.sku_id, r.reason) == (None, "no_match")


def test_same_size_skus_are_split_by_weight_or_left_ambiguous():
    catalog = dict(CATALOG, K99=SkuSpec("K99", Size3D(0.34, 0.25, 0.21), 12.0, YAWS, 400.0))
    ranges = dict(WEIGHT_RANGES, K99=(9.0, 12.0))
    assert resolve_sku(Size3D(0.34, 0.25, 0.21), 10.0, catalog, ranges).sku_id == "K99"

    ranges["K99"] = (4.0, 12.0)
    r = resolve_sku(Size3D(0.34, 0.25, 0.21), 4.5, catalog, ranges)
    assert (r.sku_id, r.reason, r.candidates) == (None, "ambiguous", ("K08", "K99"))


def test_sold_out_sku_is_excluded():
    remaining = {"K03": 0, "K04": 1, "K08": 2, "K13": 2}
    r = resolve_sku(Size3D(0.27, 0.18, 0.15), 1.39, CATALOG, WEIGHT_RANGES, remaining)
    assert r.reason == "no_match"
