import itertools
import math
import random

import numpy as np
import pytest

from th_helpers import HALF_PI  # noqa: F401  (bootstraps sys.path)
from pac_common import Size3D
from pac_candidates.geometry import (
    Rect,
    clip_polygon_to_rect,
    convex_hull,
    inside_margin,
    polygon_area,
    rotated_dims,
)
from pac_candidates.loads import area_shares, lever_shares, mckee_capacity_n
from pac_candidates.pallet_model import maximal_rectangles


def test_rotated_dims_axis_aligned_only():
    size = Size3D(0.4, 0.3, 0.2)
    assert rotated_dims(size, 0.0) == (0.4, 0.3, 0.2)
    assert rotated_dims(size, HALF_PI) == (0.3, 0.4, 0.2)
    assert rotated_dims(size, math.pi) == (0.4, 0.3, 0.2)
    assert rotated_dims(size, -HALF_PI) == (0.3, 0.4, 0.2)
    with pytest.raises(ValueError):
        rotated_dims(size, 0.3)


def test_convex_hull_and_area():
    pts = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.2, 0.8)]
    hull = convex_hull(pts)
    assert len(hull) == 4
    assert polygon_area(hull) == pytest.approx(1.0)  # CCW -> positive


def test_clip_polygon_to_rect():
    square = convex_hull([(0, 0), (1, 0), (1, 1), (0, 1)])
    clipped = clip_polygon_to_rect(square, Rect(0.5, -1, 2, 0.25))
    assert polygon_area(clipped) == pytest.approx(0.5 * 0.25)
    assert clip_polygon_to_rect(square, Rect(2, 2, 3, 3)) in ((), tuple())


def test_inside_margin_with_uncertainty():
    square = convex_hull([(0, 0), (1, 0), (1, 1), (0, 1)])
    assert inside_margin(square, (0.5, 0.5)) == pytest.approx(0.5)
    assert inside_margin(square, (0.5, 0.5), 0.1, 0.2) == pytest.approx(0.3)
    assert inside_margin(square, (0.95, 0.5), 0.1, 0.0) < 0
    assert inside_margin(((0, 0), (1, 0)), (0.5, 0.0)) == -math.inf


def _brute_maximal(grid):
    rows, cols = grid.shape
    rects = set()
    for r0, r1 in itertools.combinations_with_replacement(range(rows), 2):
        for c0, c1 in itertools.combinations_with_replacement(range(cols), 2):
            if grid[r0 : r1 + 1, c0 : c1 + 1].all():
                rects.add((r0, r1, c0, c1))

    def grows(rect):
        r0, r1, c0, c1 = rect
        return any(
            g in rects
            for g in (
                (r0 - 1, r1, c0, c1),
                (r0, r1 + 1, c0, c1),
                (r0, r1, c0 - 1, c1),
                (r0, r1, c0, c1 + 1),
            )
        )

    return {r for r in rects if not grows(r)}


@pytest.mark.parametrize("seed", range(40))
def test_maximal_rectangles_match_brute_force(seed):
    rng = np.random.default_rng(seed)
    shape = (int(rng.integers(1, 7)), int(rng.integers(1, 7)))
    grid = rng.random(shape) < rng.uniform(0.3, 0.9)
    assert set(maximal_rectangles(grid)) == _brute_maximal(grid)


def test_area_and_lever_shares_conserve_and_balance():
    assert sum(area_shares([1, 2, 3])) == pytest.approx(1.0)
    rng = random.Random(3)
    for _ in range(50):
        n = rng.randint(2, 5)
        areas = [rng.uniform(0.01, 0.2) for _ in range(n)]
        cents = [(rng.uniform(0, 1), rng.uniform(0, 1)) for _ in range(n)]
        # point inside the centroid hull -> exact moment balance expected
        w = [rng.random() for _ in range(n)]
        s = sum(w)
        point = (
            sum(wi * c[0] for wi, c in zip(w, cents)) / s,
            sum(wi * c[1] for wi, c in zip(w, cents)) / s,
        )
        shares = lever_shares(areas, cents, point)
        assert sum(shares) == pytest.approx(1.0)
        assert min(shares) >= 0
        mx = sum(f * c[0] for f, c in zip(shares, cents))
        my = sum(f * c[1] for f, c in zip(shares, cents))
        assert (mx, my) == pytest.approx(point, abs=1e-6)


def test_lever_prefers_supporter_under_load():
    shares = lever_shares([0.1, 0.1], [(0.0, 0.0), (1.0, 0.0)], (0.2, 0.0))
    assert shares[0] == pytest.approx(0.8)
    assert shares[1] == pytest.approx(0.2)


def test_mckee_capacity_reasonable():
    cap = mckee_capacity_n(0.4, 0.3, 5000.0, 0.003, 4.0)
    assert 300 < cap < 700  # ~ 47 kg allowable for a 40x30 single-wall box
