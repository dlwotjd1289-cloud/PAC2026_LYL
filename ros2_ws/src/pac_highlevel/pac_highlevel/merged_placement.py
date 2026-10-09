"""Shared placement score; no new search and no dynamic hard constraint.

``blend`` is an unvalidated design assumption. Convex mixing shares one
effect-group budget; it does not make correlated components independent.
The existing Jaesung S/L
definitions and piecewise targets are retained, but their old experiments do
not establish the performance of this blend. Side contact is a height-grid
approximation, not measured lateral support. An open pallet edge is not a wall.
"""
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from pac_candidates.geometry import footprint, rotated_dims
from pac_candidates.pallet_model import G
from .placement import heightmap, ring_metrics


CELL_M = 0.02


@dataclass(frozen=True)
class MergedConfig:
    blend: float = 0.25
    official_evaluation: bool = False
    horizon: int = 5
    s_target: float = 0.75
    s_lo: float = 20.0
    s_hi: float = 2.0
    s_weights: tuple = (0.20, 0.25, 0.40, 0.15)
    l_target: float = 0.5
    l_lo: float = 8.0
    l_hi: float = 0.5
    a_target_g: float = 0.3  # soft normalization only; no stack filter
    mu: float = 0.4         # existing model assumption, not measured friction

    def __post_init__(self):
        if type(self.official_evaluation) is not bool:
            raise ValueError("official_evaluation must be a boolean")
        numeric = (self.blend, self.s_target, self.s_lo, self.s_hi,
                   self.l_target, self.l_lo, self.l_hi, self.a_target_g, self.mu)
        if any(isinstance(v, bool) or not math.isfinite(v) for v in numeric):
            raise ValueError("Merged parameters must be finite numbers")
        if not 0 <= self.blend <= 1 or not 0 <= self.s_target <= 1 or not 0 <= self.l_target <= 1:
            raise ValueError("Blend and targets must be in [0, 1]")
        if type(self.horizon) is not int or not 0 <= self.horizon <= 5:
            raise ValueError("Observed horizon must be an integer in [0, 5]")
        if not self.s_lo >= self.s_hi >= 0 or not self.l_lo >= self.l_hi >= 0:
            raise ValueError("Reward slopes must be nonnegative and diminishing")
        weights = tuple(float(v) for v in self.s_weights)
        if len(weights) != 4 or any(not math.isfinite(v) or v < 0 for v in weights) or not math.isclose(sum(weights), 1.):
            raise ValueError("Four finite nonnegative S weights must sum to one")
        object.__setattr__(self, "s_weights", weights)
        if min(self.a_target_g, self.mu) <= 0 or piecewise_reward(1., self.s_target, self.s_lo, self.s_hi) <= 0:
            raise ValueError("Acceleration normalization, friction and S normalization must be positive")


def merged_config_from_dict(data):
    values = dict(data or {})
    unknown = set(values) - set(MergedConfig.__dataclass_fields__)
    if unknown:
        raise ValueError(f"Unknown merged keys: {sorted(unknown)}")
    return MergedConfig(**values)


def load_merged_config(path=None):
    if path is None:
        return MergedConfig()
    import yaml
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return merged_config_from_dict(data.get("merged", data))


def piecewise_reward(value, target, low, high):
    return low * min(value, target) + high * max(0., value-target)


@dataclass(frozen=True)
class FutureContext:
    """Immutable snapshot of permitted future information, never a world reference.

    Missing SKU weight ranges use (catalogue nominal, catalogue nominal).
    ``range_sources`` records that assumption; it is not a measured range.
    """
    known_weights: tuple = ()
    remaining_by_sku: tuple = ()
    weight_ranges: tuple = ()
    range_sources: tuple = ()
    known_box_ids: tuple = ()
    visible_box_ids: tuple = ()
    camera_box_ids: tuple = ()

    def heavier_share(self, weight):
        if not math.isfinite(weight) or weight <= 0:
            raise ValueError("Candidate weight must be finite and positive")
        ranges = dict(self.weight_ranges)
        total, heavy = 0., 0.
        for sku, count in self.remaining_by_sku:
            lo, hi = ranges[sku]
            probability = float(hi > weight + .5) if hi <= lo else float(np.clip((hi-weight-.5)/(hi-lo), 0., 1.))
            total += count
            heavy += count * probability
        for known in self.known_weights:
            total += 1.
            heavy += float(known > weight+.5)
        return heavy/total if total else 0.


def future_context_from_world(world, box=None, horizon=5):
    """Read only current/buffer, the explicitly bounded visible slice and SKU totals.

    No iteration over the full arrival stream or hidden truth is permitted.
    Remaining counts already exclude current and buffered boxes. Visible boxes
    are subtracted once, including the candidate if it occurs in that slice.
    """
    if type(horizon) is not int or not 0 <= horizon <= 5:
        raise ValueError("Observed horizon must be in [0, 5]")
    visible_limit = getattr(world, "visible_horizon", 5)
    if type(visible_limit) is not int or not 0 <= visible_limit <= 5:
        raise ValueError("World visible horizon must be in [0, 5]")
    horizon = min(horizon, visible_limit)
    measured_count = getattr(world, "measured_preview_count", None)
    if measured_count is not None and (type(measured_count) is not int or not 0 <= measured_count <= 5):
        raise ValueError("Measured preview count must be None or in [0, 5]")
    excluded = getattr(box, "box_id", box)
    known, visible_ids, camera_ids, seen = [], [], [], set()
    known_ids = getattr(world, "known_weight_ids", None)
    camera_counts = {}

    def remember(item, observed=True):
        if item is None or item.box_id == excluded or item.box_id in seen:
            return
        seen.add(item.box_id)
        if known_ids is not None:
            observed = item.box_id in known_ids
        if not observed:
            camera_ids.append(item.box_id)
            camera_counts[item.sku_id] = camera_counts.get(item.sku_id, 0)+1
            return
        weight = float(item.weight_kg)
        if not math.isfinite(weight) or weight <= 0:
            raise ValueError("Observed future weight must be finite and positive")
        known.append((item.box_id, weight))

    current = getattr(world, "current", None)
    remember(current.box if current is not None else None)
    for entry in world.buffer:
        remember(entry.arrival.box if entry is not None else None)
    remaining = {k: int(v) for k, v in world.remaining_by_sku().items() if v > 0}
    start = world.next_arrival
    for index, arrival in enumerate(world.arrivals[start:start+horizon]):
        item = arrival.box
        if item.box_id in visible_ids:
            continue
        visible_ids.append(item.box_id)
        remaining[item.sku_id] = max(0, remaining.get(item.sku_id, 0)-1)
        # Preserve observation provenance when an existing search branch advances.
        # A camera-only box does not become weighed because its array index moved.
        remember(item, observed=measured_count is None or index < measured_count)
    for sku, count in camera_counts.items():
        remaining[sku] = remaining.get(sku, 0)+count
    remaining = {k: v for k, v in remaining.items() if v > 0}
    provided = getattr(world, "weight_ranges", None) or {}
    ranges, sources = [], []
    for sku in sorted(remaining):
        if sku in provided:
            lo, hi = map(float, provided[sku])
            source = "declared_sku_weight_range_assumed_uniform"
        else:
            lo = hi = float(world.catalog[sku].weight_kg)
            source = "catalog_nominal_degenerate_range_assumption"
        if not all(math.isfinite(v) and v > 0 for v in (lo, hi)) or lo > hi:
            raise ValueError(f"Invalid weight range for {sku}")
        ranges.append((sku, (lo, hi)))
        sources.append((sku, source))
    return FutureContext(tuple(w for _, w in known), tuple(sorted(remaining.items())),
                         tuple(ranges), tuple(sources), tuple(k for k, _ in known),
                         tuple(visible_ids), tuple(camera_ids))


def _signed_distance(poly, point):
    if len(poly) < 3:
        return -1.
    area = sum(poly[i][0]*poly[(i+1) % len(poly)][1]-poly[(i+1) % len(poly)][0]*poly[i][1]
               for i in range(len(poly)))
    sign = 1. if area > 0 else -1.
    distance = math.inf
    for i, (x, y) in enumerate(poly):
        nx, ny = poly[(i+1) % len(poly)]
        ex, ey = nx-x, ny-y
        distance = min(distance, sign*(ex*(point[1]-y)-ey*(point[0]-x))/(math.hypot(ex, ey) or 1e-12))
    return distance


def _jaesung_grid(pallet):
    grid = np.zeros((max(1, int(round(pallet.size.x/CELL_M))), max(1, int(round(pallet.size.y/CELL_M)))))
    for item in pallet.boxes:
        dx, dy, dz = rotated_dims(item.size, item.pose.yaw)
        i0, i1, j0, j1 = _region(grid, item.pose.x, item.pose.y, dx, dy)
        np.maximum(grid[i0:i1, j0:j1], item.pose.z+dz, out=grid[i0:i1, j0:j1])
    return grid


def _region(grid, x, y, dx, dy):
    nx, ny = grid.shape
    i0, j0 = max(0, int(round(x/CELL_M))), max(0, int(round(y/CELL_M)))
    i1, j1 = min(nx, max(i0+1, int(round((x+dx)/CELL_M)))), min(ny, max(j0+1, int(round((y+dy)/CELL_M))))
    return i0, i1, j0, j1


def _jaesung_metrics(candidate, box, metrics, grid, pallet_size, cfg, heavy):
    p = candidate.target_pose
    dx, dy, dz = rotated_dims(box.size, p.yaw)
    support = float(metrics.get("support_ratio", 1.))
    on_floor = bool(metrics.get("on_floor", p.z < 1e-6))
    shares = tuple((metrics.get("supporter_shares") or {}).values())
    interlock = 1. if on_floor else min(1., 2.*(1.-max(shares))) if len(shares) >= 2 else 0.
    i0, i1, j0, j1 = _region(grid, p.x, p.y, dx, dy)
    nx, ny = grid.shape
    half = p.z+.5*dz

    def fraction(line):
        return 0. if line is None or not line.size else float(np.mean(line >= half))

    sides = ((-1, 0, fraction(grid[i0-1, j0:j1] if i0 > 0 else None)),
             (1, 0, fraction(grid[i1, j0:j1] if i1 < nx else None)),
             (0, -1, fraction(grid[i0:i1, j0-1] if j0 > 0 else None)),
             (0, 1, fraction(grid[i0:i1, j1] if j1 < ny else None)))
    side = sum(v for _, _, v in sides)/4.
    poly = metrics.get("support_polygon") or ()
    if on_floor or len(poly) < 3:
        poly = ((p.x, p.y), (p.x+dx, p.y), (p.x+dx, p.y+dy), (p.x, p.y+dy))
    cx, cy, h, acceleration = p.x+dx/2, p.y+dy/2, dz/2, 1.
    for ux, uy, blocked in sides:
        if blocked >= .5:
            continue
        lo, hi = 0., 1.
        if _signed_distance(poly, (cx+ux*hi*h, cy+uy*hi*h)) > 0:
            tipping = hi
        else:
            for _ in range(10):
                mid = .5*(lo+hi)
                if _signed_distance(poly, (cx+ux*mid*h, cy+uy*mid*h)) > 0:
                    lo = mid
                else:
                    hi = mid
            tipping = lo
        acceleration = min(acceleration, tipping, cfg.mu)
    tilt = float(np.clip(acceleration/cfg.a_target_g, 0., 1.))
    s = float(np.dot(cfg.s_weights, (support, interlock, tilt, side)))
    load = float(np.clip(1.-float(metrics.get("max_load_ratio", 0.)), 0., 1.))
    if not all(math.isfinite(v) for v in (support, interlock, tilt, side, s, load)):
        raise ValueError("Nonfinite candidate stability metrics")
    under = grid[i0:i1, j0:j1]
    void = float(np.clip(p.z-under, 0., None).sum())*CELL_M*CELL_M/(dx*dy*dz)
    after = grid.copy()
    after[i0:i1, j0:j1] = p.z+dz
    area = dx*dy/(pallet_size.x*pallet_size.y)
    return dict(support=support, interlock=interlock, tilt=tilt, side=side,
                critical_acceleration_g=acceleration, S=s, L=load,
                rough=float(after.std())/pallet_size.z, void=void,
                block=area*heavy*float(p.z+dz < pallet_size.z-.09),
                floor=area*heavy*float(p.z < 1e-6), heavier_share=heavy)


def _capacity_utilization_concentration(load_capacity_pairs):
    """Excess maximum utilization over capacity-weighted mean utilization.

    Only boxes actually bearing top load enter this comparison. Unequal loads
    proportional to unequal capacities have zero concentration penalty. This is
    utilization imbalance among active structural interfaces, not a claim of
    uniform pallet pressure or measured strength. A single loaded box has zero
    dispersion; its absolute utilization is covered by the existing L term.
    """
    active = []
    for load, capacity in load_capacity_pairs:
        if not math.isfinite(load) or not math.isfinite(capacity) or load < -1e-8:
            raise ValueError("Invalid top-load evidence")
        if load <= 0.:
            continue
        if capacity <= 0. or load/capacity > 1.+1e-8:
            raise ValueError("Invalid capacity utilization evidence")
        active.append((load, capacity))
    if not active:
        return 0.
    maximum = max(load/capacity for load, capacity in active)
    weighted_mean = sum(load for load, _ in active)/sum(capacity for _, capacity in active)
    return float(np.clip(maximum-weighted_mean, 0., 1.))


def _official_balance(candidate, box, state, backend, metrics):
    """Use the existing hard-mask geometry/load model; never relax its limits."""
    load_ratio = metrics["max_load_ratio"]
    if isinstance(load_ratio, bool):
        raise ValueError("Boolean is not load-ratio evidence")
    load_ratio = float(load_ratio)
    if not math.isfinite(load_ratio) or load_ratio < 0.:
        raise ValueError("Missing/invalid maximum load ratio")
    margin = float(metrics["pallet_cog_margin_ratio"])
    if not math.isfinite(margin) or not 0. <= margin <= 1.:
        raise ValueError("Missing/invalid pallet CoG margin")
    # Exact post-placement lever/area load increments, including redistribution.
    model = backend.model_for(state)
    p = candidate.target_pose
    dx, dy, _ = rotated_dims(box.size, p.yaw)
    rect = footprint(p.x, p.y, dx, dy)
    tolerance = float(metrics["size_tolerance_m"])
    if not math.isfinite(tolerance) or tolerance < 0.:
        raise ValueError("Invalid support tolerance evidence")
    on_floor, found = model.contacts_for(rect, tolerance, p.z)
    contacts = () if on_floor else model.contact_list(rect, found, rect.center)
    extra = model.propagate(contacts, box.weight_kg*G)
    pairs = [(model.top_load_n[g.box_id]+extra.get(g.box_id, 0.), g.capacity_n)
             for g in model.boxes]
    concentration = _capacity_utilization_concentration(pairs)
    return dict(pallet_cog_margin=margin, cog_deficit=1.-margin,
                load_concentration=concentration,
                load_concentration_model="active_top_load_capacity_utilization_dispersion",
                balance_weights_assumption=(.50, .25, .25))


class MergedPlacer:
    wants_context = True
    name = "merged"

    def __init__(self, config=None):
        self.cfg = config or MergedConfig()
        self.last_details = {}  # diagnostics only; never read to make a choice

    def scores(self, valid, box, state, backend, future_context=None):
        future = future_context or FutureContext()
        cfg, pallet = self.cfg, state.pallet
        X, Y, H = pallet.size.x, pallet.size.y, pallet.size.z
        if not all(math.isfinite(v) and v > 0 for v in (X, Y, H, box.size.x, box.size.y, box.size.z)):
            raise ValueError("Finite positive pallet and box dimensions required")
        grid = heightmap(pallet, CELL_M)
        ja_grid = _jaesung_grid(pallet)
        tol = backend.config.uncertainty.height_tolerance_m
        heights = sorted(s.size.z for s in backend.context.catalog.values())
        ref = heights[len(heights)//2] if heights else .2
        if not math.isfinite(ref) or ref <= 0:
            raise ValueError("Finite positive reference height required")
        alpha, heavy = cfg.blend, future.heavier_share(box.weight_kg)
        phi_s_max = piecewise_reward(1., cfg.s_target, cfg.s_lo, cfg.s_hi)
        phi_l_max = piecewise_reward(1., cfg.l_target, cfg.l_lo, cfg.l_hi)
        details, out = {}, []
        for candidate in valid:
            verdict = backend.validate_constraints(box, candidate, state)
            if not verdict.success:
                continue
            p = candidate.target_pose
            dx, dy, dz = rotated_dims(box.size, p.yaw)
            nonlevel, step, flush, contact = ring_metrics(grid, p.x, p.y, p.z, dx, dy, p.z+dz, CELL_M, tol, ref)
            balance = {}
            if cfg.official_evaluation:
                try:
                    balance = _official_balance(candidate, box, state, backend,
                                                verdict.details.get("metrics", {}))
                except (AttributeError, KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
                    details[candidate.candidate_id] = dict(fail_closed=True,
                        reason="OFFICIAL_BALANCE_EVIDENCE_UNAVAILABLE", error=str(exc),
                        official=dict(space_cost=None, balance_cost=None, stability_cost=None,
                                      time_cost=None, execution_cost=None,
                                      status="balance_evidence_unavailable"))
                    continue
            raw = _jaesung_metrics(candidate, box, verdict.details.get("metrics", {}), ja_grid, pallet.size, cfg, heavy)
            raw.update(balance)
            phi_s = piecewise_reward(raw["S"], cfg.s_target, cfg.s_lo, cfg.s_hi)
            phi_l = piecewise_reward(raw["L"], cfg.l_target, cfg.l_lo, cfg.l_hi)
            residual_s, residual_l = (phi_s_max-phi_s)/phi_s_max, phi_l_max-phi_l
            contributions = dict(height=p.z/H, corner=.01*(p.x+p.y)/(X+Y),
                flat_nonlevel=(1-alpha)*.3*nonlevel, flat_step=(1-alpha)*.1*step,
                flat_flush=(1-alpha)*.05*(1-flush), flat_global=alpha*.45*raw["rough"],
                stability_contact=(1-alpha)*.5*(1-contact), stability_support=(1-alpha)*.1*(1-raw["support"]),
                stability_jaesung=alpha*.6*residual_s,
                load_margin=alpha*(.6/phi_s_max)*residual_l,
                void=alpha*(.6/phi_s_max)*1.5*raw["void"],
                future_block=alpha*(.6/phi_s_max)*1.5*raw["block"],
                future_floor=alpha*(.6/phi_s_max)*.5*raw["floor"])
            if cfg.official_evaluation:
                # Share the old load-margin maximum budget; ratios are assumptions.
                budget = alpha*(.6/phi_s_max)*phi_l_max
                normalized_load = residual_l/phi_l_max if phi_l_max > 0. else 0.
                contributions["load_margin"] = budget*.50*normalized_load
                contributions["pallet_cog"] = budget*.25*raw["cog_deficit"]
                contributions["load_concentration"] = budget*.25*raw["load_concentration"]
            if alpha == 0.:
                # Preserve PR9's exact floating-point evaluation/tie order.
                score = (1.*p.z/H + .3*nonlevel + .1*step + .05*(1.-flush)
                         + .5*(1.-contact) + .1*(1.-raw["support"]) + .01*(p.x+p.y)/(X+Y))
            else:
                score = sum(contributions.values())
            if not math.isfinite(score):
                raise ValueError("Nonfinite merged score")
            details[candidate.candidate_id] = dict(raw=dict(raw, nonlevel=nonlevel, step=step,
                flush=flush, contact=contact, phi_S=phi_s, phi_L=phi_l, residual_S=residual_s,
                residual_L_raw=residual_l), contributions=contributions, score=score,
                blend_assumption=alpha, side_support_model="height_grid_neighbor_proxy_not_physical_validation",
                future=dict(known_box_ids=future.known_box_ids, visible_box_ids=future.visible_box_ids,
                            camera_box_ids=future.camera_box_ids,
                            remaining_by_sku=future.remaining_by_sku, range_sources=future.range_sources))
            balance_keys = ("load_margin", "pallet_cog", "load_concentration")
            stability_keys = ("stability_contact", "stability_support", "stability_jaesung")
            balance_cost = sum(contributions.get(k, 0.) for k in balance_keys)
            stability_cost = sum(contributions[k] for k in stability_keys)
            space_cost = sum(v for k, v in contributions.items()
                             if k not in balance_keys and k not in stability_keys)
            details[candidate.candidate_id]["official"] = dict(
                space_cost=space_cost, balance_cost=balance_cost,
                stability_cost=stability_cost, time_cost=None, execution_cost=None,
                status="geometry_only")
            out.append((score, round(p.z, 6), round(p.y, 6), round(p.x, 6), candidate.candidate_id, candidate))
        self.last_details = details
        return out

    def __call__(self, valid, box, state, backend):
        scores = self.scores(valid, box, state, backend)
        return min(scores, key=lambda row: row[:5])[-1] if scores else None

    def choose_in_world(self, valid, box, state, backend, world):
        future = future_context_from_world(world, box, self.cfg.horizon)
        scores = self.scores(valid, box, state, backend, future)
        return min(scores, key=lambda row: row[:5])[-1] if scores else None


__all__ = ["MergedConfig", "MergedPlacer", "FutureContext", "future_context_from_world",
           "merged_config_from_dict", "load_merged_config", "piecewise_reward"]
