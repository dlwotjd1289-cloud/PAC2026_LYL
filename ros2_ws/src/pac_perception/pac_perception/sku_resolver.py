"""Stage 1 label fallback: estimate the SKU from measured size and scale weight.

Used only when the ID / label is not read. Tolerances have the same meaning as
``pac_runtime.config.ValidatorConfig`` so a resolved SKU also passes stage 2's
size / weight cross-check. Ambiguous or unmatched boxes stay unlabelled, which
stage 2 routes to the base view and then Inspection / NG.
"""
from __future__ import annotations

from dataclasses import dataclass

from pac_common import Size3D, SkuSpec


@dataclass(frozen=True)
class ResolverConfig:
    size_tolerance_m: float = 0.012
    size_tolerance_ratio: float = 0.04
    weight_tolerance_ratio: float = 0.08
    inferred_confidence: float = 0.9   # < 1.0: stage 2 treats the box as uncertain (delta)


@dataclass(frozen=True)
class SkuResolution:
    sku_id: str | None
    confidence: float
    candidates: tuple[str, ...]
    reason: str  # "unique" | "ambiguous" | "no_match"


def resolve_sku(
    size: Size3D,
    weight_kg: float,
    catalog: dict[str, SkuSpec],
    weight_ranges: dict[str, tuple[float, float]],
    remaining_by_sku: dict[str, int] | None = None,
    config: ResolverConfig = ResolverConfig(),
) -> SkuResolution:
    """``catalog`` / ``weight_ranges`` as built by ``pac_runtime.order.cell_from_order``."""
    candidates = tuple(
        sku for sku, spec in sorted(catalog.items())
        if (remaining_by_sku is None or remaining_by_sku.get(sku, 0) > 0)
        and _size_fits(size, spec.size, config)
        and _weight_fits(weight_kg, weight_ranges.get(sku, (None, spec.weight_kg)), config)
    )
    if len(candidates) == 1:
        return SkuResolution(candidates[0], config.inferred_confidence, candidates, "unique")
    if candidates:
        return SkuResolution(None, 0.0, candidates, "ambiguous")
    return SkuResolution(None, 0.0, (), "no_match")


def _size_fits(measured: Size3D, nominal: Size3D, config: ResolverConfig) -> bool:
    def tol(v):
        return max(config.size_tolerance_m, config.size_tolerance_ratio * v)

    m = (measured.x, measured.y, measured.z)
    return any(
        all(abs(a - b) <= tol(b) for a, b in zip(m, n))
        for n in ((nominal.x, nominal.y, nominal.z), (nominal.y, nominal.x, nominal.z))
    )


def _weight_fits(weight: float, weight_range, config: ResolverConfig) -> bool:
    lo, hi = weight_range
    r = config.weight_tolerance_ratio
    if hi is not None and weight > hi * (1 + r):
        return False
    return lo is None or weight >= lo * (1 - r)
