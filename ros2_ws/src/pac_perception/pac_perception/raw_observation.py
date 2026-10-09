"""Stage 1 output: scale weight + depth measurement (+ label) -> ``RawObservation``.

``RawObservation`` is the stage 1 -> stage 2 contract defined in
``pac_runtime.perception``; ``StateValidator.validate`` consumes it directly.
"""
from __future__ import annotations

import math

from .depth_measurement import DepthMeasurement
from .sku_resolver import ResolverConfig, resolve_sku


def build_raw_observation(
    box_id: str,
    weight_kg: float,
    measurement: DepthMeasurement,
    stamp_sec: float,
    label_sku: str | None = None,
    catalog=None,
    weight_ranges=None,
    remaining_by_sku=None,
    resolver_config: ResolverConfig = ResolverConfig(),
):
    """``weight_kg`` is the settled scale reading (e.g. ``AutoScaleCycle.measured_kg``).

    Without a label, the SKU is estimated from size and weight when ``catalog`` is
    given; the observation confidence is capped by the estimate's confidence.
    """
    from pac_runtime.perception import RawObservation

    if not isinstance(box_id, str) or not box_id.strip():
        raise ValueError("box_id must be nonempty text")
    if not math.isfinite(weight_kg) or weight_kg < 0:
        raise ValueError(f"Invalid weight_kg: {weight_kg}")
    if not math.isfinite(stamp_sec):
        raise ValueError(f"Invalid stamp_sec: {stamp_sec}")

    confidence = measurement.confidence
    if label_sku is None and catalog is not None:
        resolution = resolve_sku(measurement.size, weight_kg, catalog, weight_ranges or {},
                                 remaining_by_sku, resolver_config)
        if resolution.sku_id is not None:
            label_sku = resolution.sku_id
            confidence = min(confidence, resolution.confidence)

    return RawObservation(
        box_id=box_id,
        label_sku=label_sku,
        weight_kg=round(float(weight_kg), 4),
        size=measurement.size,
        confidence=float(confidence),
        visual_damage=measurement.damaged,
        view="top",
        stamp_sec=float(stamp_sec),
    )
