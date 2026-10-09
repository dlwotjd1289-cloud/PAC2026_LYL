"""Stage 1 raw signals that ``RawObservation`` does not carry.

No classification here: the values are measurements (or ``None`` when not
measured). Stage 2 (``pac_reinspection``) decides what they mean. Keeping them
in a separate dataclass leaves the ``RawObservation`` contract unchanged.
"""
from __future__ import annotations

from dataclasses import dataclass

from .depth_measurement import DepthConfig, DepthMeasurement


@dataclass(frozen=True)
class PerceptionSignals:
    label_payload_ok: bool | None = None   # label decoded completely / checksum valid
    base_label_sku: str | None = None      # label read by a second view (inlet / base camera)
    scale_settled: bool | None = None      # AutoScaleCycle reached a stable reading
    scale_stdev_kg: float | None = None    # spread of the averaged scale window
    tare_ok: bool | None = None            # empty-scale zero inside its tolerance
    frame_sizes: tuple = ()                # Size3D per frame (multi-frame capture)
    in_full_view: bool | None = None       # box not cut by the ROI / image edge
    valid_ratio: float | None = None       # valid depth pixels around the box
    box_count: int | None = None           # separate boxes seen in the ROI
    arrival_yaw_rad: float | None = None   # long-side yaw on the conveyor
    damage_score: float | None = None      # largest damage cue / its threshold (1.0 = at threshold)


def signals_from_measurement(measurement: DepthMeasurement, config: DepthConfig, frames=(),
                             **extra) -> PerceptionSignals:
    """Depth part of the signals. ``frames``: more ``DepthMeasurement``s of the same box;
    ``extra``: scale / label fields (e.g. from ``AutoScaleCycle`` and the label reader)."""
    m = measurement
    score = None
    if m.in_full_view:
        score = max(m.dent_depth_m / config.max_dent_m, m.tilt_rad / config.max_tilt_rad,
                    m.corner_gap_m / config.max_corner_gap_m,
                    (1.0 - m.rectangularity) / (1.0 - config.min_rectangularity))
    return PerceptionSignals(
        frame_sizes=tuple(f.size for f in frames) or (m.size,),
        in_full_view=m.in_full_view,
        valid_ratio=m.valid_ratio,
        box_count=m.box_count,
        arrival_yaw_rad=m.pose.yaw,
        damage_score=score,
        **extra,
    )
