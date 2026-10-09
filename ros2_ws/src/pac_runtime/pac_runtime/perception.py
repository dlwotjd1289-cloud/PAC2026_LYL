"""Stage 1 (virtual): inline scale -> ID / label -> top-view RGB-D -> raw box
observation. The base view is used only when stage 2 reports an anomaly.

``FieldBox`` is the ground truth of one arriving box (what is really on the
conveyor). Only the observation leaves this module; the truth is kept by the
simulation for execution and metrics.
"""

from dataclasses import dataclass, replace

from pac_common import BoxStatus, Size3D


@dataclass(frozen=True)
class FieldBox:
    truth: object                 # true BoxState (true size / weight / sku)
    damaged: bool = False         # dented top (true)
    capacity_n: float | None = None  # true top-load capacity (metrics only)


@dataclass(frozen=True)
class RawObservation:
    box_id: str
    label_sku: str | None         # None: label / ID not read
    weight_kg: float
    size: Size3D                  # top view: x, y footprint and height
    confidence: float
    visual_damage: bool
    view: str                     # "top" | "base"
    stamp_sec: float


class PerceptionSim:
    def __init__(self, config, rng):
        self.cfg = config
        self.rng = rng

    def _size(self, true, std):
        g = self.rng.gauss
        return Size3D(*(max(0.01, round(v + g(0.0, std), 4)) for v in (true.x, true.y, true.z)))

    def observe(self, field, stamp_sec):
        c, t = self.cfg, field.truth
        uncertain = self.rng.random() < c.uncertain_probability
        std = c.uncertain_size_noise_std_m if uncertain else c.size_noise_std_m
        weight = max(0.01, round(t.weight_kg * (1.0 + self.rng.gauss(0.0, c.weight_noise_std_ratio)), 4))
        label = None if self.rng.random() < c.label_fail_probability else t.sku_id
        damage = field.damaged and self.rng.random() < c.damage_detect_probability
        return RawObservation(t.box_id, label, weight, self._size(t.size, std),
                              c.uncertain_confidence if uncertain else 1.0, damage, "top", stamp_sec)

    def base_view(self, field, previous):
        """Second look from the auxiliary camera (anomaly only)."""
        c, t = self.cfg, field.truth
        label = previous.label_sku
        if label is None and self.rng.random() < c.base_view_recover_probability:
            label = t.sku_id
        return replace(previous, label_sku=label, size=self._size(t.size, c.base_view_size_noise_std_m),
                       confidence=1.0 if label is not None else previous.confidence, view="base")


def to_box_state(obs, sku_spec, size=None, weight=None):
    """Measured ``BoxState`` handed to the State Manager (status MEASURED)."""
    from pac_common import BoxState, Pose3D

    return BoxState(
        box_id=obs.box_id,
        sku_id=sku_spec.sku_id,
        size=size or obs.size,
        weight_kg=float(weight if weight is not None else obs.weight_kg),
        pose=Pose3D("conveyor", 0.0, 0.0, 0.0),
        allowed_yaws_rad=tuple(sku_spec.allowed_yaws_rad),
        status=BoxStatus.MEASURED,
        confidence=float(obs.confidence),
        stamp_sec=float(obs.stamp_sec),
        source="RUNTIME_PERCEPTION_" + obs.view.upper(),
    )
