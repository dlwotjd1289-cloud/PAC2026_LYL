"""Stage 1 on the virtual cell with Donghan's depth algorithm.

Drop-in for ``pac_runtime.perception.PerceptionSim`` (``observe`` /
``base_view``). Per box:

    scale weight (noise) ─┐
    label reader (may fail) ─┼─► build_raw_observation ─► RawObservation ─► stage 2
    synthetic depth image ─ measure_box ─┤        (size, damage, confidence)
    label unread ─ resolve_sku (size + weight) ─┘

Size, dent / crushed corner and confidence come only from
``pac_perception.depth_measurement.measure_box`` on the rendered image; the
ground truth is used only to draw the scene and to score perception after the
run (``records``).
"""
from __future__ import annotations

from dataclasses import replace
import math
import random

import numpy as np

from pac_common import Size3D
from pac_perception.depth_measurement import measure_box
from pac_perception.raw_observation import build_raw_observation
from pac_runtime.perception import RawObservation

from .depth_camera import CameraSpec, at_stopper, render


class DepthPerception:
    def __init__(self, cell, depth_config, resolver_config, perception_config, seed, *,
                 camera=CameraSpec(), dent_m=0.025, crushed_corner_m=0.06, queue_probability=0.5):
        self.cell = cell
        self.depth_cfg = depth_config
        self.resolver_cfg = resolver_config
        self.pcfg = perception_config      # runtime.yaml perception: scale / label noise only
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(self.rng.getrandbits(32))
        self.camera = camera
        self.camera_model = camera.model()
        self.dent_m = dent_m
        self.crushed_corner_m = crushed_corner_m
        self.queue_probability = queue_probability
        self.next_in_queue = {}            # box id -> FieldBox right behind it on the conveyor
        for a, b in zip(cell.stream, cell.stream[1:]):
            self.next_in_queue[a.truth.box_id] = b
        self.records = []                  # one per measurement, for the perception report

    # ---- scene ---------------------------------------------------------
    def _damage(self, field):
        if not field.damaged:
            return {}
        # deterministic per box: dented top or crushed corner
        kind = random.Random(field.truth.box_id).random() < 0.5
        return {"dent_m": self.dent_m} if kind else {"crushed_corner_m": self.crushed_corner_m}

    def _scene(self, field, with_queue):
        t = field.truth.size
        length, width = max(t.x, t.y), min(t.x, t.y)
        y = self.rng.uniform(-0.03, 0.03)
        yaw = math.radians(self.rng.uniform(-2.0, 2.0))
        boxes = [at_stopper(length, width, t.z, y=y, yaw=yaw, **self._damage(field))]
        nxt = self.next_in_queue.get(field.truth.box_id)
        if with_queue and nxt is not None and self.rng.random() < self.queue_probability:
            n = nxt.truth.size
            gap = self.rng.uniform(0.06, 0.25)
            nl = max(n.x, n.y)
            boxes.append(replace(at_stopper(nl, min(n.x, n.y), n.z, y=self.rng.uniform(-0.03, 0.03)),
                                 x=boxes[0].x - length / 2 - gap - nl / 2))
        return boxes

    # ---- stage 1 -------------------------------------------------------
    def _measure(self, field, stamp_sec, view, label, with_queue):
        t = field.truth
        weight = max(0.01, t.weight_kg * (1.0 + self.rng.gauss(0.0, self.pcfg.weight_noise_std_ratio)))
        rec = {"box_id": t.box_id, "view": view, "true_size": [t.size.x, t.size.y, t.size.z],
               "true_damaged": field.damaged, "label_read": label is not None}
        try:
            m = measure_box(render(self.camera, self._scene(field, with_queue), self.np_rng),
                            self.camera_model, self.depth_cfg)
        except ValueError as error:
            rec.update(error=str(error))
            self.records.append(rec)
            return RawObservation(t.box_id, None, round(weight, 4), Size3D(0.01, 0.01, 0.01), 0.0, False,
                                  view, stamp_sec)
        obs = build_raw_observation(t.box_id, weight, m, stamp_sec, label_sku=label, catalog=self.cell.catalog,
                                    weight_ranges=self.cell.weight_ranges, resolver_config=self.resolver_cfg)
        rec.update(measured_size=[m.size.x, m.size.y, m.size.z], damage_reasons=list(m.damage_reasons),
                   dent_mm=round(m.dent_depth_m * 1000, 1), rectangularity=round(m.rectangularity, 3),
                   confidence=obs.confidence, boxes_in_roi=m.box_count, sku=obs.label_sku,
                   sku_resolved=label is None and obs.label_sku is not None, true_sku=t.sku_id)
        self.records.append(rec)
        return replace(obs, view=view)

    def observe(self, field, stamp_sec):
        label = None if self.rng.random() < self.pcfg.label_fail_probability else field.truth.sku_id
        return self._measure(field, stamp_sec, "top", label, with_queue=True)

    def base_view(self, field, previous):
        """Second look (anomaly only): re-measure; the side label may be read now."""
        label = previous.label_sku
        if label is None and self.rng.random() < self.pcfg.base_view_recover_probability:
            label = field.truth.sku_id
        return self._measure(field, previous.stamp_sec, "base", label, with_queue=False)
