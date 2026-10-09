"""Synthetic top-view depth camera over the pick zone (virtual plant only).

Renders what a straight-down depth camera sees at the conveyor stopper: the
roller surface plus the top faces of the box at the stopper and, optionally,
the next box queued upstream. Damage is drawn into the geometry (dented top or
crushed corner); the depth algorithm has to find it, nothing is passed through.

Frame: ``conveyor`` of ``config/donghan/perception_depth.yaml`` (origin on the
roller top at the pick-zone centre, x downstream, stopper face at
``STOPPER_X_M``). Adapted from ``tests/donghan/perception_testkit.py``.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from pac_perception.depth_measurement import CameraModel

STOPPER_X_M = 0.235


@dataclass(frozen=True)
class CameraSpec:
    height_m: float = 1.5           # optical centre above the rollers
    width_px: int = 480
    height_px: int = 360
    focal_px: float = 450.0         # ~2.4 mm / px on a 0.4 m tall box
    centre_x_m: float = -0.13       # optical axis over the ROI centre
    depth_noise_std_m: float = 0.001
    hole_ratio: float = 0.002       # invalid depth pixels (specular / dropout)

    def model(self) -> CameraModel:
        # optical x -> conveyor x, optical y -> -conveyor y, optical z -> -conveyor z
        t = np.eye(4)
        t[:3, :3] = np.diag([1.0, -1.0, -1.0])
        t[0, 3] = self.centre_x_m
        t[2, 3] = self.height_m
        return CameraModel(self.focal_px, self.focal_px, self.width_px / 2, self.height_px / 2, t)


@dataclass(frozen=True)
class BoxView:
    length: float                   # along the conveyor (x)
    width: float
    height: float
    x: float                        # top-face centre, conveyor frame
    y: float = 0.0
    yaw: float = 0.0
    dent_m: float = 0.0
    crushed_corner_m: float = 0.0


def at_stopper(length, width, height, *, gap_m=0.0, y=0.0, yaw=0.0, **damage) -> BoxView:
    """Box resting ``gap_m`` upstream of the stopper face."""
    return BoxView(length, width, height, STOPPER_X_M - gap_m - length / 2, y, yaw, **damage)


def render(spec: CameraSpec, boxes, rng: np.random.Generator) -> np.ndarray:
    """Depth image (m along the optical axis); the nearest top face wins per pixel."""
    v, u = np.mgrid[0:spec.height_px, 0:spec.width_px].astype(float)
    cx, cy, f, h = spec.width_px / 2, spec.height_px / 2, spec.focal_px, spec.height_m
    depth = np.full(u.shape, h)
    for b in boxes:
        z_top = h - b.height
        px = (u - cx) / f * z_top + spec.centre_x_m
        py = -(v - cy) / f * z_top
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        a, bb = c * (px - b.x) + s * (py - b.y), -s * (px - b.x) + c * (py - b.y)
        on_top = (np.abs(a) <= b.length / 2) & (np.abs(bb) <= b.width / 2)
        if b.crushed_corner_m:
            on_top &= (b.length / 2 - a) + (b.width / 2 - bb) > b.crushed_corner_m
        top = np.full(on_top.shape, b.height)
        if b.dent_m:
            r = min(b.length, b.width) / 4
            top -= b.dent_m * np.clip(1 - np.hypot(a, bb) / r, 0, None)
        depth = np.where(on_top & (h - top < depth), h - top, depth)
    depth = depth + rng.normal(0.0, spec.depth_noise_std_m, depth.shape)
    if spec.hole_ratio:
        depth[rng.random(depth.shape) < spec.hole_ratio] = np.nan
    return depth
