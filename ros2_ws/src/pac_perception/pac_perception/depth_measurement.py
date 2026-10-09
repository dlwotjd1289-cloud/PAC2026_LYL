"""Stage 1 (top-view RGB-D): measure one box in the pick zone from an aligned depth image.

Input is a ROS-style depth image (float metres along the optical axis, NaN or 0 =
no reading) plus the camera intrinsics and the ``camera_3d -> conveyor`` transform.
Points are measured in the ``conveyor`` frame, whose z = 0 is the roller top.

The result carries the measured size, the box pose and damage cues (dented or
tilted top, crushed corner). It does not decide what to do with an anomaly;
stage 2 (``pac_runtime.state_validator``) does.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from pac_common import Pose3D, Size3D


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    conveyor_from_camera: np.ndarray  # 4x4, camera_3d optical frame -> conveyor frame

    def __post_init__(self):
        t = np.asarray(self.conveyor_from_camera, dtype=float)
        if t.shape != (4, 4) or not np.all(np.isfinite(t)):
            raise ValueError("conveyor_from_camera must be a finite 4x4 matrix")
        if min(self.fx, self.fy) <= 0:
            raise ValueError("focal lengths must be positive")
        object.__setattr__(self, "conveyor_from_camera", t)


@dataclass(frozen=True)
class DepthConfig:
    roi_xy_m: tuple[float, float, float, float]  # pick zone in conveyor frame: x_min, x_max, y_min, y_max
    min_depth_m: float = 0.2
    max_depth_m: float = 4.0
    min_box_height_m: float = 0.03       # points higher than this above the rollers belong to the box
    min_points: int = 200
    cluster_gap_m: float = 0.02          # boxes in the ROI are split along conveyor x at wider gaps
    edge_margin_m: float = 0.004         # box this close to the ROI edge is cut off: no damage call
    top_percentile: float = 90.0         # robust top height (ignores small dents and edge noise)
    interior_margin_m: float = 0.03      # top-face interior used for dent / tilt checks
    max_dent_m: float = 0.012            # deeper interior dip -> dented top
    max_tilt_rad: float = math.radians(3.0)
    min_rectangularity: float = 0.92     # hull area / min-area rectangle; lower -> crushed corner
    max_corner_gap_m: float = 0.02       # rectangle corner to nearest hull vertex; size independent
    min_valid_ratio: float = 0.85        # valid depth pixels around the box
    uncertain_confidence: float = 0.6    # same meaning as pac_runtime PerceptionConfig


@dataclass(frozen=True)
class DepthMeasurement:
    size: Size3D                 # x = long side, y = short side, z = height
    pose: Pose3D                 # bottom-face centre in the conveyor frame, yaw of the long side
    tilt_rad: float
    dent_depth_m: float
    rectangularity: float
    corner_gap_m: float
    valid_ratio: float
    point_count: int
    box_count: int                  # separate boxes seen in the ROI
    gap_upstream_m: float | None    # free conveyor length to the next box upstream (-x), None if none
    gap_downstream_m: float | None
    in_full_view: bool              # False: box reaches the ROI edge, size is a lower bound
    damage_reasons: tuple[str, ...]
    confidence: float

    @property
    def damaged(self) -> bool:
        return bool(self.damage_reasons)


def measure_box(depth: np.ndarray, camera: CameraModel, config: DepthConfig,
                target_x: float | None = None) -> DepthMeasurement:
    """Measure one box inside ``config.roi_xy_m``. Raises ``ValueError`` if none is found.

    Boxes queued on the conveyor can share the ROI, so box points are split along
    the conveyor x axis. The downstream-most box (at the stopper) is measured, or
    the one at ``target_x`` (conveyor frame). Touching boxes stay one cluster.
    """
    depth = np.asarray(depth, dtype=float)
    if depth.ndim != 2:
        raise ValueError("depth must be a 2-D image")
    valid = np.isfinite(depth) & (depth >= config.min_depth_m) & (depth <= config.max_depth_m)
    v, u = np.nonzero(valid)
    pts = _to_conveyor(u, v, depth[v, u], camera)

    x0, x1, y0, y1 = config.roi_xy_m
    in_roi = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
    box = in_roi & (pts[:, 2] > config.min_box_height_m)
    if int(box.sum()) < config.min_points:
        raise ValueError(f"no box in the pick zone ({int(box.sum())} points)")
    box_pts, bu, bv = pts[box], u[box], v[box]

    order = np.argsort(box_pts[:, 0], kind="stable")
    xs = box_pts[order, 0]
    clusters = np.split(order, np.nonzero(np.diff(xs) > config.cluster_gap_m)[0] + 1)
    spans = [(box_pts[c, 0].min(), box_pts[c, 0].max()) for c in clusters]
    if target_x is None:
        k = len(clusters) - 1
    else:
        miss = [max(lo - target_x, target_x - hi, 0.0) for lo, hi in spans]
        k = int(np.argmin(miss))
        if miss[k] > 0.05:
            raise ValueError(f"no box at target_x {target_x:.3f} m")
    gap_up = float(spans[k][0] - spans[k - 1][1]) if k > 0 else None
    gap_down = float(spans[k + 1][0] - spans[k][1]) if k + 1 < len(spans) else None
    if len(clusters[k]) < config.min_points:
        raise ValueError(f"no box in the pick zone ({len(clusters[k])} points in the selected cluster)")
    box_pts, bu, bv = box_pts[clusters[k]], bu[clusters[k]], bv[clusters[k]]

    hull = _convex_hull(box_pts[:, :2])
    (cx, cy), length, width, yaw = _min_area_rect(hull)
    height = float(np.percentile(box_pts[:, 2], config.top_percentile))
    rectangularity = _polygon_area(hull) / (length * width)

    c, s = math.cos(yaw), math.sin(yaw)
    half = [(sa * length / 2, sb * width / 2) for sa, sb in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    corners = np.array([[cx + c * a - s * b, cy + s * a + c * b] for a, b in half])
    corner_gap = float(np.max(np.min(np.linalg.norm(corners[:, None] - hull[None], axis=2), axis=1)))
    dx, dy = box_pts[:, 0] - cx, box_pts[:, 1] - cy
    along, across = c * dx + s * dy, -s * dx + c * dy
    m = config.interior_margin_m
    interior = (np.abs(along) <= length / 2 - m) & (np.abs(across) <= width / 2 - m)
    tilt, dent = 0.0, 0.0
    if interior.sum() >= 3:
        ip = box_pts[interior]
        a, b, _ = np.linalg.lstsq(np.c_[ip[:, 0], ip[:, 1], np.ones(len(ip))], ip[:, 2], rcond=None)[0]
        tilt = math.atan(math.hypot(a, b))
        dent = max(0.0, height - float(np.percentile(ip[:, 2], 2.0)))

    patch = valid[bv.min():bv.max() + 1, bu.min():bu.max() + 1]
    valid_ratio = float(patch.mean())

    e = config.edge_margin_m
    on_image_edge = bu.min() == 0 or bv.min() == 0 or bu.max() == depth.shape[1] - 1 or bv.max() == depth.shape[0] - 1
    full_view = bool(box_pts[:, 0].min() - x0 > e and x1 - box_pts[:, 0].max() > e
                     and box_pts[:, 1].min() - y0 > e and y1 - box_pts[:, 1].max() > e and not on_image_edge)
    reasons = tuple(name for name, bad in (
        ("top_dent", dent > config.max_dent_m),
        ("top_tilt", tilt > config.max_tilt_rad),
        ("crushed_corner", rectangularity < config.min_rectangularity or corner_gap > config.max_corner_gap_m),
    ) if bad) if full_view else ()
    confidence = 1.0 if valid_ratio >= config.min_valid_ratio and full_view else config.uncertain_confidence

    return DepthMeasurement(
        size=Size3D(round(length, 4), round(width, 4), round(height, 4)),
        pose=Pose3D("conveyor", float(cx), float(cy), 0.0, yaw=float(yaw)),
        tilt_rad=tilt,
        dent_depth_m=dent,
        rectangularity=float(rectangularity),
        corner_gap_m=corner_gap,
        valid_ratio=valid_ratio,
        point_count=len(box_pts),
        box_count=len(clusters),
        gap_upstream_m=gap_up,
        gap_downstream_m=gap_down,
        in_full_view=full_view,
        damage_reasons=reasons,
        confidence=confidence,
    )


def _to_conveyor(u, v, z, camera: CameraModel) -> np.ndarray:
    cam = np.stack([(u - camera.cx) / camera.fx * z, (v - camera.cy) / camera.fy * z, z, np.ones_like(z)])
    return (camera.conveyor_from_camera @ cam)[:3].T


def _convex_hull(points: np.ndarray) -> np.ndarray:
    """Monotone chain; returns the hull counter-clockwise."""
    pts = sorted(set(map(tuple, np.round(points, 5))))
    if len(pts) < 3:
        raise ValueError("box footprint is degenerate")

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and _cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(reversed(pts)))


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _polygon_area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _min_area_rect(hull: np.ndarray):
    """Rotating calipers over hull edges -> (centre, long side, short side, yaw in (-pi/2, pi/2])."""
    best = None
    edges = np.roll(hull, -1, axis=0) - hull
    for ex, ey in edges:
        theta = math.atan2(ey, ex)
        c, s = math.cos(theta), math.sin(theta)
        a = hull[:, 0] * c + hull[:, 1] * s
        b = -hull[:, 0] * s + hull[:, 1] * c
        area = (a.max() - a.min()) * (b.max() - b.min())
        if best is None or area < best[0]:
            best = (area, theta, a.min(), a.max(), b.min(), b.max())
    _, theta, a0, a1, b0, b1 = best
    c, s = math.cos(theta), math.sin(theta)
    ma, mb = (a0 + a1) / 2, (b0 + b1) / 2
    centre = (ma * c - mb * s, ma * s + mb * c)
    length, width = a1 - a0, b1 - b0
    if width > length:
        length, width, theta = width, length, theta + math.pi / 2
    yaw = math.atan2(math.sin(theta), math.cos(theta))
    if yaw <= -math.pi / 2:
        yaw += math.pi
    elif yaw > math.pi / 2:
        yaw -= math.pi
    return centre, float(length), float(width), yaw
