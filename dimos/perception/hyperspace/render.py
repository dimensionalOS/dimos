# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Render scored voxel clouds as 2D heatmap images.

All views are score-wins orthographic projections ("x-ray heatmap"): where
voxels overlap along the view axis, the highest-scoring one is shown, so a
strong match is never hidden behind closer geometry.
"""

from __future__ import annotations

import math
from typing import Literal

import cv2
import numpy as np
from numpy.typing import NDArray

ViewName = Literal["top", "side", "front", "isometric"]

VIEWS: tuple[str, ...] = ("top", "side", "front", "isometric")

_GRAY = np.array([130.0, 130.0, 130.0])
_RED = np.array([255.0, 30.0, 30.0])

_ISO_AZIMUTH_RAD = math.radians(45.0)
_ISO_ELEVATION_RAD = math.radians(35.264)  # classic isometric tilt


def normalize_scores(
    scores: NDArray[np.floating],
    p_lo: float = 50.0,
    p_hi: float = 99.5,
    gamma: float = 2.0,
) -> NDArray[np.float32]:
    """Percentile-normalize scores to [0, 1] for coloring.

    Patch-token cosine scores are only meaningful relatively and their
    distribution is narrow, so the scale spans the upper half of the current
    map's distribution and ``gamma`` suppresses middling scores — only the
    strongest matches saturate to red.
    """
    scores = np.asarray(scores, dtype=np.float32)
    if scores.size == 0:
        return scores
    lo, hi = np.percentile(scores, [p_lo, p_hi])
    if hi - lo < 1e-9:
        return np.zeros_like(scores)
    norm = np.clip((scores - lo) / (hi - lo), 0.0, 1.0) ** gamma
    return np.asarray(norm, dtype=np.float32)


def scores_to_colors(normalized: NDArray[np.floating]) -> NDArray[np.uint8]:
    """(N,) scores in [0, 1] -> (N, 3) RGB, gray for low, red for high."""
    t = np.asarray(normalized, dtype=np.float32).reshape(-1, 1)
    return np.asarray(_GRAY + (_RED - _GRAY) * t, dtype=np.uint8)


def _view_coords(
    centers: NDArray[np.floating], view: str
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Map world centers to 2D view-plane coords (u right, v up), in meters."""
    c = np.asarray(centers, dtype=np.float64)
    if view == "top":
        return c[:, 0], c[:, 1]
    if view == "front":  # looking along +x
        return c[:, 1], c[:, 2]
    if view == "side":  # looking along +y
        return c[:, 0], c[:, 2]
    if view == "isometric":
        ca, sa = math.cos(_ISO_AZIMUTH_RAD), math.sin(_ISO_AZIMUTH_RAD)
        ce, se = math.cos(_ISO_ELEVATION_RAD), math.sin(_ISO_ELEVATION_RAD)
        x = c[:, 0] * ca - c[:, 1] * sa
        y = c[:, 0] * sa + c[:, 1] * ca
        return x, c[:, 2] * ce + y * se
    raise ValueError(f"unknown view {view!r}; expected one of {VIEWS}")


def render_view(
    centers: NDArray[np.floating],
    scores: NDArray[np.floating],
    view: str = "top",
    voxel_size: float = 0.1,
    out_px: int = 900,
    background: tuple[int, int, int] = (15, 15, 20),
) -> NDArray[np.uint8]:
    """Render voxel centers + scores into an RGB heatmap image.

    One voxel maps to one cell at native resolution (score-wins per cell),
    then the image is nearest-upscaled to roughly ``out_px``.
    """
    if len(centers) == 0:
        return np.full((out_px, out_px, 3), background, dtype=np.uint8)

    u, v = _view_coords(centers, view)
    cell = voxel_size  # native grid: 1 cell per voxel edge
    iu = np.floor(u / cell).astype(np.int64)
    iv = np.floor(v / cell).astype(np.int64)
    iu -= iu.min()
    iv -= iv.min()
    w = int(iu.max()) + 1
    h = int(iv.max()) + 1

    score_flat = np.full(h * w, -np.inf, dtype=np.float32)
    flat = (h - 1 - iv) * w + iu  # v up -> image row down
    np.maximum.at(score_flat, flat, np.asarray(scores, dtype=np.float32))
    score_img = score_flat.reshape(h, w)
    occupied = np.isfinite(score_img)

    norm = np.zeros((h, w), dtype=np.float32)
    if occupied.any():
        norm[occupied] = normalize_scores(score_img[occupied])

    img = np.full((h, w, 3), background, dtype=np.uint8)
    img[occupied] = scores_to_colors(norm[occupied])

    scale = max(1, round(out_px / max(h, w)))
    resized = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    return np.asarray(resized, dtype=np.uint8)


def render_nearby(
    centers: NDArray[np.floating],
    scores: NDArray[np.floating],
    robot_xy: tuple[float, float],
    robot_yaw_rad: float,
    heading_rad: float,
    radius_m: float,
    voxel_size: float = 0.1,
    out_px: int = 700,
    background: tuple[int, int, int] = (15, 15, 20),
) -> NDArray[np.uint8]:
    """Top-down heatmap of the voxels inside the query cylinder, with arrows.

    ``heading_rad`` is the world-frame azimuth of the best-scoring direction.
    Draws the cylinder boundary, a short robot-heading arrow, and a long
    arrow toward the best direction.
    """
    centers = np.asarray(centers, dtype=np.float64)
    scores = np.asarray(scores, dtype=np.float32)
    rx, ry = robot_xy

    pad = radius_m * 1.15
    scale = out_px / (2 * pad)
    img = np.full((out_px, out_px, 3), background, dtype=np.uint8)

    def to_px(x: NDArray[np.float64], y: NDArray[np.float64]) -> tuple[np.ndarray, np.ndarray]:
        px = np.round((x - rx + pad) * scale).astype(np.int64)
        py = np.round((pad - (y - ry)) * scale).astype(np.int64)
        return px, py

    if len(centers):
        d = np.hypot(centers[:, 0] - rx, centers[:, 1] - ry)
        inside = d <= radius_m
        if inside.any():
            sel = centers[inside]
            norm = normalize_scores(scores[inside])
            order = np.argsort(norm)  # score-wins: draw high scores last
            colors = scores_to_colors(norm[order])
            px, py = to_px(sel[order, 0], sel[order, 1])
            r = max(1, int(voxel_size * scale / 2))
            ok = (px >= 0) & (px < out_px) & (py >= 0) & (py < out_px)
            for (x0, y0), c in zip(np.stack([px[ok], py[ok]], axis=1), colors[ok], strict=True):
                cv2.circle(img, (int(x0), int(y0)), r, tuple(int(v) for v in c), -1)

    center_px = (out_px // 2, out_px // 2)
    cv2.circle(img, center_px, int(radius_m * scale), (90, 90, 110), 2)

    def arrow(
        angle_rad: float, length_m: float, color: tuple[int, int, int], thickness: int
    ) -> None:
        tip = (
            int(center_px[0] + math.cos(angle_rad) * length_m * scale),
            int(center_px[1] - math.sin(angle_rad) * length_m * scale),
        )
        cv2.arrowedLine(img, center_px, tip, color, thickness, tipLength=0.18)

    arrow(robot_yaw_rad, radius_m * 0.35, (120, 200, 255), 2)  # current heading (blue)
    arrow(heading_rad, radius_m * 0.9, (0, 255, 80), 4)  # best direction (green)
    return img
