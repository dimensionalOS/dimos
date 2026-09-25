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

"""A pinhole camera's projection and the depth raster it draws returns into."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.agent_encode.pointcloud.queries.lib.spacing import point_spacing


def axes(
    pose: tuple[float, float, float, float, float],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Unit vectors forward, right, up in the cloud's frame for pose
    (x, y, z, yaw_deg, pitch_deg). Yaw 0 looks along +x, positive yaw turns toward +y;
    positive pitch looks up."""
    yaw = math.radians(pose[3])
    pitch = math.radians(pose[4])
    forward = np.array(
        [math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), math.sin(pitch)]
    )
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    up = np.cross(right, forward)
    return forward, right, up


def project(
    points: NDArray[np.float64],
    pose: tuple[float, float, float, float, float],
    fov_deg: float,
    size: tuple[int, int],
    max_depth: float | None,
) -> NDArray[np.float64]:
    """(K, 2) continuous pixel coordinates (u, v) of (K, 3) world points; NaN for a point
    behind the camera or beyond ``max_depth``. Pixel (i, j) spans [i, i+1) x [j, j+1)."""
    forward, right, up = axes(pose)
    focal = (size[0] / 2) / math.tan(math.radians(fov_deg) / 2)
    relative = points - np.array(pose[:3])
    distance = relative @ forward
    visible = distance > 0.05
    if max_depth is not None:
        visible &= distance <= max_depth
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = np.column_stack(
            (
                (relative @ right) / distance * focal + size[0] / 2,
                -(relative @ up) / distance * focal + size[1] / 2,
            )
        )
    uv[~visible] = np.nan
    return uv


@dataclass(frozen=True, eq=False)
class DepthRaster:
    """Which return each pixel of a perspective render shows, and how far away it is."""

    depth: NDArray[np.float32]
    """(height, width) depth along the view direction; inf where nothing was hit."""
    widths: tuple[float, float] | None
    """The smallest and largest width in metres the returns were drawn at; None when
    nothing was in view."""
    point_ids: NDArray[np.int64]
    """(height, width) index of the shown return among the drawn returns; -1 for none."""
    provenance: NDArray[np.uint8]
    """(height, width): 0 no return, 1 projected return, 2 splat, 3 filled pixel."""
    projected_uv: NDArray[np.float64]
    """(N, 2) continuous pixel of each drawn return; NaN when out of view."""


def depth_raster(
    points: NDArray[np.float32],
    pose: tuple[float, float, float, float, float],
    fov_deg: float,
    size: tuple[int, int],
    max_depth: float | None,
    point_size_m: float | None,
    max_splat_px: int = 12,
) -> DepthRaster:
    """Each return drawn as a square at its depth, as wide as the gap to its nearest
    neighbour in the cloud (or ``point_size_m`` for every return when given), so a dense
    cloud renders finely and a sparse one still closes into surfaces; the nearest return
    wins wherever squares overlap. A square reaches at most ``max_splat_px`` pixels from
    its return. A pixel no square reached takes its nearest neighbouring pixel's return."""
    width, height = size
    if any(type(n) is not int or not 1 <= n <= 2048 for n in size) or width * height > 2097152:
        raise ValueError("size must be positive integers <=2048 and <=2097152 pixels")
    if not np.isfinite([*pose, fov_deg]).all() or not 0 < fov_deg < 180:
        raise ValueError("pose must be finite and fov_deg must be between 0 and 180")
    if max_depth is not None and (not np.isfinite(max_depth) or max_depth <= 0.05):
        raise ValueError("max_depth must be finite and greater than 0.05")
    if point_size_m is not None and (not np.isfinite(point_size_m) or point_size_m < 0):
        raise ValueError("point_size_m must be finite and nonnegative")
    depth = np.full((height, width), np.inf, dtype=np.float32)
    ids = np.full((height, width), -1, dtype=np.int64)
    provenance = np.zeros((height, width), dtype=np.uint8)
    world = points.astype(np.float64)
    projected = project(world, pose, fov_deg, size, max_depth)
    keep = ~np.isnan(projected[:, 0])
    source_ids = np.flatnonzero(keep)
    if len(source_ids) == 0:
        return DepthRaster(depth, None, ids, provenance, projected)
    forward, _, _ = axes(pose)
    d = (world[keep] - np.array(pose[:3])) @ forward
    focal = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    col = np.floor(projected[keep, 0]).astype(np.int64)
    row = np.floor(projected[keep, 1]).astype(np.int64)
    width_m = point_spacing(points[keep]) if point_size_m is None else np.full(len(d), point_size_m)
    radius = np.clip(np.round(focal * width_m / d / 2.0), 0, max_splat_px).astype(np.int64)
    # Positive float32 bit order equals numeric depth order; low bits break ties by return ID.
    keys = (d.astype(np.float32).view(np.uint32).astype(np.uint64) << 32) | source_ids.astype(
        np.uint64
    )
    missing = np.iinfo(np.uint64).max
    packed = np.full(width * height, missing, dtype=np.uint64)
    for r in np.unique(radius):
        group = radius == r
        gc, gr, gkeys = col[group], row[group], keys[group]
        for dy in range(-int(r), int(r) + 1):
            rows = gr + dy
            for dx in range(-int(r), int(r) + 1):
                cols = gc + dx
                inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
                np.minimum.at(packed, rows[inside] * width + cols[inside], gkeys[inside])
    nearest = packed.reshape(height, width)
    valid = nearest != missing
    depth[valid] = (nearest[valid] >> 32).astype(np.uint32).view(np.float32)
    ids[valid] = (nearest[valid] & np.uint64(0xFFFFFFFF)).astype(np.int64)
    yy, xx = np.indices(depth.shape)
    provenance[valid] = 2
    exact = (
        valid
        & (np.floor(projected[np.maximum(ids, 0), 0]) == xx)
        & (np.floor(projected[np.maximum(ids, 0), 1]) == yy)
    )
    provenance[exact] = 1
    padded = np.pad(depth, 1, constant_values=np.inf)
    padded_ids = np.pad(ids, 1, constant_values=-1)
    filled_depth = depth.copy()
    filled_ids = ids.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            neighbour = padded[1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width]
            take = ~valid & (neighbour < filled_depth)
            filled_depth[take] = neighbour[take]
            filled_ids[take] = padded_ids[1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width][take]
    provenance[~valid & np.isfinite(filled_depth)] = 3
    return DepthRaster(
        filled_depth,
        (round(float(width_m.min()), 3), round(float(width_m.max()), 3)),
        filled_ids,
        provenance,
        projected,
    )
