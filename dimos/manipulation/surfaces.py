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

"""Flat surfaces, such as a table top, in a point cloud."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage
from scipy.spatial import cKDTree

from dimos.perception.localize.support_plane import SupportPlane, anchored, peel_planes

MIN_SURFACE_SIDE = 0.10  # m - a narrower plane, such as a block's top, is not a surface
PIECE_CELL = 0.02  # m - inliers this close in the plane are one patch, at least
PIECE_SPACINGS = 1.5  # ...or this many of the cloud's own point spacings, for sparse clouds
DEDUPE_CELL = 0.01  # m - points closer than this count once
STANDING_LOW, STANDING_HIGH = 0.01, 0.30  # m - over a surface, what stands on it
LEVEL_MAX_TILT_DEG = 5.0  # level mode takes nothing steeper as parallel
MAX_TILT_DEG = 25.0  # nothing steeper is a surface
SPAN_PERCENTILES = (1, 99)  # a surface's extent past its strays; runs about 2% short


@dataclass
class Surface:
    """One flat surface, in the frame it was asked in.

    Attributes:
        plane: Its plane, normal up.
        height: The plane's z under the frame's origin, metres.
        center: The centre of its outline, on the plane.
        size: The outline's long and short side, metres.
        yaw_deg: The long side's heading.
        tilt_deg: The normal's angle from +z.
        downhill_deg: The heading it slopes down towards.
        points: How many points it has.
        rms_m: Their spread about the plane.
        points_above: Points standing on it, inside its outline.
        fitted: The fitted plane, when ``plane`` is reported level instead.
    """

    plane: SupportPlane
    height: float
    center: tuple[float, float, float]
    size: tuple[float, float]
    yaw_deg: float
    tilt_deg: float
    downhill_deg: float
    points: int
    rms_m: float
    points_above: int = 0
    fitted: SupportPlane | None = None

    @property
    def normal(self) -> tuple[float, float, float]:
        a, b, c, _ = self.plane.coefficients
        return (a, b, c)

    def as_dict(self) -> dict[str, Any]:
        return {
            "height": self.height,
            "center": list(self.center),
            "size": list(self.size),
            "yaw_deg": self.yaw_deg,
            "normal": list(self.normal),
            "tilt_deg": self.tilt_deg,
            "downhill_deg": self.downhill_deg,
            "coefficients": list(self.plane.coefficients),
            "points": self.points,
            "rms_m": self.rms_m,
            "points_above": self.points_above,
        }


def plane_axes(normal: NDArray[np.float64]) -> NDArray[np.float64]:
    """Two unit vectors in the plane: the x axis, else the y axis, laid onto it, and their cross."""
    first = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
    if np.linalg.norm(first) < 1e-6:
        first = np.array([0.0, 1.0, 0.0]) - normal * normal[1]
    first /= np.linalg.norm(first)
    return np.stack([first, np.cross(normal, first)])


def point_spacing(points: NDArray[np.float64]) -> float:
    """The median distance from a point to its nearest neighbour, such as a lidar map's grid."""
    if len(points) < 2:
        return 0.0
    distances, _ = cKDTree(points).query(points, k=2)
    return float(np.median(distances[:, 1]))


def connected_pieces(
    model: NDArray[np.float64], inliers: NDArray[np.float64]
) -> list[NDArray[np.float64]]:
    """A plane's inliers split into touching patches, so scattered block tops are not one surface."""
    uv = inliers @ plane_axes(np.asarray(model[:3], dtype=np.float64)).T
    cell = max(PIECE_CELL, PIECE_SPACINGS * point_spacing(uv))
    cells = np.floor((uv - uv.min(axis=0)) / cell).astype(np.int64)
    grid = np.zeros(cells.max(axis=0) + 1, dtype=bool)
    grid[cells[:, 0], cells[:, 1]] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3), dtype=bool))
    of_point = labels[cells[:, 0], cells[:, 1]]
    return [inliers[of_point == label] for label in range(1, count + 1)]


def describe_surface(
    model: NDArray[np.float64], inliers: NDArray[np.float64], level: bool = False
) -> Surface:
    """A plane's height, outline and tilt; ``level`` reports it flat at its centre's height.

    The outline is the smallest rectangle round the inliers, so a square table reads square.
    """
    import cv2  # heavy: only where it is used

    plane = anchored(model, inliers)
    normal = np.asarray(plane.coefficients[:3])
    distances = plane.height_above(inliers)
    axes = plane_axes(normal)
    uv = (inliers - np.outer(distances, normal)) @ axes.T
    middle = np.median(uv, axis=0)
    (_, _), (_, _), angle = cv2.minAreaRect((uv - middle).astype(np.float32))
    turn = np.radians(angle)
    sides = np.array([[np.cos(turn), np.sin(turn)], [-np.sin(turn), np.cos(turn)]])
    spans = []
    center_uv = middle.copy()
    for side in sides:
        low, high = np.percentile((uv - middle) @ side, SPAN_PERCENTILES)
        spans.append(float(high - low))
        center_uv += side * (low + high) / 2
    long_axis = (sides[0] if spans[0] >= spans[1] else sides[1]) @ axes
    yaw = float(np.degrees(np.arctan2(long_axis[1], long_axis[0])))
    # A side's heading is its opposite's too: keep it in (-90, 90].
    yaw = yaw - 180.0 if yaw > 90.0 else yaw + 180.0 if yaw <= -90.0 else yaw
    # Back from plane coordinates: the plane sits -d along its normal.
    center = center_uv @ axes - normal * plane.coefficients[3]
    fitted = None
    if level:
        fitted = plane
        plane = SupportPlane(coefficients=(0.0, 0.0, 1.0, -float(center[2])))
        distances = plane.height_above(inliers)
    a, b, c, d = plane.coefficients
    return Surface(
        plane=plane,
        height=-d / c if abs(c) > 1e-6 else float("nan"),
        center=(float(center[0]), float(center[1]), float(center[2])),
        size=(max(spans), min(spans)),
        yaw_deg=yaw,
        tilt_deg=float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))),
        downhill_deg=float(np.degrees(np.arctan2(b, a))) if not level else 0.0,
        points=len(inliers),
        rms_m=float(np.sqrt(np.mean((distances - np.median(distances)) ** 2))),
        fitted=fitted,
    )


def find_surfaces(
    points: NDArray[np.float64],
    max_tilt_deg: float = MAX_TILT_DEG,
    to_frame: NDArray[np.float64] | None = None,
    level: bool = False,
) -> list[Surface]:
    """The flat surfaces in world-frame ``points``, the biggest first.

    ``to_frame`` (4x4, world to another frame) reports them in that frame. ``level``
    takes them as parallel to its xy plane, within ``LEVEL_MAX_TILT_DEG``; otherwise
    their tilt is fitted, up to ``max_tilt_deg``.
    """
    points = np.asarray(points, dtype=np.float64)
    _, first = np.unique(np.floor(points / DEDUPE_CELL).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(first)]
    if level and to_frame is not None:
        points = points @ to_frame[:3, :3].T + to_frame[:3, 3]
    if level:
        max_tilt_deg = min(max_tilt_deg, LEVEL_MAX_TILT_DEG)
    surfaces = []
    for model, inliers in peel_planes(points, float(np.cos(np.radians(max_tilt_deg)))):
        if to_frame is not None and not level:
            model = np.append(to_frame[:3, :3] @ model[:3], 0.0)
            inliers = inliers @ to_frame[:3, :3].T + to_frame[:3, 3]
        for piece in connected_pieces(model, inliers):
            surface = describe_surface(model, piece, level)
            if surface.size[1] >= MIN_SURFACE_SIDE:
                surfaces.append(surface)
    if to_frame is not None and not level:
        points = points @ to_frame[:3, :3].T + to_frame[:3, 3]
    surfaces = [replace(surface, points_above=standing_on(surface, points)) for surface in surfaces]
    return sorted(surfaces, key=lambda surface: -surface.points)


def in_outline(
    center: NDArray[np.float64],
    normal: NDArray[np.float64],
    yaw_deg: float,
    size: tuple[float, float],
    points: NDArray[np.float64],
) -> NDArray[np.bool_]:
    """Which points lie over or under a surface's outline."""
    # The in-plane direction whose heading is yaw_deg: its z from the plane equation.
    x, y = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    z = -(normal[0] * x + normal[1] * y) / normal[2] if abs(normal[2]) > 1e-6 else 0.0
    along = np.array([x, y, z])
    along /= np.linalg.norm(along)
    offsets = np.atleast_2d(points) - center
    length, width = size
    inside: NDArray[np.bool_] = (np.abs(offsets @ along) <= length / 2) & (
        np.abs(offsets @ np.cross(normal, along)) <= width / 2
    )
    return inside


def standing_on(surface: Surface, points: NDArray[np.float64]) -> int:
    """How many points stand on a surface: in its outline, STANDING_LOW to STANDING_HIGH over it."""
    above = (surface.fitted or surface.plane).height_above(points)
    near = (above > STANDING_LOW) & (above < STANDING_HIGH)
    inside = in_outline(
        np.asarray(surface.center),
        np.asarray(surface.normal),
        surface.yaw_deg,
        surface.size,
        points[near],
    )
    return int(np.count_nonzero(inside))
