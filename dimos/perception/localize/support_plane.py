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

"""RANSAC fit of the dominant horizontal support surface from window keyframes."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy import ndimage

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.type.observation import Observation
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.localize.rig import Rig

logger = setup_logger()

PLANE_DISTANCE = 0.01  # m - RANSAC inlier distance
MIN_HORIZONTAL_DOT = 0.90  # |normal . z| for a plane to count as horizontal
PLANE_INLIER_QUANTILE = 0.15  # anchor plane offset near the base of the inlier slab
PLANE_SEED = 0  # so one window always fits the same plane
MIN_SURFACE_SIDE = 0.10  # m - a narrower plane, such as a block's top, is not a surface
PIECE_CELL = 0.02  # m - inliers this close in the plane are one patch
DEDUPE_CELL = 0.01  # m - find_surfaces counts points closer than this once
STANDING_LOW, STANDING_HIGH = 0.01, 0.30  # m - over a surface, what stands on it
LEVEL_MAX_TILT_DEG = 5.0  # find_surfaces(level=True) takes nothing steeper as parallel
MAX_TILT_DEG = 25.0  # find_surfaces' default: nothing steeper is a surface
SPAN_PERCENTILES = (1, 99)  # a surface's extent, past its strays; runs about 2% short


@dataclass
class SupportPlane:
    """Plane coefficients (a, b, c, d), normal up."""

    coefficients: tuple[float, float, float, float]

    def height_above(self, points: np.ndarray) -> np.ndarray:
        a, b, c, d = self.coefficients
        heights: np.ndarray = points @ np.array([a, b, c]) + d
        return heights


def window_points(
    rig: Rig, keyframes: list[Observation[Image]], frame: str, frames: int = 5
) -> NDArray[np.float64] | None:
    """Up to ``frames`` of one camera's keyframes, spread over the list, merged at 1 cm;
    None when they hold fewer than 500 points."""
    if not keyframes:
        return None
    clouds = []
    for obs in keyframes[:: max(1, len(keyframes) // frames)][:frames]:
        cloud = rig.backdrop(obs.ts, frame)
        if cloud is not None:
            clouds.append(cloud.voxel_downsample(0.01))
    if not clouds:
        return None
    merged = clouds[0]
    for cloud in clouds[1:]:
        merged = merged + cloud
    points = np.asarray(merged.pointcloud.points)
    return points if len(points) >= 500 else None


def peel_planes(
    points: NDArray[np.float64], min_up_dot: float
) -> list[tuple[NDArray[np.float64], NDArray[np.float64]]]:
    """Up to four planes peeled off by RANSAC, biggest first as found; only those whose
    normal has ``|n . z| >= min_up_dot``, each as (a, b, c, d) normal up and its inliers."""
    import open3d as o3d

    o3d.utility.random.seed(PLANE_SEED)
    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(points)
    found: list[tuple[NDArray[np.float64], NDArray[np.float64]]] = []
    for _ in range(4):
        if len(remaining.points) < 500:
            break
        model, inlier_idx = remaining.segment_plane(
            distance_threshold=PLANE_DISTANCE, ransac_n=3, num_iterations=1000, probability=1.0
        )
        model = np.array(model)
        if abs(model[2]) >= min_up_dot:
            found.append(
                (model if model[2] >= 0 else -model, np.asarray(remaining.points)[inlier_idx])
            )
        remaining = remaining.select_by_index(inlier_idx, invert=True)
    return found


def anchored(model: NDArray[np.float64], inliers: NDArray[np.float64]) -> SupportPlane:
    """The plane through the base of its inlier slab.

    Thin objects (< 1 cm) sitting on the support surface fall inside the RANSAC
    inlier slab and bias the least-squares offset upward; anchoring the offset
    at the lower quantile of the inliers keeps the plane on the support surface.
    """
    d = -float(np.quantile(inliers @ model[:3], PLANE_INLIER_QUANTILE))
    return SupportPlane(coefficients=(float(model[0]), float(model[1]), float(model[2]), d))


def fit_support_plane(
    rig: Rig, keyframes: list[Observation[Image]], frame: str
) -> SupportPlane | None:
    """Dominant near-horizontal plane in one camera's keyframes; non-horizontal
    candidates are peeled off."""
    points = window_points(rig, keyframes, frame)
    if points is None:
        return None
    planes = peel_planes(points, MIN_HORIZONTAL_DOT)
    if not planes:
        logger.warning("support plane: no horizontal plane found in backdrop")
        return None
    model, inliers = max(planes, key=lambda plane: len(plane[1]))
    return anchored(model, inliers)


@dataclass
class Surface:
    """One flat surface, such as a table top, in the frame it was asked in.

    Attributes:
        plane: Its plane, normal up.
        height: The plane's z straight under the frame's origin, metres.
        center: The centre of its points' outline, on the plane.
        size: The outline's long side and short side, metres.
        yaw_deg: The long side's heading in the frame's xy plane.
        tilt_deg: The normal's angle from +z.
        downhill_deg: The heading the surface goes down towards.
        points: How many points it has.
        rms_m: Their spread about the plane.
        points_above: Points standing on it: inside its outline, ``STANDING_LOW`` to
            ``STANDING_HIGH`` over it.
        fitted: The plane fitted to it when ``plane`` is reported level instead;
            what stands on it is measured from this one, so a slope's own points do
            not count.
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
    """Two unit vectors in a plane with this normal: the x axis and the y axis laid onto it."""
    first = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
    if np.linalg.norm(first) < 1e-6:
        first = np.array([0.0, 1.0, 0.0]) - normal * normal[1]
    first /= np.linalg.norm(first)
    return np.stack([first, np.cross(normal, first)])


def connected_pieces(
    model: NDArray[np.float64], inliers: NDArray[np.float64]
) -> list[NDArray[np.float64]]:
    """A plane's inliers split into the patches that touch, on a ``PIECE_CELL`` grid in
    the plane: four block tops at one height are four pieces, not one surface."""
    uv = inliers @ plane_axes(np.asarray(model[:3], dtype=np.float64)).T
    cells = np.floor((uv - uv.min(axis=0)) / PIECE_CELL).astype(np.int64)
    grid = np.zeros(cells.max(axis=0) + 1, dtype=bool)
    grid[cells[:, 0], cells[:, 1]] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3), dtype=bool))
    of_point = labels[cells[:, 0], cells[:, 1]]
    return [inliers[of_point == label] for label in range(1, count + 1)]


def describe_surface(
    model: NDArray[np.float64], inliers: NDArray[np.float64], level: bool = False
) -> Surface:
    """A plane's height, extent, heading and tilt, from its inliers.

    The size and heading are those of the smallest rectangle round the inliers in
    the plane, so a square table reads square, spanning ``SPAN_PERCENTILES`` of the
    inliers along its sides. ``level`` reports the surface as parallel to the xy
    plane, at the height of its centre.
    """
    plane = anchored(model, inliers)
    normal = np.asarray(plane.coefficients[:3])
    distances = plane.height_above(inliers)
    flat = inliers - np.outer(distances, normal)
    axes = plane_axes(normal)
    uv = flat @ axes.T
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
    long_side = sides[0] if spans[0] >= spans[1] else sides[1]
    long_axis = long_side @ axes
    yaw = float(np.degrees(np.arctan2(long_axis[1], long_axis[0])))
    # A side's heading is the same as its opposite's: keep it in (-90, 90].
    yaw = yaw - 180.0 if yaw > 90.0 else yaw + 180.0 if yaw <= -90.0 else yaw
    # Out of the plane's own coordinates: the plane sits -d along its normal.
    center = center_uv @ axes - normal * plane.coefficients[3]
    fitted = None
    if level:
        fitted = plane
        plane = SupportPlane(coefficients=(0.0, 0.0, 1.0, -float(center[2])))
        normal = np.array([0.0, 0.0, 1.0])
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
    """Every flat surface at least ``MIN_SURFACE_SIDE`` wide among up to four RANSAC
    peels of ``points``, each split into the patches that touch, the biggest first.

    ``points`` are world-frame; points closer than 1 cm count once, so a surface
    seen in many frames does not outweigh one seen in a few. ``to_frame``, a 4x4
    matrix from the world frame to another, describes each surface in that frame.

    ``level`` takes every surface as parallel to that frame's xy plane, such as a
    table top square to the arm's base, and reports it at the height of its centre
    with no tilt. The fit allows ``LEVEL_MAX_TILT_DEG`` at most, so a table a degree
    or two off its base stays one surface and a ramp is no surface; past about half
    a degree a table's two ends sit a centimetre either side of the reported height
    over a metre. Otherwise the tilt is fitted, level to the world frame, up to
    ``max_tilt_deg``.
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
    """Which points lie over or under a surface's outline: within half its length along
    its long side, at ``yaw_deg``, and half its width across it."""
    along = np.array([np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg)), 0.0])
    along -= normal * (along @ normal)
    along /= max(float(np.linalg.norm(along)), 1e-9)
    offsets = np.atleast_2d(points) - center
    length, width = size
    inside: NDArray[np.bool_] = (np.abs(offsets @ along) <= length / 2) & (
        np.abs(offsets @ np.cross(normal, along)) <= width / 2
    )
    return inside


def standing_on(surface: Surface, points: NDArray[np.float64]) -> int:
    """How many of ``points`` stand on a surface: inside its outline, between
    ``STANDING_LOW`` and ``STANDING_HIGH`` over it."""
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
