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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

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
) -> np.ndarray | None:
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


def peel_planes(points: np.ndarray, min_up_dot: float) -> list[tuple[np.ndarray, np.ndarray]]:
    """Up to four planes peeled off by RANSAC, biggest first as found; only those whose
    normal has ``|n . z| >= min_up_dot``, each as (a, b, c, d) normal up and its inliers."""
    import open3d as o3d

    o3d.utility.random.seed(PLANE_SEED)
    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(points)
    found: list[tuple[np.ndarray, np.ndarray]] = []
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


def anchored(model: np.ndarray, inliers: np.ndarray) -> SupportPlane:
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

    ``height`` is the surface's z straight under the frame's origin; ``center`` and
    ``size`` (long side, short side) come from its points, ``yaw`` is the long side's
    heading. ``tilt_deg`` is the normal's angle from +z, and the surface goes down
    towards ``downhill_deg``, a heading in the frame's xy plane.
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
        }


def describe_surface(model: np.ndarray, inliers: np.ndarray) -> Surface:
    """A plane's height, extent, heading and tilt, from its inliers."""
    plane = anchored(model, inliers)
    a, b, c, d = plane.coefficients
    normal = np.array([a, b, c])
    distances = plane.height_above(inliers)
    # The inliers flattened onto the plane; the 1st to 99th percentile spans ignore strays and run about 2% short.
    flat = inliers - np.outer(distances, normal)
    middle = np.median(flat, axis=0)
    in_plane = np.eye(3) - np.outer(normal, normal)
    axes = np.linalg.svd((flat - middle) @ in_plane, full_matrices=False)[2]
    spans = []
    center = middle.copy()
    for axis in axes[:2]:
        low, high = np.percentile((flat - middle) @ axis, [1, 99])
        spans.append(float(high - low))
        center += axis * (low + high) / 2
    long_axis = axes[0] if spans[0] >= spans[1] else axes[1]
    yaw = float(np.degrees(np.arctan2(long_axis[1], long_axis[0])))
    # A side's heading is the same as its opposite's: keep it in (-90, 90].
    yaw = yaw - 180.0 if yaw > 90.0 else yaw + 180.0 if yaw <= -90.0 else yaw
    return Surface(
        plane=plane,
        height=-d / c if abs(c) > 1e-6 else float("nan"),
        center=(float(center[0]), float(center[1]), float(center[2])),
        size=(max(spans), min(spans)),
        yaw_deg=yaw,
        tilt_deg=float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))),
        downhill_deg=float(np.degrees(np.arctan2(b, a))),
        points=len(inliers),
        rms_m=float(np.sqrt(np.mean((distances - np.median(distances)) ** 2))),
    )


def level_planes(points: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Up to four planes parallel to the points' xy plane, densest first, each as
    (0, 0, 1, -z) and its inliers: the heights where most points sit within
    ``PLANE_DISTANCE``, peeled off one after another."""
    found: list[tuple[np.ndarray, np.ndarray]] = []
    remaining = np.asarray(points, dtype=np.float64)
    for _ in range(4):
        if len(remaining) < 500:
            break
        z = remaining[:, 2]
        edges = np.arange(z.min(), z.max() + 2 * PLANE_DISTANCE, PLANE_DISTANCE)
        counts, edges = np.histogram(z, edges)
        # Two neighbouring bins: one slab PLANE_DISTANCE either side of their shared edge.
        pairs = counts[:-1] + counts[1:] if len(counts) > 1 else counts
        best = int(np.argmax(pairs))
        height = float(edges[best + 1] if len(counts) > 1 else edges[0])
        inside = np.abs(z - height) <= PLANE_DISTANCE
        height = float(np.median(z[inside]))
        inside = np.abs(z - height) <= PLANE_DISTANCE
        found.append((np.array([0.0, 0.0, 1.0, -height]), remaining[inside]))
        remaining = remaining[~inside]
    return found


def find_surfaces(
    points: np.ndarray,
    max_tilt_deg: float = 25.0,
    to_frame: np.ndarray | None = None,
    level: bool = False,
) -> list[Surface]:
    """Every flat surface at least ``MIN_SURFACE_SIDE`` wide among up to four peels of
    ``points``, the biggest first.

    ``points`` are world-frame. ``to_frame``, a 4x4 matrix from the world frame to
    another, describes each surface in that frame.

    ``level`` takes every surface as parallel to that frame's xy plane, such as a
    table top square to the arm's base: only heights are fitted, so a few points
    off the plane cannot tilt it, and ``max_tilt_deg`` does not apply. Otherwise a
    RANSAC fit takes surfaces tilted at most ``max_tilt_deg`` from level to the
    world frame.
    """
    if level:
        if to_frame is not None:
            points = points @ to_frame[:3, :3].T + to_frame[:3, 3]
        planes = level_planes(points)
    else:
        planes = []
        for model, inliers in peel_planes(points, float(np.cos(np.radians(max_tilt_deg)))):
            if to_frame is not None:
                model = np.append(to_frame[:3, :3] @ model[:3], 0.0)
                inliers = inliers @ to_frame[:3, :3].T + to_frame[:3, 3]
            planes.append((model, inliers))
    surfaces = [describe_surface(model, inliers) for model, inliers in planes]
    surfaces = [surface for surface in surfaces if surface.size[1] >= MIN_SURFACE_SIDE]
    return sorted(surfaces, key=lambda surface: -surface.points)
