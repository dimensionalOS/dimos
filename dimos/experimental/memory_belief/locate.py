# Copyright 2025-2026 Dimensional Inc.
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

"""Giving a 2D detection a place in the world, or refusing to.

``world_from_camera`` is the camera's own pose, already in the world frame. A
``camera_pose_in_world`` helper stood here that composed a robot pose with a
static mount; every caller was passing a frame pose the recorder had *already*
resolved through tf as ``world <- camera_optical``, so the mount went in twice
and every detection landed as though the camera had been yawed 90 degrees.

**Refusing is a result.** Too few lidar returns in the box gets ``None``, not a
guessed depth: everything downstream treats a present position as evidence.

**Extent, not a centroid.** A bottle is *on* a desk when its footprint sits above
the desk's top surface, which a point cannot support -- so placements carry an
axis-aligned extent and callers derive relations geometrically.

Box depth is the median of the returns inside it, and returns beyond
``depth_band_m`` are dropped first; without that a chair framed against a far
wall measures metres deep.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from collections.abc import Sequence

#: Where in a box's depth distribution the foreground surface sits. A box frames
#: an object in front of its background, so the near end is the object; the
#: median lands in the empty air between the two when they are of similar size.
SEED_PERCENTILE = 25.0


@dataclass(frozen=True, slots=True)
class Placement:
    """Where a detection is, and how well supported that claim is."""

    position: tuple[float, float, float]
    """Centroid of the inlier returns, in the frame the points arrived in."""

    extent: tuple[float, float, float]
    """Axis-aligned size of the inlier returns. Zero on a single return."""

    support: int
    """Inlier count. A placement built from three returns is not the same claim
    as one built from three hundred, and callers that treat them alike will be
    confidently wrong about small or distant objects."""

    depth_m: float
    """Median range from the camera, kept because it bounds the angular error:
    a box a pixel wide is a fine position at 1 m and a poor one at 10 m."""


class PinholeFisheye:
    """Projection for a camera whose distortion model is ``equidistant``.

    Handles the plain pinhole case too -- a zero distortion vector reduces the
    fisheye model to it -- so callers do not branch on which camera they have.
    """

    def __init__(self, camera_info: Any) -> None:
        k = np.asarray(camera_info.K, float).reshape(3, 3)
        self.K = k
        self.D = np.asarray(getattr(camera_info, "D", []) or [0.0, 0.0, 0.0, 0.0], float)[:4]
        self.width = int(camera_info.width)
        self.height = int(camera_info.height)
        self.model = str(getattr(camera_info, "distortion_model", "") or "")

    def project(
        self, points_cam: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], NDArray[np.bool_]]:
        """Project camera-frame points to pixels.

        Returns ``(uv, keep)`` where ``keep`` marks points that are in front of
        the camera and land inside the image. Points behind the camera are
        dropped before projection: a fisheye model will happily produce pixel
        coordinates for them, and they land in plausible-looking places.
        """
        import cv2

        pts = np.asarray(points_cam, np.float64).reshape(-1, 3)
        in_front = pts[:, 2] > 0.05
        uv = np.full((len(pts), 2), np.nan)
        if not in_front.any():
            return uv, np.zeros(len(pts), bool)

        usable = pts[in_front].reshape(-1, 1, 3)
        zero = np.zeros(3)
        if self.model == "equidistant":
            proj, _ = cv2.fisheye.projectPoints(usable, zero, zero, self.K, self.D)
        else:
            proj, _ = cv2.projectPoints(usable, zero, zero, self.K, np.zeros(5))
        uv[in_front] = proj.reshape(-1, 2)

        keep = in_front.copy()
        good = ~np.isnan(uv).any(axis=1)
        keep &= good
        keep[good] &= (
            (uv[good, 0] >= 0)
            & (uv[good, 0] < self.width)
            & (uv[good, 1] >= 0)
            & (uv[good, 1] < self.height)
        )
        return uv, keep


def locate_detections(
    detections: Sequence[Any],
    points_world: NDArray[np.float64],
    *,
    world_from_camera: Any,
    camera: PinholeFisheye,
    min_points: int = 8,
    depth_band_m: float = 0.6,
) -> list[Placement | None]:
    """Place each detection in the world, or return ``None`` for that detection.

    ``points_world`` is ``(N, 3)`` in the world frame and ``world_from_camera``
    maps back via its ``inverse()``. Below ``min_points`` inliers a detection
    gets no position; returns further than ``depth_band_m`` from the box's
    median depth are dropped as background. A segmentation mask is preferred
    when the detection carries one -- a box around a chair contains a lot of
    floor.
    """
    out: list[Placement | None] = [None] * len(detections)
    pts = np.asarray(points_world, np.float64).reshape(-1, 3)
    if not len(pts) or not len(detections):
        return out

    cam_from_world = np.asarray(world_from_camera.inverse().to_matrix(), float)
    homo = np.hstack([pts, np.ones((len(pts), 1))])
    cam_pts = (cam_from_world @ homo.T).T[:, :3]

    uv, keep = camera.project(cam_pts)
    if not keep.any():
        return out

    vis_uv = uv[keep]
    vis_world = pts[keep]
    vis_depth = cam_pts[keep, 2]

    for i, det in enumerate(detections):
        bbox = getattr(det, "bbox", None)
        if bbox is None:
            continue
        x1, y1, x2, y2 = (float(v) for v in bbox)
        inside = (
            (vis_uv[:, 0] >= x1)
            & (vis_uv[:, 0] <= x2)
            & (vis_uv[:, 1] >= y1)
            & (vis_uv[:, 1] <= y2)
        )
        mask = getattr(det, "mask", None)
        if mask is not None and np.ndim(mask) == 2 and np.any(inside):
            m = np.asarray(mask)
            rows = np.clip(vis_uv[:, 1].astype(int), 0, m.shape[0] - 1)
            cols = np.clip(vis_uv[:, 0].astype(int), 0, m.shape[1] - 1)
            on_mask = m[rows, cols] > 0
            # Counted inside the box, not across the frame. `on_mask` covers
            # every visible return, so a mask with enough hits elsewhere passed
            # this guard while leaving the box nearly empty -- and the narrowed
            # selection then fell under `min_points` and dropped a detection the
            # box alone would have placed. That is the discard this is meant to
            # prevent, not cause.
            narrowed = inside & on_mask
            if narrowed.sum() >= min_points:
                inside = narrowed

        n = int(inside.sum())
        if n < min_points:
            continue

        depths = vis_depth[inside]
        # Seeded at a low percentile rather than the median. A box frames an
        # object in front of its background, so the foreground surface is the
        # near end of the depth distribution -- and when the two are of similar
        # size, the median lands in the empty air between them and excludes both.
        seed = float(np.percentile(depths, SEED_PERCENTILE))
        near = inside.copy()
        near[inside] = np.abs(depths - seed) <= depth_band_m
        if int(near.sum()) < min_points:
            continue

        sel = vis_world[near]
        centre = sel.mean(axis=0)
        size = sel.max(axis=0) - sel.min(axis=0)
        out[i] = Placement(
            position=(float(centre[0]), float(centre[1]), float(centre[2])),
            extent=(float(size[0]), float(size[1]), float(size[2])),
            support=int(near.sum()),
            depth_m=float(np.median(vis_depth[near])),
        )
    return out
