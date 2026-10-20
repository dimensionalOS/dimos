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

"""Deterministic grasp proposals for segmented object point clouds."""

from __future__ import annotations

import math

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header

# Fraction of the XY points, farthest from the median, dropped as depth noise before the fit.
_TRIM_QUANTILE = 0.02


class HeuristicGraspConfig(ModuleConfig):
    # Widest object the open jaws close on, metres; the xArm gripper opens to 88.9 mm.
    max_jaw_span: float = 0.085
    # Least depth below the object's top the grasp goes, metres, when little more than the
    # top was seen.
    grasp_depth: float = 0.02


class HeuristicGraspModule(Module, GraspGenSpec):
    """Top-down parallel-jaw grasps from a gravity-aligned point cloud.

    The input frame's XY plane must be horizontal and its -Z axis must point down.
    """

    config: HeuristicGraspConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        return top_down_grasps(object_pointcloud, self.config.max_jaw_span, self.config.grasp_depth)


def top_down_grasps(
    object_pointcloud: PointCloud2, max_jaw_span: float, grasp_depth: float = 0.02
) -> GraspCandidateArray:
    """Grasps straight down at the centre of the cloud's footprint, half way up the surfaces
    seen and at least ``grasp_depth`` below the top: a camera above an object may see only its
    top, and a grasp there closes on nothing. The footprint is the smallest rectangle around the XY points, so its sides follow
    the object's faces at any yaw. The first candidate closes the jaws across the short side;
    a second closes them across the long side when the jaws open that wide."""
    if object_pointcloud.ts is None or not math.isfinite(float(object_pointcloud.ts)):
        raise ValueError("object pointcloud must have a finite timestamp")
    if not object_pointcloud.frame_id:
        raise ValueError("object pointcloud frame_id must not be empty")
    points = object_pointcloud.points_f32()
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("object pointcloud must contain at least three XYZ points")
    if not np.all(np.isfinite(points)):
        raise ValueError("object pointcloud XYZ values must be finite floats in metres")

    (cx, cy), sides, short_yaw = _footprint(points[:, :2])
    low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
    position = Vector3(cx, cy, float(min((low_z + high_z) / 2.0, high_z - grasp_depth)))
    candidates = [GraspCandidate(_top_down(position, short_yaw), score=1.0)]
    if max(sides) <= max_jaw_span and not math.isclose(sides[0], sides[1], rel_tol=0.05):
        candidates.append(GraspCandidate(_top_down(position, short_yaw + math.pi / 2), score=0.5))
    return GraspCandidateArray(
        Header(float(object_pointcloud.ts), object_pointcloud.frame_id), candidates
    )


def _footprint(xy: NDArray[np.float32]) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Centre, side lengths and the short side's direction (radians) of the smallest
    rectangle around ``xy``, with the outermost points dropped as depth noise."""
    # Trimmed by distance from the median, not per axis: a per-axis cut clips a turned
    # object's corners unevenly and tilts the rectangle.
    distance = np.linalg.norm(xy - np.median(xy, axis=0), axis=1)
    kept = xy[distance <= np.quantile(distance, 1.0 - _TRIM_QUANTILE)]
    if len(kept) < 3:
        kept = xy
    corners = cv2.boxPoints(cv2.minAreaRect(kept.astype(np.float32)))
    edges = [corners[1] - corners[0], corners[2] - corners[1]]
    lengths = [float(np.hypot(*edge)) for edge in edges]
    short = edges[int(np.argmin(lengths))]
    centre = corners.mean(axis=0)
    return (
        (float(centre[0]), float(centre[1])),
        (min(lengths), max(lengths)),
        math.atan2(float(short[1]), float(short[0])),
    )


def _top_down(position: Vector3, jaw_yaw: float) -> Pose:
    """A TCP pose pointing straight down whose jaws close along ``jaw_yaw``. Flipped to point
    down, the TCP's y axis (the jaw axis) lies a quarter turn from its yaw; a parallel-jaw grasp
    is unchanged by a half turn of the wrist, so the yaw is folded into [-90, 90) degrees."""
    yaw = jaw_yaw % math.pi - math.pi / 2.0
    return Pose(position, Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)))
