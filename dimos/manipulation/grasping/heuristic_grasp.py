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

from copy import deepcopy
import math

import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.message_utils import geometry, pointcloud, time as message_time
from dimos.messages.dimos_msgs.msg import GraspCandidate, GraspCandidateArray
from dimos.messages.geometry_msgs.msg import Point, Pose
from dimos.messages.sensor_msgs.msg import PointCloud2


class HeuristicGraspModule(Module, GraspGenSpec):
    """Generate one top-down parallel-jaw grasp from a gravity-aligned point cloud.

    The input frame's XY plane must be horizontal and its -Z axis must point down.
    """

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        try:
            stamp = message_time.to_seconds(object_pointcloud.header.stamp)
        except (AttributeError, TypeError, OverflowError) as error:
            raise ValueError("object pointcloud must have a finite timestamp") from error
        if not math.isfinite(stamp):
            raise ValueError("object pointcloud must have a finite timestamp")
        if not object_pointcloud.header.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")
        points = pointcloud.xyz(object_pointcloud).astype(np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            raise ValueError("object pointcloud must contain at least three XYZ points")
        if not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")

        xy = points[:, :2]
        center_xy = np.median(xy, axis=0)
        low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
        pose = Pose(
            Point(float(center_xy[0]), float(center_xy[1]), float((low_z + high_z) / 2.0)),
            geometry.quaternion_from_euler(-math.pi, 0.0, self._narrow_axis_yaw(xy)),
        )
        return GraspCandidateArray(
            deepcopy(object_pointcloud.header),
            [GraspCandidate(pose, score=1.0)],
        )

    @staticmethod
    def _narrow_axis_yaw(xy: NDArray[np.float32]) -> float:
        centered = xy - np.mean(xy, axis=0)
        covariance = centered.T @ centered
        values, vectors = np.linalg.eigh(covariance)
        if values[1] <= 0.0 or np.isclose(values[0], values[1], rtol=0.05):
            return 0.0
        narrow_axis = vectors[:, 0]
        yaw = math.atan2(float(narrow_axis[1]), float(narrow_axis[0])) - math.pi / 2.0
        # A parallel-jaw grasp is unchanged by a 180-degree wrist rotation.
        return (yaw + math.pi / 2.0) % math.pi - math.pi / 2.0
