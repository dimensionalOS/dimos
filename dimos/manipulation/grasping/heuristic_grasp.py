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

"""Deterministic top-down grasp proposals for segmented object point clouds."""

from __future__ import annotations

import math

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.grasping.grasp_gen_spec import HeuristicGraspSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


class HeuristicGraspModule(Module, HeuristicGraspSpec):
    """Propose one top-down parallel-jaw grasp from object-cloud geometry."""

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        """Return a center grasp with jaws aligned to the narrow horizontal cloud axis."""
        if object_pointcloud.ts is None:
            raise ValueError("object pointcloud must have a timestamp")
        if not object_pointcloud.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")
        points = object_pointcloud.points_f32()
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            raise ValueError("object pointcloud must contain at least three XYZ points")
        if not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")

        xy = points[:, :2]
        center_xy = np.median(xy, axis=0)
        low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
        yaw = self._narrow_axis_yaw(xy)
        pose = Pose(
            Vector3(float(center_xy[0]), float(center_xy[1]), float((low_z + high_z) / 2.0)),
            Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)),
        )
        return GraspCandidateArray(
            Header(float(object_pointcloud.ts), object_pointcloud.frame_id),
            [GraspCandidate(pose, score=1.0)],
        )

    @staticmethod
    def _narrow_axis_yaw(xy: np.ndarray) -> float:
        """Align the gripper closing axis with the cloud's narrow horizontal axis."""
        centered = xy - np.mean(xy, axis=0)
        covariance = centered.T @ centered
        values, vectors = np.linalg.eigh(covariance)
        if values[1] <= 0.0 or np.isclose(values[0], values[1], rtol=0.05):
            return 0.0
        narrow_axis = vectors[:, 0]
        return math.atan2(float(narrow_axis[1]), float(narrow_axis[0])) - math.pi / 2.0
