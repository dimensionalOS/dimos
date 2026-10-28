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

# Least gap (metres) kept between the fingertips and what the object stands on,
# and between the palm and the object's top.
_CLEARANCE = 0.005


class HeuristicGraspModuleConfig(ModuleConfig):
    # Where the gripper's parts sit along its approach, measured from the tool tip,
    # metres. Left unset, the tool tip itself goes to the object's mid-height.
    # The closed finger pads' centre, behind the tool tip.
    pad_behind_tcp: float = 0.0
    # The farthest the fingertips reach past the tool tip while closing.
    fingertip_reach: float | None = None
    # The nearest part other than the fingers (the palm), behind the tool tip.
    palm_behind_tcp: float | None = None


class HeuristicGraspModule(Module, GraspGenSpec):
    """Generate one top-down parallel-jaw grasp from a gravity-aligned point cloud.

    The input frame's XY plane must be horizontal and its -Z axis must point down.
    """

    config: HeuristicGraspModuleConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        if object_pointcloud.ts is None or not math.isfinite(float(object_pointcloud.ts)):
            raise ValueError("object pointcloud must have a finite timestamp")
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
        pose = Pose(
            Vector3(float(center_xy[0]), float(center_xy[1]), self._tip_height(low_z, high_z)),
            Quaternion.from_euler(Vector3(-math.pi, 0.0, self._narrow_axis_yaw(xy))),
        )
        return GraspCandidateArray(
            Header(float(object_pointcloud.ts), object_pointcloud.frame_id),
            [GraspCandidate(pose, score=1.0)],
        )

    def _tip_height(self, low_z: float, high_z: float) -> float:
        """Tool tip height (metres) that puts the closed pads at the object's mid-height.

        Raised if needed so the fingertips stay above the object's low end (what it
        stands on) and the palm above its high end.

        Args:
            low_z: Height of the object's low end, metres.
            high_z: Height of the object's high end, metres.
        """
        config = self.config
        # Straight down: the pads and palm sit above the tool tip, the fingertips below.
        heights = [float(low_z + high_z) / 2.0 - config.pad_behind_tcp]
        if config.fingertip_reach is not None:
            heights.append(float(low_z) + _CLEARANCE + config.fingertip_reach)
        if config.palm_behind_tcp is not None:
            heights.append(float(high_z) + _CLEARANCE - config.palm_behind_tcp)
        return max(heights)

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
