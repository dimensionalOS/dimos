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

"""Concrete grasp proposal implementation, loaded only in the isolated process."""

from __future__ import annotations

from copy import deepcopy
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.core import rpc
from dimos.manipulation.grasping.grasp_gen_x.module import (
    GraspGenXConfig,
    GraspGenXError,
    GraspGenXModule,
)
from dimos.message_utils import geometry, pointcloud
from dimos.messages.dimos_msgs.msg import GraspCandidate, GraspCandidateArray
from dimos.messages.geometry_msgs.msg import Pose
from dimos.messages.sensor_msgs.msg import PointCloud2

if TYPE_CHECKING:
    from graspgenx_runtime.backend import GraspGenXRuntime


def _create_runtime(config: GraspGenXConfig) -> GraspGenXRuntime:
    # This import is the intentional first-use boundary for the optional GPU runtime.
    from graspgenx_runtime.backend import GraspGenXRuntime

    return GraspGenXRuntime(config)


class GraspGenXRuntimeModule(GraspGenXModule):
    """Direct adapter whose optional runtime is loaded synchronously by ``start``."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime: GraspGenXRuntime | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self._runtime is not None:
            return
        try:
            self._runtime = _create_runtime(self.config)
        except Exception as exc:
            raise GraspGenXError("failed to initialize GraspGenX") from exc

    @rpc
    def stop(self) -> None:
        self._runtime = None
        super().stop()

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        if self._runtime is None:
            raise GraspGenXError("GraspGenX module has not been started")
        if not object_pointcloud.header.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")

        points = pointcloud.xyz(object_pointcloud).astype(np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("object pointcloud must contain at least one XYZ point")
        if not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")

        try:
            poses, scores = self._runtime.infer(points, self.config.num_samples)
        except Exception as exc:
            raise GraspGenXError("GraspGenX inference failed") from exc
        scores = scores.reshape(-1)

        if poses.size == 0 and scores.size == 0:
            return GraspCandidateArray(
                deepcopy(object_pointcloud.header),
                [],
            )
        if poses.shape != (len(scores), 4, 4):
            raise ValueError("backend poses must have shape (N, 4, 4)")
        if not np.all(np.isfinite(poses)) or not np.all(np.isfinite(scores)):
            raise ValueError("backend returned non-finite poses or scores")
        if not np.allclose(poses[:, 3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-7):
            raise ValueError("backend poses must be homogeneous")
        rotations = poses[:, :3, :3]
        if not np.allclose(np.einsum("nij,nkj->nik", rotations, rotations), np.eye(3), atol=1e-5):
            raise ValueError("backend poses must have orthonormal rotations")
        if not np.allclose(np.linalg.det(rotations), 1.0, atol=1e-5):
            raise ValueError("backend poses must have proper rotations")

        tcp_poses = poses @ np.asarray(self.config.grasp_frame_to_tcp)
        order = np.argsort(-scores, kind="stable")[: self.config.max_candidates]
        candidates = [
            GraspCandidate(self._pose_from_matrix(tcp_poses[index]), float(scores[index]))
            for index in order
        ]
        return GraspCandidateArray(
            deepcopy(object_pointcloud.header),
            candidates,
        )

    @staticmethod
    def _pose_from_matrix(matrix: np.ndarray) -> Pose:
        return geometry.pose_from_matrix(matrix)
