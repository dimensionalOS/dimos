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

import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.core import rpc
from dimos.manipulation.grasping.grasp_gen_spec import SceneGrasps
from dimos.manipulation.grasping.grasp_gen_x.module import (
    GraspGenXConfig,
    GraspGenXError,
    GraspGenXModule,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header

if TYPE_CHECKING:
    from .backend import GraspGenXRuntime


def _create_runtime(config: GraspGenXConfig) -> GraspGenXRuntime:
    # This import is the intentional first-use boundary for the optional GPU runtime.
    from .backend import GraspGenXRuntime

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
        poses, scores = self._infer(object_pointcloud, self.config.num_samples)
        return self._ranked(object_pointcloud, poses, scores)

    @rpc
    def propose_grasps_in_scene(
        self,
        object_pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2,
        collision_threshold: float,
        num_samples: int | None = None,
    ) -> SceneGrasps:
        samples = self.config.num_samples if num_samples is None else num_samples
        if isinstance(samples, bool) or not isinstance(samples, int | np.integer) or samples <= 0:
            raise ValueError("num_samples must be a positive whole number")
        if not self.config.gripper_body:
            raise ValueError("gripper_body is empty, so grasps cannot be checked against a scene")
        if not np.isfinite(collision_threshold) or collision_threshold <= 0.0:
            raise ValueError("collision_threshold must be a positive distance in metres")
        if scene_pointcloud.frame_id != object_pointcloud.frame_id:
            raise ValueError(
                f"scene pointcloud is in frame {scene_pointcloud.frame_id!r}, "
                f"the object pointcloud in {object_pointcloud.frame_id!r}"
            )
        scene = scene_pointcloud.points_f32().reshape((-1, 3))
        if not np.all(np.isfinite(scene)):
            raise ValueError("scene pointcloud XYZ values must be finite floats in metres")

        started = time.perf_counter()
        poses, scores = self._infer(object_pointcloud, int(samples))
        inferred = time.perf_counter()
        if len(poses) and len(scene):
            assert self._runtime is not None
            try:
                clear = self._runtime.collision_free(scene, poses, collision_threshold)
            except Exception as exc:
                raise GraspGenXError("GraspGenX scene collision check failed") from exc
        else:
            clear = np.ones(len(poses), dtype=bool)
        checked = time.perf_counter()
        top = np.argsort(-scores, kind="stable")[: self.config.max_candidates]
        return SceneGrasps(
            self._ranked(object_pointcloud, poses[clear], scores[clear]),
            removed=int(len(poses) - np.count_nonzero(clear)),
            sampled=len(poses),
            clear_in_top=int(np.count_nonzero(clear[top])),
            infer_s=inferred - started,
            check_s=checked - inferred,
        )

    def _infer(
        self, object_pointcloud: PointCloud2, num_samples: int
    ) -> tuple[np.ndarray, np.ndarray]:
        """Check an object cloud, run the model on it and check what comes back.

        Args:
            object_pointcloud: The object's points, metres.
            num_samples: How many grasps the model samples.

        Returns the model's grasp poses (N, 4, 4), in the cloud's frame and for
        GraspGenX's grasp frame rather than the TCP, and their scores (N,).
        Raises ValueError for a bad cloud or a bad model output.
        """
        if self._runtime is None:
            raise GraspGenXError("GraspGenX module has not been started")
        if object_pointcloud.ts is None:
            raise ValueError("object pointcloud must have a timestamp")
        if not object_pointcloud.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")

        points = object_pointcloud.points_f32()
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("object pointcloud must contain at least one XYZ point")
        if not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")

        try:
            poses, scores = self._runtime.infer(points, num_samples)
        except Exception as exc:
            raise GraspGenXError("GraspGenX inference failed") from exc
        scores = scores.reshape(-1)

        if poses.size == 0 and scores.size == 0:
            return np.empty((0, 4, 4)), np.empty(0)
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
        return poses, scores

    def _ranked(
        self, object_pointcloud: PointCloud2, poses: np.ndarray, scores: np.ndarray
    ) -> GraspCandidateArray:
        """The best ``max_candidates`` grasps as TCP poses, highest score first."""
        header = Header(float(object_pointcloud.ts), object_pointcloud.frame_id)
        if len(poses) == 0:
            return GraspCandidateArray(header, [])
        tcp_poses = poses @ np.asarray(self.config.grasp_frame_to_tcp)
        order = np.argsort(-scores, kind="stable")[: self.config.max_candidates]
        candidates = [
            GraspCandidate(self._pose_from_matrix(tcp_poses[index]), float(scores[index]))
            for index in order
        ]
        return GraspCandidateArray(header, candidates)

    @staticmethod
    def _pose_from_matrix(matrix: np.ndarray) -> Pose:
        return Pose(
            {
                "position": Vector3(matrix[:3, 3]),
                "orientation": Quaternion.from_rotation_matrix(matrix[:3, :3]),
            }
        )
