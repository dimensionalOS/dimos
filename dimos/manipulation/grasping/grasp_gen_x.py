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

"""Import-safe DimOS adapter for GraspGenX grasp proposals."""

from __future__ import annotations

from typing import TYPE_CHECKING, Annotated, Any, Literal, TypeAlias

import numpy as np
from pydantic import Field, FiniteFloat, field_validator

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
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.manipulation.grasping.grasp_gen_x_runtime import GraspGenXRuntime

GRASPGENX_MODEL_REPO = "adithyamurali/GraspGenXModel"
GRASPGENX_MODEL_REVISION = "7c834043c11a11417e31d6d5ea9355801e40a2c1"
GRASPGENX_MODEL_VERSION = "release"

BoundedExtent = Annotated[FiniteFloat, Field(gt=0.0, le=0.5)]
BoundedOffset = Annotated[FiniteFloat, Field(ge=-0.5, le=0.5)]
PositiveCount = Annotated[int, Field(gt=0, strict=True)]
SweepExtents: TypeAlias = tuple[BoundedExtent, BoundedExtent, BoundedExtent]
SweepOffset: TypeAlias = tuple[BoundedOffset, BoundedOffset, BoundedOffset]
Vector4: TypeAlias = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
RigidTransform: TypeAlias = tuple[Vector4, Vector4, Vector4, Vector4]
GripperFamily: TypeAlias = Literal["parallel_2f", "revolute_2f", "revolute_3f"]

IDENTITY_TRANSFORM: RigidTransform = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


class SweepVolumeGripperConfig(BaseConfig):
    """Axis-aligned open and half-open sweep-volume description."""

    extents_open: SweepExtents
    offset_open: SweepOffset
    extents_half_open: SweepExtents
    offset_half_open: SweepOffset
    fingertip_depth: BoundedExtent
    family: GripperFamily = "parallel_2f"


class GraspGenXConfig(ModuleConfig):
    """GraspGenX deployment settings, serializable by DimOS blueprints."""

    gripper: SweepVolumeGripperConfig
    grasp_frame_to_tcp: RigidTransform = IDENTITY_TRANSFORM
    max_candidates: PositiveCount = 100
    load_on_start: bool = True

    # Relational matrix properties cannot be expressed through scalar Field constraints.
    @field_validator("grasp_frame_to_tcp")
    @classmethod
    def _validate_rigid_transform(cls, value: RigidTransform) -> RigidTransform:
        matrix = np.asarray(value, dtype=float)
        if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
            raise ValueError("grasp_frame_to_tcp must be homogeneous")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-6
        ):
            raise ValueError("grasp_frame_to_tcp rotation must be orthonormal with determinant +1")
        return value


class GraspGenXError(RuntimeError):
    """Base error for model loading and inference failures."""


def _create_runtime(config: GraspGenXConfig) -> GraspGenXRuntime:
    # This import is the intentional first-use boundary for the optional GPU runtime.
    from dimos.manipulation.grasping.grasp_gen_x_runtime import GraspGenXRuntime

    return GraspGenXRuntime(config)


class GraspGenXModule(Module, GraspGenSpec):
    """Direct adapter whose optional runtime is loaded synchronously by ``start``."""

    dedicated_worker = True
    config: GraspGenXConfig  # type: ignore[assignment]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime: GraspGenXRuntime | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.load_on_start:
            self._ensure_runtime()

    def _ensure_runtime(self) -> None:
        if self._runtime is not None:
            return
        try:
            self._runtime = _create_runtime(self.config)
        except Exception as exc:
            raise GraspGenXError("failed to initialize GraspGenX") from exc

    @rpc
    def stop(self) -> None:
        if self._runtime is not None:
            self._runtime.stop()
        self._runtime = None
        super().stop()

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        if object_pointcloud.ts is None:
            raise ValueError("object pointcloud must have a timestamp")
        if not object_pointcloud.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")

        points = object_pointcloud.points_f32()
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("object pointcloud must contain at least one XYZ point")
        if not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")

        self._ensure_runtime()
        assert self._runtime is not None
        try:
            poses, scores = self._runtime.infer(points)
        except Exception as exc:
            raise GraspGenXError("GraspGenX inference failed") from exc
        scores = scores.reshape(-1)

        if poses.size == 0 and scores.size == 0:
            return GraspCandidateArray(
                Header(float(object_pointcloud.ts), object_pointcloud.frame_id),
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
            Header(float(object_pointcloud.ts), object_pointcloud.frame_id),
            candidates,
        )

    @staticmethod
    def _pose_from_matrix(matrix: np.ndarray) -> Pose:
        return Pose(
            {
                "position": Vector3(matrix[:3, 3]),
                "orientation": Quaternion.from_rotation_matrix(matrix[:3, :3]),
            }
        )
