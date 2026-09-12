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

from typing import Annotated, Literal, TypeAlias

import numpy as np
from pydantic import Field, FiniteFloat, field_validator

from dimos.core.core import rpc
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service.spec import BaseConfig

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


class GraspGenXConfig(IsolatedPythonModuleConfig):
    """GraspGenX deployment settings, serializable by DimOS blueprints."""

    gripper: SweepVolumeGripperConfig
    grasp_frame_to_tcp: RigidTransform = IDENTITY_TRANSFORM
    max_candidates: PositiveCount = 100

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


class GraspGenXModule(IsolatedPythonModule, GraspGenSpec):
    """Grasp proposals implemented in a separately locked Python environment."""

    implementation = "graspgenx_runtime.runtime:_GraspGenXRuntimeModule"
    config: GraspGenXConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        """Return ranked TCP poses for an object point cloud in its input frame."""
        raise NotImplementedError
