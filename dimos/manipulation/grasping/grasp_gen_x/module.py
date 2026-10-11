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

from collections.abc import Sequence
from typing import Annotated, Literal, TypeAlias

import numpy as np
from pydantic import Field, FiniteFloat, field_validator

from dimos.core.core import rpc
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.manipulation.grasping.grasp_gen_spec import (
    GraspGenSpec,
    SceneGraspGenSpec,
    SceneGrasps,
)
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

# Most distance (metres) between neighbouring points sampled on the gripper body.
# A collision threshold of at least this leaves no gap a scene point can sit in.
GRIPPER_POINT_SPACING = 0.007

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


class GripperBox(BaseConfig):
    """One box of the gripper's solid body, in the same frame as the sweep volume.

    Args:
        extents: Full edge lengths along X, Y and Z, metres.
        offset: Centre of the box, metres.
    """

    extents: SweepExtents
    offset: SweepOffset


class GraspGenXConfig(IsolatedPythonModuleConfig):
    """GraspGenX deployment settings, serializable by DimOS blueprints."""

    gripper: SweepVolumeGripperConfig
    grasp_frame_to_tcp: RigidTransform = IDENTITY_TRANSFORM
    max_candidates: PositiveCount = 100
    # The gripper's solid parts with the jaws open, for propose_grasps_in_scene.
    # Empty makes that RPC raise.
    gripper_body: tuple[GripperBox, ...] = ()

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


class GraspGenXModule(IsolatedPythonModule, GraspGenSpec, SceneGraspGenSpec):
    """Grasp proposals implemented in a separately locked Python environment."""

    project_dir = "native/python/graspgenx"
    implementation = "graspgenx_runtime.runtime:GraspGenXRuntimeModule"
    config: GraspGenXConfig

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        """Return ranked TCP poses for an object point cloud in its input frame."""
        raise NotImplementedError

    @rpc
    def propose_grasps_in_scene(
        self,
        object_pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2,
        collision_threshold: float,
    ) -> SceneGrasps:
        """Propose grasps like propose_grasps, then drop those that would hit the scene.

        A grasp is dropped when any part of the gripper body (``gripper_body``),
        open and at the grasp pose, comes closer than ``collision_threshold`` to
        a scene point.

        Args:
            object_pointcloud: The object's points, metres.
            scene_pointcloud: Points around the object, such as the table and other
                objects, without the object's own points, in the same frame,
                metres. Object points left in make every grasp collide.
            collision_threshold: A gripper part closer than this (metres) to a
                scene point is a collision. Below ``GRIPPER_POINT_SPACING`` a scene
                point can sit between gripper points unseen.
        """
        raise NotImplementedError


def gripper_body_points(
    boxes: Sequence[GripperBox], spacing: float = GRIPPER_POINT_SPACING
) -> np.ndarray:
    """Points spread over the surface of each box, for checking the gripper against a scene.

    Args:
        boxes: The gripper's solid parts.
        spacing: Most distance (metres) between neighbouring points on a face.

    Returns an (N, 3) float32 array in the boxes' frame, metres; empty for no boxes.
    """
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    surfaces = []
    for box in boxes:
        half = np.asarray(box.extents, dtype=np.float64) / 2.0
        center = np.asarray(box.offset, dtype=np.float64)
        axes = [np.linspace(-h, h, int(np.ceil(2.0 * h / spacing)) + 1) for h in half]
        grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
        # The grid's outermost layer on any axis lies on the box surface.
        on_surface = np.any(np.isclose(np.abs(grid), half), axis=1)
        surfaces.append(grid[on_surface] + center)
    if not surfaces:
        return np.empty((0, 3), dtype=np.float32)
    return np.concatenate(surfaces).astype(np.float32)
