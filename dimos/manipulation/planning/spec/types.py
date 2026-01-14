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

"""Data types for manipulation planning."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from dimos.manipulation.planning.spec.enums import (
    IKStatus,
    ObstacleType,
    PlanningStatus,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


@dataclass
class Obstacle:
    """Obstacle specification for collision avoidance.

    Attributes:
        name: Unique name for the obstacle
        obstacle_type: Type of geometry (BOX, SPHERE, CYLINDER, MESH)
        pose: 4x4 homogeneous transform
        dimensions: Type-specific dimensions:
            - BOX: (width, height, depth)
            - SPHERE: (radius,)
            - CYLINDER: (radius, height)
            - MESH: Not used
        color: RGBA color tuple (0-1 range)
        mesh_path: Path to mesh file (for MESH type)
    """

    name: str
    obstacle_type: ObstacleType
    pose: NDArray[np.float64]
    dimensions: tuple[float, ...] = ()
    color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8)
    mesh_path: str | None = None


@dataclass
class IKResult:
    """Result of an IK solve.

    Attributes:
        status: Solution status
        joint_positions: Solution joint positions (None if failed)
        position_error: Cartesian position error (meters)
        orientation_error: Orientation error (radians)
        iterations: Number of iterations taken
        message: Human-readable status message
    """

    status: IKStatus
    joint_positions: NDArray[np.float64] | None = None
    position_error: float = 0.0
    orientation_error: float = 0.0
    iterations: int = 0
    message: str = ""

    def is_success(self) -> bool:
        """Check if IK was successful."""
        return self.status == IKStatus.SUCCESS


@dataclass
class PlanningResult:
    """Result of motion planning.

    Attributes:
        status: Planning status
        path: List of joint configurations (empty if failed)
        planning_time: Time taken to plan (seconds)
        path_length: Total path length in joint space (radians)
        iterations: Number of iterations/nodes expanded
        message: Human-readable status message
    """

    status: PlanningStatus
    path: list[NDArray[np.float64]] = field(default_factory=list)
    planning_time: float = 0.0
    path_length: float = 0.0
    iterations: int = 0
    message: str = ""

    def is_success(self) -> bool:
        """Check if planning was successful."""
        return self.status == PlanningStatus.SUCCESS


@dataclass
class CollisionObjectMessage:
    """Message for adding/updating/removing obstacles.

    Used by monitors to handle obstacle updates from external sources.

    Attributes:
        id: Unique identifier for the object
        operation: "add", "update", or "remove"
        primitive_type: "box", "sphere", or "cylinder" (for add/update)
        pose: 4x4 transform (for add/update)
        dimensions: Type-specific dimensions (for add/update)
        color: RGBA color tuple
    """

    id: str
    operation: str  # "add", "update", "remove"
    primitive_type: str | None = None
    pose: NDArray[np.float64] | None = None
    dimensions: tuple[float, ...] | None = None
    color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8)


@dataclass
class PerceptionDetection:
    """3D detection for planning obstacle updates.

    Internal type for the planning system using numpy transforms.
    For LCM messages, use dimos_lcm.vision_msgs.Detection3D instead.

    Attributes:
        id: Unique detection ID
        label: Object class label
        pose: 4x4 homogeneous transform
        dimensions: (width, height, depth)
        confidence: Detection confidence (0-1)
    """

    id: str
    label: str
    pose: NDArray[np.float64]
    dimensions: tuple[float, float, float]
    confidence: float = 1.0


# Backwards compatibility alias - will be removed in future version
Detection3D = PerceptionDetection
