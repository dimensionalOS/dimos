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
from typing import TYPE_CHECKING, TypeAlias

from dimos.manipulation.planning.spec.enums import (
    IKStatus,
    ObstacleType,
    PlanningStatus,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs import PoseStamped
    from dimos.msgs.sensor_msgs import JointState

# =============================================================================
# Semantic ID Types (documentation only, not enforced at runtime)
# =============================================================================

RobotName: TypeAlias = str
"""User-facing robot name (e.g., 'left_arm', 'right_arm')"""

WorldRobotID: TypeAlias = str
"""Internal Drake world robot ID"""

JointPath: TypeAlias = "list[JointState]"
"""List of joint states forming a path (each waypoint has names + positions)"""

# =============================================================================
# Numeric Array Types
# =============================================================================

Jacobian: TypeAlias = "NDArray[np.float64]"
"""6 x n Jacobian matrix (rows: [vx, vy, vz, wx, wy, wz])"""


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class Obstacle:
    """Obstacle specification for collision avoidance.

    Attributes:
        name: Unique name for the obstacle
        obstacle_type: Type of geometry (BOX, SPHERE, CYLINDER, MESH)
        pose: Pose of the obstacle in world frame
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
    pose: PoseStamped
    dimensions: tuple[float, ...] = ()
    color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8)
    mesh_path: str | None = None


@dataclass
class IKResult:
    """Result of an IK solve.

    Attributes:
        status: Solution status
        joint_state: Solution joint state with names and positions (None if failed)
        position_error: Cartesian position error (meters)
        orientation_error: Orientation error (radians)
        iterations: Number of iterations taken
        message: Human-readable status message
    """

    status: IKStatus
    joint_state: JointState | None = None
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
        path: List of joint states forming the path (empty if failed).
            Each JointState contains names, positions, and optionally velocities.
        planning_time: Time taken to plan (seconds)
        path_length: Total path length in joint space (radians)
        iterations: Number of iterations/nodes expanded
        message: Human-readable status message
        timestamps: Optional timestamps for each waypoint (seconds from start).
            If provided by the planner, trajectory generator can use these directly.
    """

    status: PlanningStatus
    path: list[JointState] = field(default_factory=list)
    planning_time: float = 0.0
    path_length: float = 0.0
    iterations: int = 0
    message: str = ""
    # Optional timing (set by optimization-based planners)
    timestamps: list[float] | None = None

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
        pose: Pose of the obstacle (for add/update)
        dimensions: Type-specific dimensions (for add/update)
        color: RGBA color tuple
    """

    id: str
    operation: str  # "add", "update", "remove"
    primitive_type: str | None = None
    pose: PoseStamped | None = None
    dimensions: tuple[float, ...] | None = None
    color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8)
