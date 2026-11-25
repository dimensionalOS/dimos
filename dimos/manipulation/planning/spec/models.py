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

from collections.abc import Sequence
from dataclasses import dataclass, field, fields
from typing import TYPE_CHECKING, TypeAlias
from uuid import uuid4

import numpy as np

from dimos.manipulation.planning.spec.enums import (
    IKStatus,
    ObstacleType,
    PlanningStatus,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.groups.models import PlanningGroup
    from dimos.manipulation.planning.spec.validation import PreparedRobotModel
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


PlanningGroupID: TypeAlias = str
"""Stable public planning-group name."""

JointName: TypeAlias = str
"""Canonical joint name used by model, planning, state, and execution."""

JointPath: TypeAlias = "list[JointState]"
"""List of joint states forming a path (each waypoint has names + positions)"""


CartesianWaypoint: TypeAlias = "PoseStamped | Transform"
"""One absolute TCP pose or relative rigid displacement from the planning start."""

CartesianTarget: TypeAlias = "Sequence[PoseStamped] | Sequence[Transform]"
"""Ordered homogeneous Cartesian waypoints for one planning group."""


@dataclass(frozen=True)
class PlanningSceneInfo:
    """Stable planning-scene metadata for external collaborators.

    This snapshot intentionally carries setup metadata only. It must not expose
    backend handles, mutable world contexts, GUI state, or execution state.
    """

    model: PreparedRobotModel
    """The materialized and validated logical robot model."""

    planning_groups: tuple[PlanningGroup, ...] = ()
    """Resolved immutable planning groups for the initialized scene."""


@dataclass(frozen=True)
class VisualizationSession:
    """One-shot immutable visualization initialization payload."""

    scene: PlanningSceneInfo
    operator: object | None = None
    """Optional concrete ManipulationOperator; typed as object to avoid low-level cycles."""


@dataclass(frozen=True)
class VisualizationStateFrame:
    """Pushed current joint states for visualization backends."""

    joint_state: JointState | None


Jacobian: TypeAlias = "NDArray[np.float64]"
"""6 x n Jacobian matrix (rows: [vx, vy, vz, wx, wy, wz])"""


DEFAULT_OBSTACLE_RGBA: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8)
"""Default RGBA (0-1 range) applied to obstacles that carry no explicit color."""


@dataclass
class Obstacle:
    """Obstacle specification for collision avoidance.

    Attributes:
        name: Unique name for the obstacle
        obstacle_type: Type of geometry (BOX, SPHERE, CYLINDER, MESH, OCTREE)
        pose: Pose of the obstacle in world frame
        dimensions: Type-specific dimensions:
            - BOX: (width, height, depth)
            - SPHERE: (radius,)
            - CYLINDER: (radius, height)
            - MESH: Not used
        color: RGBA color tuple (0-1 range)
        mesh_path: Path to mesh file (for MESH type)
        points: Occupied cell centers in the obstacle's frame as an (N, 3) float
            array (for OCTREE type). Sequences of triples are accepted and converted.
        octree_resolution: Edge length of an OCTREE cell (meters)
        touch_links: Robot links allowed to touch the obstacle, by exact link
            name; their collisions with it are ignored
    """

    name: str
    obstacle_type: ObstacleType
    pose: PoseStamped
    dimensions: tuple[float, ...] = ()
    color: tuple[float, float, float, float] = DEFAULT_OBSTACLE_RGBA
    mesh_path: str | None = None
    # An array, not one tuple per cell: it crosses worker RPC as a single ndarray tag.
    # __eq__ below keeps obstacles comparable, which the world's snapshots rely on.
    points: NDArray[np.float64] = field(default_factory=lambda: np.empty((0, 3)))
    octree_resolution: float | None = None
    touch_links: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Obstacle):
            return NotImplemented
        for f in fields(self):
            mine, theirs = getattr(self, f.name), getattr(other, f.name)
            same = np.array_equal(mine, theirs) if f.name == "points" else mine == theirs
            if not same:
                return False
        return True


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
        path_length: Unweighted Euclidean coordinate-space length. This diagnostic
            has no single physical unit when a path mixes linear and angular joints.
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
class GeneratedPlan:
    """Canonical selected-planning-group plan exposed by ManipulationModule."""

    group_ids: tuple[PlanningGroupID, ...]
    trajectory: JointTrajectory
    path: list[JointState] = field(default_factory=list)
    status: PlanningStatus = PlanningStatus.NO_SOLUTION
    planning_time: float = 0.0
    path_length: float = 0.0
    iterations: int = 0
    message: str = ""
    plan_id: str = field(default_factory=lambda: uuid4().hex)

    def is_success(self) -> bool:
        """Check if the generated plan was successful."""
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
        color: RGBA color tuple. ``None`` means the field was omitted.
    """

    id: str
    operation: str  # "add", "update", "remove"
    primitive_type: str | None = None
    pose: PoseStamped | None = None
    dimensions: tuple[float, ...] | None = None
    color: tuple[float, float, float, float] | None = None
