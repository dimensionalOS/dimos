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

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, TypeAlias

from dimos.manipulation.planning.spec.enums import (
    IKStatus,
    ObstacleType,
    ParametrizationStatus,
    PlanningStatus,
    TrajectoryDispatchStatus,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
    from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


RobotName: TypeAlias = str
"""User-facing robot name (e.g., 'left_arm', 'right_arm')"""

WorldRobotID: TypeAlias = str
"""Internal Drake world robot ID"""

PlanningGroupID: TypeAlias = str
"""Public planning group ID of the form {robot_name}/{group_name}."""

LocalModelJointName: TypeAlias = str
"""Joint name as it appears in URDF/SRDF before world binding."""

GlobalJointName: TypeAlias = str
"""Public joint name of the form {robot_name}/{local_joint_name}."""

JointPath: TypeAlias = "list[JointState]"
"""List of joint states forming a path (each waypoint has names + positions)"""


@dataclass(frozen=True)
class PlanningSceneInfo:
    """Stable planning-scene metadata for external collaborators.

    This snapshot intentionally carries setup metadata only. It must not expose
    backend handles, mutable world contexts, GUI state, or execution state.
    """

    robots: Mapping[WorldRobotID, RobotModelConfig]
    """Robot model configurations keyed by world robot ID."""


Jacobian: TypeAlias = "NDArray[np.float64]"
"""6 x n Jacobian matrix (rows: [vx, vy, vz, wx, wy, wz])"""

CollisionCheckStatus: TypeAlias = Literal[
    "VALID",
    "COLLISION",
    "INVALID",
    "UNAVAILABLE",
    "STALE_STATE",
]
"""Status for a planning-world collision target check."""

ForwardKinematicsStatus: TypeAlias = Literal[
    "VALID",
    "INVALID",
    "UNAVAILABLE",
    "STALE_STATE",
]
"""Status for a group-scoped forward-kinematics query."""

CartesianPathMode: TypeAlias = Literal["free", "linear"]
"""Mode describing requested Cartesian path semantics."""

PathConstraintKind: TypeAlias = Literal["linear_tcp"]
"""Kind discriminator for geometric path constraints."""


@dataclass(frozen=True)
class CartesianDelta:
    """Relative TCP delta for Cartesian planning.

    Translation is meters. Rotation is roll, pitch, yaw in radians. The frame is the
    frame in which the delta is expressed.
    """

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frame_id: str = "world"


@dataclass(frozen=True)
class LinearTcpPathConstraint:
    """Straight-line TCP constraint carried by a geometric plan.

    The constrained TCP must follow the world-frame segment from `start_pose` to
    `target_pose` within the provided translational and rotational tolerances.
    """

    kind: PathConstraintKind = "linear_tcp"
    group_id: PlanningGroupID = ""
    tcp_frame: str = ""
    start_pose: PoseStamped | None = None
    target_pose: PoseStamped | None = None
    max_translational_deviation: float = 1e-3
    max_rotational_deviation: float = 1e-2


PathConstraintMetadata: TypeAlias = LinearTcpPathConstraint
"""Optional metadata declaring constraints a post-processor must preserve."""


@dataclass(frozen=True)
class CollisionCheckResult:
    """Result of a planning-world collision target check."""

    status: CollisionCheckStatus
    collision_free: bool | None
    message: str


@dataclass(frozen=True)
class ForwardKinematicsResult:
    """Result of a group-scoped forward-kinematics query."""

    status: ForwardKinematicsStatus
    pose: PoseStamped | None
    message: str


@dataclass
class GeneratedPlan:
    """Canonical generated planning artifact.

    The path uses global joint names and contains exactly the selected joints.
    Downstream preview/execution projections are computed lazily from this data.
    """

    group_ids: tuple[PlanningGroupID, ...]
    path: list[JointState] = field(default_factory=list)
    status: PlanningStatus = PlanningStatus.NO_SOLUTION
    planning_time: float = 0.0
    path_length: float = 0.0
    iterations: int = 0
    message: str = ""
    path_constraints: PathConstraintMetadata | None = None

    def is_success(self) -> bool:
        """Check if planning was successful."""
        return self.status == PlanningStatus.SUCCESS


@dataclass
class GeneratedTrajectory:
    """Canonical global time-parametrized manipulation artifact.

    The trajectory uses global joint names, a single shared `time_from_start`
    domain across all joints, and status that is independent from the source
    geometric `GeneratedPlan.status`.
    """

    joint_names: list[GlobalJointName] = field(default_factory=list)
    points: list[TrajectoryPoint] = field(default_factory=list)
    duration: float = 0.0
    speed_scale: float = 1.0
    status: ParametrizationStatus = ParametrizationStatus.FAILED
    message: str = ""
    source_group_ids: tuple[PlanningGroupID, ...] = ()
    source_plan_status: PlanningStatus = PlanningStatus.NO_SOLUTION
    source_plan_message: str = ""

    def is_success(self) -> bool:
        """Check if trajectory parametrization was successful."""
        return self.status == ParametrizationStatus.SUCCESS


@dataclass
class TrajectoryDispatch:
    """Execution-preparation artifact derived from `GeneratedTrajectory`.

    `trajectories_by_task` contains coordinator-task-specific messages. These
    messages preserve the generated trajectory timing instead of retiming each
    task projection independently.
    """

    trajectories_by_task: dict[str, JointTrajectory] = field(default_factory=dict)
    robot_names_by_task: dict[str, RobotName] = field(default_factory=dict)
    status: TrajectoryDispatchStatus = TrajectoryDispatchStatus.FAILED
    message: str = ""

    def is_success(self) -> bool:
        """Check if dispatch preparation was successful."""
        return self.status == TrajectoryDispatchStatus.SUCCESS


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
    path_constraints: PathConstraintMetadata | None = None

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
