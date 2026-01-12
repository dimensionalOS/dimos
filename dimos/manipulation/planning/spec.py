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

"""
Manipulation Planning Specifications

Contains Protocol definitions and data classes for the manipulation planning stack.

## Protocols

- WorldSpec: Core backend owning physics/collision (implemented by DrakeWorld)
- KinematicsSpec: Stateless IK operations (implemented by DrakeKinematics)
- PlannerSpec: Joint-space path planning (implemented by DrakePlanner)
- VizSpec: Visualization backend (future)

## Data Classes

- RobotModelConfig: Robot configuration for adding to world
- Obstacle: Obstacle specification
- IKResult: Result of IK solve
- PlanningResult: Result of path planning

## Usage

All code should use Protocol types (not concrete classes):

```python
from dimos.manipulation.planning.spec import WorldSpec, KinematicsSpec

def plan_motion(world: WorldSpec, kinematics: KinematicsSpec, ...):
    # Works with any conforming implementation
    pass
```

Use factory functions to create concrete instances:

```python
from dimos.manipulation.planning.factory import create_world, create_kinematics

world = create_world(backend="drake")  # Returns WorldSpec
kinematics = create_kinematics()       # Returns KinematicsSpec
```
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    from contextlib import AbstractContextManager

    import numpy as np
    from numpy.typing import NDArray


# =============================================================================
# Enums
# =============================================================================


class ObstacleType(Enum):
    """Type of obstacle geometry."""

    BOX = auto()
    SPHERE = auto()
    CYLINDER = auto()
    MESH = auto()


class IKStatus(Enum):
    """Status of IK solution."""

    SUCCESS = auto()
    NO_SOLUTION = auto()
    SINGULARITY = auto()
    JOINT_LIMITS = auto()
    COLLISION = auto()
    TIMEOUT = auto()


class PlanningStatus(Enum):
    """Status of motion planning."""

    SUCCESS = auto()
    NO_SOLUTION = auto()
    TIMEOUT = auto()
    INVALID_START = auto()
    INVALID_GOAL = auto()
    COLLISION_AT_START = auto()
    COLLISION_AT_GOAL = auto()


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class RobotModelConfig:
    """Configuration for adding a robot to the world.

    Attributes:
        name: Human-readable robot name
        urdf_path: Path to URDF file (can be .urdf or .xacro)
        base_pose: 4x4 homogeneous transform for robot base
        joint_names: Ordered list of controlled joint names (in URDF namespace)
        end_effector_link: Name of the end-effector link for FK/IK
        base_link: Name of the base link (default: "base_link")
        package_paths: Dict mapping package names to filesystem paths
        joint_limits_lower: Lower joint limits (radians)
        joint_limits_upper: Upper joint limits (radians)
        velocity_limits: Joint velocity limits (rad/s)
        auto_convert_meshes: Auto-convert DAE/STL meshes to OBJ for Drake
        xacro_args: Arguments to pass to xacro processor (for .xacro files)
        collision_exclusion_pairs: List of (link1, link2) pairs to exclude from collision.
            Useful for parallel linkage mechanisms like grippers where non-adjacent
            links may legitimately overlap (e.g., mimic joints).
        max_velocity: Maximum joint velocity for trajectory generation (rad/s)
        max_acceleration: Maximum joint acceleration for trajectory generation (rad/s^2)
        joint_name_mapping: Maps orchestrator joint names to URDF joint names.
            Example: {"left_joint1": "joint1"} means orchestrator's "left_joint1"
            corresponds to URDF's "joint1". If empty, names are assumed to match.
        orchestrator_task_name: Task name for executing trajectories via orchestrator RPC.
            If set, trajectories can be executed via execute_trajectory() RPC.
    """

    name: str
    urdf_path: str
    base_pose: NDArray[np.float64]
    joint_names: list[str]
    end_effector_link: str
    base_link: str = "base_link"
    package_paths: dict[str, str] = field(default_factory=dict)
    joint_limits_lower: NDArray[np.float64] | None = None
    joint_limits_upper: NDArray[np.float64] | None = None
    velocity_limits: NDArray[np.float64] | None = None
    auto_convert_meshes: bool = False
    xacro_args: dict[str, str] = field(default_factory=dict)
    collision_exclusion_pairs: list[tuple[str, str]] = field(default_factory=list)
    # Motion constraints for trajectory generation
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    # Orchestrator integration
    joint_name_mapping: dict[str, str] = field(default_factory=dict)
    orchestrator_task_name: str | None = None

    def get_urdf_joint_name(self, orchestrator_name: str) -> str:
        """Translate orchestrator joint name to URDF joint name.

        Args:
            orchestrator_name: Joint name from orchestrator (e.g., "left_joint1")

        Returns:
            Corresponding URDF joint name (e.g., "joint1")
        """
        return self.joint_name_mapping.get(orchestrator_name, orchestrator_name)

    def get_orchestrator_joint_name(self, urdf_name: str) -> str:
        """Translate URDF joint name to orchestrator joint name.

        Args:
            urdf_name: Joint name from URDF (e.g., "joint1")

        Returns:
            Corresponding orchestrator joint name (e.g., "left_joint1")
        """
        # Reverse lookup
        for orch_name, u_name in self.joint_name_mapping.items():
            if u_name == urdf_name:
                return orch_name
        return urdf_name

    def get_orchestrator_joint_names(self) -> list[str]:
        """Get joint names in orchestrator namespace.

        Returns:
            List of joint names as they appear in orchestrator messages.
            If no mapping, returns the URDF joint names.
        """
        if not self.joint_name_mapping:
            return self.joint_names
        return [self.get_orchestrator_joint_name(j) for j in self.joint_names]


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
class Detection3D:
    """3D detection result from perception pipeline.

    Used by monitors to handle perception updates.

    Attributes:
        id: Unique detection ID
        label: Object class label
        pose: 4x4 transform
        dimensions: (width, height, depth)
        confidence: Detection confidence (0-1)
    """

    id: str
    label: str
    pose: NDArray[np.float64]
    dimensions: tuple[float, float, float]
    confidence: float = 1.0


# =============================================================================
# Protocols
# =============================================================================


@runtime_checkable
class WorldSpec(Protocol):
    """Protocol for the world/scene backend.

    The world owns the physics/collision backend and provides:
    - Robot/obstacle management
    - Collision checking
    - Forward kinematics
    - Context management for thread safety

    ## Context Management

    The world maintains:
    - Live context: Mirrors current robot state (synced from driver)
    - Scratch contexts: Thread-safe clones for planning/IK operations

    All state-dependent operations (collision checking, FK, Jacobian) take
    a context parameter to ensure thread safety.

    ## Implementations

    - DrakeWorld: Uses Drake's MultibodyPlant and SceneGraph
    """

    # =========================================================================
    # Robot Management
    # =========================================================================

    def add_robot(self, config: RobotModelConfig) -> str:
        """Add a robot to the world.

        Args:
            config: Robot configuration

        Returns:
            Unique robot ID
        """
        ...

    def get_robot_ids(self) -> list[str]:
        """Get all robot IDs."""
        ...

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        """Get robot configuration."""
        ...

    def get_joint_limits(self, robot_id: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits (lower, upper) for a robot."""
        ...

    # =========================================================================
    # Obstacle Management
    # =========================================================================

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle to the world.

        Args:
            obstacle: Obstacle specification

        Returns:
            Unique obstacle ID
        """
        ...

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle.

        Returns:
            True if obstacle was removed
        """
        ...

    def update_obstacle_pose(self, obstacle_id: str, pose: NDArray[np.float64]) -> bool:
        """Update obstacle pose.

        Returns:
            True if obstacle was updated
        """
        ...

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        ...

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def finalize(self) -> None:
        """Finalize the world for simulation/collision checking.

        Must be called after adding all robots and before any operations.
        """
        ...

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        ...

    # =========================================================================
    # Context Management
    # =========================================================================

    def get_live_context(self) -> Any:
        """Get the live context (mirrors real robot state).

        The live context is synced with real robot state via
        sync_from_joint_state(). Use with caution - modifications
        affect all users.

        For planning, prefer scratch_context() instead.
        """
        ...

    def scratch_context(self) -> AbstractContextManager[Any]:
        """Get a scratch context for planning (thread-safe clone).

        Example:
            with world.scratch_context() as ctx:
                world.set_positions(ctx, robot_id, q_test)
                if world.is_collision_free(ctx, robot_id):
                    # Valid configuration
                    pass

        Yields:
            Context that can be used for state operations
        """
        ...

    def sync_from_joint_state(self, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Sync live context from joint state (called by driver/monitor).

        Args:
            robot_id: Robot to update
            positions: Joint positions from driver
        """
        ...

    # =========================================================================
    # State Operations (require context)
    # =========================================================================

    def set_positions(self, ctx: Any, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Set robot joint positions in a context.

        Args:
            ctx: Context from scratch_context() or get_live_context()
            robot_id: Robot to update
            positions: Joint positions (radians)
        """
        ...

    def get_positions(self, ctx: Any, robot_id: str) -> NDArray[np.float64]:
        """Get robot joint positions from a context.

        Args:
            ctx: Context
            robot_id: Robot to query

        Returns:
            Joint positions (radians)
        """
        ...

    # =========================================================================
    # Collision Checking (require context)
    # =========================================================================

    def is_collision_free(self, ctx: Any, robot_id: str) -> bool:
        """Check if robot configuration is collision-free.

        Args:
            ctx: Context with robot state set
            robot_id: Robot to check

        Returns:
            True if no collisions
        """
        ...

    def get_min_distance(self, ctx: Any, robot_id: str) -> float:
        """Get minimum distance to obstacles.

        Args:
            ctx: Context with robot state set
            robot_id: Robot to check

        Returns:
            Minimum distance (meters), negative if in collision
        """
        ...

    # =========================================================================
    # Forward Kinematics (require context)
    # =========================================================================

    def get_ee_pose(self, ctx: Any, robot_id: str) -> NDArray[np.float64]:
        """Get end-effector pose.

        Args:
            ctx: Context with robot state set
            robot_id: Robot to query

        Returns:
            4x4 homogeneous transform
        """
        ...

    def get_jacobian(self, ctx: Any, robot_id: str) -> NDArray[np.float64]:
        """Get end-effector Jacobian.

        Args:
            ctx: Context with robot state set
            robot_id: Robot to query

        Returns:
            6 x n_joints Jacobian matrix [linear; angular]
        """
        ...


@runtime_checkable
class KinematicsSpec(Protocol):
    """Protocol for inverse kinematics solver.

    Kinematics solvers are stateless (except for configuration) and
    use WorldSpec for all FK/collision operations.

    ## Methods

    - solve(): Full optimization-based IK with collision checking
    - solve_iterative(): Iterative Jacobian-based IK
    - solve_differential(): Single Jacobian step for velocity control

    ## Implementations

    - DrakeKinematics: Uses Drake's InverseKinematics + SNOPT/IPOPT
    """

    def solve(
        self,
        world: WorldSpec,
        robot_id: str,
        target_pose: NDArray[np.float64],
        seed: NDArray[np.float64] | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve full IK with optional collision checking.

        Args:
            world: World for FK/collision
            robot_id: Robot to solve for
            target_pose: 4x4 target end-effector transform
            seed: Initial guess (uses current state if None)
            position_tolerance: Position tolerance (meters)
            orientation_tolerance: Orientation tolerance (radians)
            check_collision: Check solution is collision-free
            max_attempts: Random restarts for robustness

        Returns:
            IKResult with status and solution
        """
        ...

    def solve_iterative(
        self,
        world: WorldSpec,
        robot_id: str,
        target_pose: NDArray[np.float64],
        seed: NDArray[np.float64],
        max_iterations: int = 100,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
    ) -> IKResult:
        """Solve IK iteratively using Jacobian method.

        Slower but more predictable behavior near singularities.

        Args:
            world: World for FK/Jacobian
            robot_id: Robot to solve for
            target_pose: 4x4 target transform
            seed: Initial joint configuration
            max_iterations: Maximum iterations
            position_tolerance: Convergence tolerance (meters)
            orientation_tolerance: Convergence tolerance (radians)

        Returns:
            IKResult with status and solution
        """
        ...

    def solve_differential(
        self,
        world: WorldSpec,
        robot_id: str,
        current_joints: NDArray[np.float64],
        twist: NDArray[np.float64],
        dt: float,
    ) -> NDArray[np.float64] | None:
        """Single Jacobian step for velocity control.

        Args:
            world: World for Jacobian computation
            robot_id: Robot to control
            current_joints: Current joint positions
            twist: Desired end-effector twist [vx, vy, vz, wx, wy, wz]
            dt: Time step (seconds)

        Returns:
            Joint velocities (rad/s) or None if near singularity
        """
        ...


@runtime_checkable
class PlannerSpec(Protocol):
    """Protocol for motion planner.

    Planners find collision-free paths from start to goal configurations.
    They use WorldSpec for collision checking and are stateless.

    ## Implementations

    - DrakePlanner: RRT-Connect planner
    - DrakeRRTStarPlanner: RRT* planner (asymptotically optimal)
    """

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a collision-free joint-space path.

        Args:
            world: World for collision checking
            robot_id: Robot to plan for
            q_start: Start configuration
            q_goal: Goal configuration
            timeout: Planning timeout (seconds)

        Returns:
            PlanningResult with status and path
        """
        ...

    def get_name(self) -> str:
        """Get planner name."""
        ...


@runtime_checkable
class VizSpec(Protocol):
    """Protocol for visualization backend.

    Provides methods to update robot/obstacle visualization.

    Note: For Drake, visualization is typically integrated into DrakeWorld
    via enable_viz=True. This protocol is for advanced use cases.

    ## Implementations

    - DrakeWorld (integrated): Use create_world(enable_viz=True)
    """

    def set_robot_state(self, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Update robot visualization state."""
        ...

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add obstacle to visualization."""
        ...

    def remove_obstacle(self, obstacle_id: str) -> None:
        """Remove obstacle from visualization."""
        ...

    def get_url(self) -> str | None:
        """Get visualization URL (e.g., Meshcat URL)."""
        ...

    def publish(self) -> None:
        """Force publish current state to visualization."""
        ...
