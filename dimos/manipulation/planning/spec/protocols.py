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

"""Protocol definitions for manipulation planning.

All code should use these Protocol types (not concrete classes).
Use factory functions from dimos.manipulation.planning.factory to create instances.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    from contextlib import AbstractContextManager

    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.types import IKResult, Obstacle, PlanningResult


@runtime_checkable
class WorldSpec(Protocol):
    """Protocol for the world/scene backend.

    The world owns the physics/collision backend and provides:
    - Robot/obstacle management
    - Collision checking
    - Forward kinematics
    - Context management for thread safety

    Context Management:
        - Live context: Mirrors current robot state (synced from driver)
        - Scratch contexts: Thread-safe clones for planning/IK operations

    Implementations:
        - DrakeWorld: Uses Drake's MultibodyPlant and SceneGraph
    """

    # Robot Management
    def add_robot(self, config: RobotModelConfig) -> str:
        """Add a robot to the world. Returns unique robot ID."""
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

    # Obstacle Management
    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle to the world. Returns unique obstacle ID."""
        ...

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle. Returns True if removed."""
        ...

    def update_obstacle_pose(self, obstacle_id: str, pose: NDArray[np.float64]) -> bool:
        """Update obstacle pose. Returns True if updated."""
        ...

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        ...

    # Lifecycle
    def finalize(self) -> None:
        """Finalize the world. Must be called after adding robots."""
        ...

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        ...

    # Context Management
    def get_live_context(self) -> Any:
        """Get the live context (mirrors real robot state)."""
        ...

    def scratch_context(self) -> AbstractContextManager[Any]:
        """Get a scratch context for planning (thread-safe clone)."""
        ...

    def sync_from_joint_state(self, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Sync live context from joint state."""
        ...

    # State Operations (require context)
    def set_positions(self, ctx: Any, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Set robot joint positions in a context."""
        ...

    def get_positions(self, ctx: Any, robot_id: str) -> NDArray[np.float64]:
        """Get robot joint positions from a context."""
        ...

    # Collision Checking (require context)
    def is_collision_free(self, ctx: Any, robot_id: str) -> bool:
        """Check if robot configuration is collision-free."""
        ...

    def get_min_distance(self, ctx: Any, robot_id: str) -> float:
        """Get minimum distance to obstacles (negative if collision)."""
        ...

    # Collision Checking (context-free, for planning)
    def check_config_collision_free(self, robot_id: str, q: NDArray[np.float64]) -> bool:
        """Check if a configuration is collision-free (manages context internally)."""
        ...

    def check_edge_collision_free(
        self,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
        step_size: float = 0.05,
    ) -> bool:
        """Check if the entire edge between two configurations is collision-free."""
        ...

    # Forward Kinematics (require context)
    def get_ee_pose(self, ctx: Any, robot_id: str) -> NDArray[np.float64]:
        """Get end-effector pose (4x4 transform)."""
        ...

    def get_jacobian(self, ctx: Any, robot_id: str) -> NDArray[np.float64]:
        """Get end-effector Jacobian (6 x n_joints)."""
        ...

    # Visualization (optional)
    def get_visualization_url(self) -> str | None:
        """Get visualization URL if enabled."""
        ...

    def publish_visualization(self, ctx: Any | None = None) -> None:
        """Publish current state to visualization."""
        ...

    def animate_path(
        self, robot_id: str, path: list[NDArray[np.float64]], duration: float = 3.0
    ) -> None:
        """Animate a path in visualization."""
        ...


@runtime_checkable
class KinematicsSpec(Protocol):
    """Protocol for inverse kinematics solvers. Stateless, uses WorldSpec for FK/collision."""

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
        """Solve IK with optional collision checking."""
        ...


@runtime_checkable
class PlannerSpec(Protocol):
    """Protocol for motion planner.

    Planners find collision-free paths from start to goal configurations.
    They use WorldSpec for collision checking and are stateless.
    All planners are backend-agnostic - they only use WorldSpec methods.

    Implementations:
        - RRTConnectPlanner: Bi-directional RRT-Connect planner
        - RRTStarPlanner: RRT* planner (asymptotically optimal)
    """

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a collision-free joint-space path."""
        ...

    def get_name(self) -> str:
        """Get planner name."""
        ...
