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

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    from contextlib import AbstractContextManager

    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        IKResult,
        Obstacle,
        PlanningGroupID,
        PlanningResult,
        VisualizationSession,
        VisualizationStateFrame,
        WorldRobotID,
    )
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


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
    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a robot to the world. Returns unique robot ID."""
        ...

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs."""
        ...

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get robot configuration."""
        ...

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:  # lower limits, upper limits
        """Get joint limits (lower, upper) for a robot."""
        ...

    # Obstacle Management
    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle to the world. Returns unique obstacle ID."""
        ...

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle. Returns True if removed."""
        ...

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update obstacle pose. Returns True if updated."""
        ...

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        ...

    def get_obstacles(self) -> list[Obstacle]:
        """Get all obstacles currently in the world."""
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

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live context from joint state message."""
        ...

    # State Operations (require context)
    def set_joint_state(self, ctx: Any, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Set robot joint state in a context."""
        ...

    def get_joint_state(self, ctx: Any, robot_id: WorldRobotID) -> JointState:
        """Get robot joint state from a context."""
        ...

    # Collision Checking (require context)
    def is_collision_free(self, ctx: Any, robot_id: WorldRobotID) -> bool:
        """Check if robot configuration is collision-free."""
        ...

    def get_min_distance(self, ctx: Any, robot_id: WorldRobotID) -> float:
        """Get minimum distance to obstacles (negative if collision)."""
        ...

    # Collision Checking (context-free, for planning)
    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check if a joint state is collision-free (manages context internally)."""
        ...

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check if the entire edge between two joint states is collision-free."""
        ...

    # Forward Kinematics (require context)
    def get_ee_pose(self, ctx: Any, robot_id: WorldRobotID) -> PoseStamped:
        """Get end-effector pose."""
        ...

    def get_link_pose(
        self, ctx: Any, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        """Get link pose as 4x4 homogeneous transform."""
        ...

    def get_jacobian(self, ctx: Any, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """Get end-effector Jacobian (6 x n_joints)."""
        ...

    def get_group_ee_pose(self, ctx: Any, group_id: PlanningGroupID) -> PoseStamped:
        """Get planning-group tip pose."""
        ...

    def get_group_jacobian(self, ctx: Any, group_id: PlanningGroupID) -> NDArray[np.float64]:
        """Get planning-group Jacobian (6 x n_group_joints)."""
        ...


@runtime_checkable
class VisualizationSpec(Protocol):
    """Protocol for optional manipulation planning visualization.

    Visualization backends expose inspection and preview behavior without being part of
    the world/collision/kinematics contract.

    Implementations may render, ignore, or map semantic events to their native
    visualization affordances.
    """

    def initialize(self, session: VisualizationSession) -> None:
        """Receive one-shot visualization session metadata after world startup."""
        ...

    def get_visualization_url(self) -> str | None:
        """Get visualization URL if enabled."""
        ...

    def update_state(self, frame: VisualizationStateFrame) -> None:
        """Receive current joint states keyed by initialized world robot ID."""
        ...

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        """Animate a raw globally named trajectory."""
        ...

    def cancel_preview_animation(self, robot_ids: Sequence[WorldRobotID] | None = None) -> None:
        """Cancel an active preview animation without waiting for its renderer to finish."""
        ...

    def close(self) -> None:
        """Release visualization resources."""
        ...


@runtime_checkable
class KinematicsSpec(Protocol):
    """Protocol for inverse kinematics solvers. Stateless, uses WorldSpec for FK/collision."""

    def solve(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve IK with optional collision checking."""
        ...

    def solve_pose_targets(
        self,
        world: WorldSpec,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        auxiliary_groups: Sequence[PlanningGroup] = (),
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve planning-group-scoped pose targets."""
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
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a collision-free joint-space path."""
        ...

    def plan_selected_joint_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan a collision-free path for an ordered planning-group selection."""
        ...

    def get_name(self) -> str:
        """Get planner name."""
        ...
