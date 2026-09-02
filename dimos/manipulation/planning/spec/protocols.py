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
    from dimos.manipulation.planning.planners.config import CartesianPathConfig
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        CartesianTarget,
        GeneratedPlan,
        IKResult,
        Obstacle,
        PlanningGroupID,
        PlanningResult,
        VisualizationSession,
        VisualizationStateFrame,
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

    # Model Management
    def load_model(self, config: RobotModelConfig) -> None:
        """Load the logical robot model."""
        ...

    def get_model_config(self) -> RobotModelConfig:
        """Get the logical robot model configuration."""
        ...

    def get_joint_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get model joint limits (lower, upper)."""
        ...

    # Obstacle Management
    def add_obstacle(self, obstacle: Obstacle) -> str | None:
        """Add an obstacle, returning a non-empty native ID if inserted."""
        ...

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle. Returns True if removed."""
        ...

    def update_obstacle(self, obstacle: Obstacle) -> bool:
        """Replace the complete obstacle identified by obstacle.name."""
        ...

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update only an obstacle pose. Returns True if updated."""
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

    def sync_from_joint_state(self, joint_state: JointState) -> None:
        """Sync live context from joint state message."""
        ...

    # State Operations (require context)
    def set_joint_state(self, ctx: Any, joint_state: JointState) -> None:
        """Set robot joint state in a context."""
        ...

    def get_joint_state(self, ctx: Any) -> JointState:
        """Get robot joint state from a context."""
        ...

    # Collision Checking (require context)
    def is_collision_free(self, ctx: Any) -> bool:
        """Check if robot configuration is collision-free."""
        ...

    def get_min_distance(self, ctx: Any) -> float:
        """Get minimum distance to obstacles (negative if collision)."""
        ...

    # Collision Checking (context-free, for planning)
    def check_config_collision_free(self, joint_state: JointState) -> bool:
        """Check if a joint state is collision-free (manages context internally)."""
        ...

    def check_edge_collision_free(
        self,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check if the entire edge between two joint states is collision-free."""
        ...

    # Forward Kinematics (require context)
    def get_ee_pose(self, ctx: Any) -> PoseStamped:
        """Get end-effector pose."""
        ...

    def get_link_pose(self, ctx: Any, link_name: str) -> NDArray[np.float64]:
        """Get link pose as 4x4 homogeneous transform."""
        ...

    def get_jacobian(self, ctx: Any) -> NDArray[np.float64]:
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

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        """Render or otherwise accept an obstacle added to the planning world."""
        ...

    def update_vis_obstacle(self, obstacle: Obstacle) -> None:
        """Replace a complete obstacle representation."""
        ...

    def update_vis_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> None:
        """Update only an obstacle representation's pose."""
        ...

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        """Remove an obstacle representation from the visualization."""
        ...

    def clear_vis_obstacles(self) -> None:
        """Clear obstacle representations from the visualization."""
        ...

    def get_visualization_url(self) -> str | None:
        """Get visualization URL if enabled."""
        ...

    def update_state(self, frame: VisualizationStateFrame) -> None:
        """Receive the current model joint state."""
        ...

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        """Animate a raw globally named trajectory."""
        ...

    def cancel_preview_animation(self) -> None:
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

    The supplied ``start`` is the authoritative state snapshot for a planning
    request. Implementations must not replace it with, or compare it against,
    a later sample from the live world. Trajectory execution is responsible
    for rejecting a path when the robot has moved too far from its start.

    Implementations:
        - RRTConnectPlanner: Bi-directional RRT-Connect planner
        - RRTStarPlanner: RRT* planner (asymptotically optimal)
    """

    def plan_joint_path(
        self,
        world: WorldSpec,
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

    def plan_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        targets: Mapping[PlanningGroupID, CartesianTarget],
        config: CartesianPathConfig,
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        check_collision: bool = True,
    ) -> PlanningResult:
        """Plan synchronized TCP waypoint paths for an ordered group selection."""
        ...

    def get_name(self) -> str:
        """Get planner name."""
        ...


@runtime_checkable
class TrajectoryParametrizerSpec(Protocol):
    """Convert successful planning output into one canonical generated plan."""

    def materialize_plan(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        result: PlanningResult,
        speed_scale: float = 1.0,
    ) -> GeneratedPlan:
        """Preserve timed output or parametrize an untimed path, then validate it."""
        ...
