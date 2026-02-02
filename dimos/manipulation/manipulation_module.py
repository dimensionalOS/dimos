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

"""Manipulation Module - Motion planning with ControlCoordinator execution."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import threading
from typing import TYPE_CHECKING, TypeAlias

from dimos.core import In, Module, rpc
from dimos.core.module import ModuleConfig
from dimos.manipulation.planning import (
    JointPath,
    JointTrajectoryGenerator,
    KinematicsSpec,
    Obstacle,
    ObstacleType,
    PlannerSpec,
    RobotModelConfig,
    RobotName,
    WorldRobotID,
    create_kinematics,
    create_planner,
)
from dimos.manipulation.planning.monitor import WorldMonitor

# These must be imported at runtime (not TYPE_CHECKING) for In/Out port creation
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.trajectory_msgs import JointTrajectory
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import RPCClient
    from dimos.msgs.geometry_msgs import Pose

logger = setup_logger()

# Composite type aliases for readability (using semantic IDs from planning.spec)
RobotEntry: TypeAlias = tuple[WorldRobotID, RobotModelConfig, JointTrajectoryGenerator]
"""(world_robot_id, config, trajectory_generator)"""

RobotRegistry: TypeAlias = dict[RobotName, RobotEntry]
"""Maps robot_name -> RobotEntry"""

PlannedPaths: TypeAlias = dict[RobotName, JointPath]
"""Maps robot_name -> planned joint path"""

PlannedTrajectories: TypeAlias = dict[RobotName, JointTrajectory]
"""Maps robot_name -> planned trajectory"""


class ManipulationState(Enum):
    """State machine for manipulation module."""

    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    COMPLETED = 3
    FAULT = 4


@dataclass
class ManipulationModuleConfig(ModuleConfig):
    """Configuration for ManipulationModule."""

    robots: list[RobotModelConfig] = field(default_factory=list)
    planning_timeout: float = 10.0
    enable_viz: bool = False
    planner_name: str = "rrt_connect"  # "rrt_connect"
    kinematics_name: str = "jacobian"  # "jacobian" or "drake_optimization"


class ManipulationModule(Module):
    """Motion planning module with ControlCoordinator execution."""

    default_config = ManipulationModuleConfig

    # Type annotation for the config attribute (mypy uses this)
    config: ManipulationModuleConfig

    # Input: Joint state from coordinator (for world sync)
    joint_state: In[JointState]

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

        # State machine
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner: PlannerSpec | None = None
        self._kinematics: KinematicsSpec | None = None

        # Robot registry: maps robot_name -> (world_robot_id, config, trajectory_gen)
        self._robots: RobotRegistry = {}

        # Stored path for plan/preview/execute workflow (per robot)
        self._planned_paths: PlannedPaths = {}
        self._planned_trajectories: PlannedTrajectories = {}

        # Coordinator integration (lazy initialized)
        self._coordinator_client: RPCClient | None = None

        logger.info("ManipulationModule initialized")

    @rpc
    def start(self) -> None:
        """Start the manipulation module."""
        super().start()

        # Initialize planning stack
        self._initialize_planning()

        # Subscribe to joint state via port
        if self.joint_state is not None:
            self.joint_state.subscribe(self._on_joint_state)
            logger.info("Subscribed to joint_state port")

        logger.info("ManipulationModule started")

    def _initialize_planning(self) -> None:
        """Initialize world, planner, and trajectory generator."""
        if not self.config.robots:
            logger.warning("No robots configured, planning disabled")
            return

        self._world_monitor = WorldMonitor(enable_viz=self.config.enable_viz)

        for robot_config in self.config.robots:
            robot_id = self._world_monitor.add_robot(robot_config)
            traj_gen = JointTrajectoryGenerator(
                num_joints=len(robot_config.joint_names),
                max_velocity=robot_config.max_velocity,
                max_acceleration=robot_config.max_acceleration,
            )
            self._robots[robot_config.name] = (robot_id, robot_config, traj_gen)

        self._world_monitor.finalize()

        for _, (robot_id, _, _) in self._robots.items():
            self._world_monitor.start_state_monitor(robot_id)

        if self.config.enable_viz:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info(f"Visualization: {url}")

        self._planner = create_planner(name=self.config.planner_name)
        self._kinematics = create_kinematics(name=self.config.kinematics_name)

    def _get_default_robot_name(self) -> RobotName | None:
        """Get default robot name (first robot if only one, else None)."""
        if len(self._robots) == 1:
            return next(iter(self._robots.keys()))
        return None

    def _get_robot(
        self, robot_name: RobotName | None = None
    ) -> tuple[RobotName, WorldRobotID, RobotModelConfig, JointTrajectoryGenerator] | None:
        """Get robot by name or default.

        Args:
            robot_name: Robot name or None for default (if single robot)

        Returns:
            (robot_name, robot_id, config, traj_gen) or None if not found
        """
        if robot_name is None:
            robot_name = self._get_default_robot_name()
            if robot_name is None:
                logger.error("Multiple robots configured, must specify robot_name")
                return None

        if robot_name not in self._robots:
            logger.error(f"Unknown robot: {robot_name}")
            return None

        robot_id, config, traj_gen = self._robots[robot_name]
        return (robot_name, robot_id, config, traj_gen)

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback when joint state received from driver."""
        try:
            # Forward to world monitor for state synchronization.
            # Pass robot_id=None to broadcast to all monitors - each monitor
            # extracts only its robot's joints based on joint_name_mapping.
            if self._world_monitor is not None:
                self._world_monitor.on_joint_state(msg, robot_id=None)

        except Exception as e:
            logger.error(f"Exception in _on_joint_state: {e}")
            import traceback

            logger.error(traceback.format_exc())

    # =========================================================================
    # RPC Methods
    # =========================================================================

    @rpc
    def get_state(self) -> str:
        """Get current manipulation state name."""
        return self._state.name

    @rpc
    def get_error(self) -> str:
        """Get last error message.

        Returns:
            Error message or empty string
        """
        return self._error_message

    @rpc
    def cancel(self) -> bool:
        """Cancel current motion."""
        if self._state != ManipulationState.EXECUTING:
            return False
        self._state = ManipulationState.IDLE
        logger.info("Motion cancelled")
        return True

    @rpc
    def reset(self) -> bool:
        """Reset to IDLE state (fails if EXECUTING)."""
        if self._state == ManipulationState.EXECUTING:
            return False
        self._state = ManipulationState.IDLE
        self._error_message = ""
        return True

    @rpc
    def get_current_joints(self, robot_name: RobotName | None = None) -> list[float] | None:
        """Get current joint positions.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            state = self._world_monitor.get_current_joint_state(robot[1])
            if state is not None:
                return list(state.position)
        return None

    @rpc
    def get_ee_pose(self, robot_name: RobotName | None = None) -> Pose | None:
        """Get current end-effector pose.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            return self._world_monitor.get_ee_pose(robot[1], joint_state=None)
        return None

    @rpc
    def is_collision_free(self, joints: list[float], robot_name: RobotName | None = None) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check
            robot_name: Robot to check (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            _, robot_id, config, _ = robot
            joint_state = JointState(name=config.joint_names, position=joints)
            return self._world_monitor.is_state_valid(robot_id, joint_state)
        return False

    # =========================================================================
    # Plan/Preview/Execute Workflow RPC Methods
    # =========================================================================

    def _begin_planning(
        self, robot_name: RobotName | None = None
    ) -> tuple[RobotName, WorldRobotID] | None:
        """Check state and begin planning. Returns (robot_name, robot_id) or None.

        Args:
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return None
        if (robot := self._get_robot(robot_name)) is None:
            return None
        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot plan: state is {self._state.name}")
                return None
            self._state = ManipulationState.PLANNING
        return robot[0], robot[1]

    def _fail(self, msg: str) -> bool:
        """Set FAULT state with error message."""
        logger.warning(msg)
        self._state = ManipulationState.FAULT
        self._error_message = msg
        return False

    @rpc
    def plan_to_pose(self, pose: Pose, robot_name: RobotName | None = None) -> bool:
        """Plan motion to pose. Use preview_path() then execute().

        Args:
            pose: Target end-effector pose
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._kinematics is None or (r := self._begin_planning(robot_name)) is None:
            return False
        robot_name, robot_id = r
        assert self._world_monitor  # guaranteed by _begin_planning

        current = self._world_monitor.get_current_joint_state(robot_id)
        if current is None:
            return self._fail("No joint state")

        # Convert Pose to PoseStamped for the IK solver
        from dimos.msgs.geometry_msgs import PoseStamped

        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )

        ik = self._kinematics.solve(
            world=self._world_monitor.world,
            robot_id=robot_id,
            target_pose=target_pose,
            seed=current,
            check_collision=True,
        )
        if not ik.is_success() or ik.joint_state is None:
            return self._fail(f"IK failed: {ik.status.name}")

        logger.info(f"IK solved, error: {ik.position_error:.4f}m")
        return self._plan_path_only(robot_name, robot_id, ik.joint_state)

    @rpc
    def plan_to_joints(self, joints: list[float], robot_name: RobotName | None = None) -> bool:
        """Plan motion to joint config. Use preview_path() then execute().

        Args:
            joints: Target joint configuration
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if (r := self._begin_planning(robot_name)) is None:
            return False
        robot_name, robot_id = r
        _, config, _ = self._robots[robot_name]
        goal_state = JointState(name=config.joint_names, position=joints)
        logger.info(f"Planning to joints for {robot_name}: {[f'{j:.3f}' for j in joints]}")
        return self._plan_path_only(robot_name, robot_id, goal_state)

    def _plan_path_only(
        self, robot_name: RobotName, robot_id: WorldRobotID, goal: JointState
    ) -> bool:
        """Plan path from current position to goal, store result."""
        assert self._world_monitor and self._planner  # guaranteed by _begin_planning
        start = self._world_monitor.get_current_joint_state(robot_id)
        if start is None:
            return self._fail("No joint state")

        result = self._planner.plan_joint_path(
            world=self._world_monitor.world,
            robot_id=robot_id,
            start=start,
            goal=goal,
            timeout=self.config.planning_timeout,
        )
        if not result.is_success():
            return self._fail(f"Planning failed: {result.status.name}")

        logger.info(f"Path: {len(result.path)} waypoints")
        self._planned_paths[robot_name] = result.path

        _, _, traj_gen = self._robots[robot_name]
        # Convert JointState path to list of position lists for trajectory generator
        traj = traj_gen.generate([list(state.position) for state in result.path])
        self._planned_trajectories[robot_name] = traj
        logger.info(f"Trajectory: {traj.duration:.3f}s")

        self._state = ManipulationState.COMPLETED
        return True

    @rpc
    def preview_path(self, duration: float = 3.0, robot_name: RobotName | None = None) -> bool:
        """Preview the planned path in the visualizer.

        Args:
            duration: Total animation duration in seconds
            robot_name: Robot to preview (required if multiple robots configured)
        """
        from dimos.manipulation.planning.utils.path_utils import interpolate_path

        if self._world_monitor is None:
            return False

        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        robot_name, robot_id, _, _ = robot

        planned_path = self._planned_paths.get(robot_name)
        if planned_path is None or len(planned_path) == 0:
            logger.warning(f"No planned path to preview for {robot_name}")
            return False

        # Interpolate and animate
        interpolated = interpolate_path(planned_path, resolution=0.02)
        self._world_monitor.world.animate_path(robot_id, interpolated, duration)
        return True

    @rpc
    def has_planned_path(self) -> bool:
        """Check if there's a planned path ready.

        Returns:
            True if a path is planned and ready
        """
        robot = self._get_robot()
        if robot is None:
            return False
        robot_name, _, _, _ = robot

        path = self._planned_paths.get(robot_name)
        return path is not None and len(path) > 0

    @rpc
    def get_visualization_url(self) -> str | None:
        """Get the visualization URL.

        Returns:
            URL string or None if visualization not enabled
        """
        if self._world_monitor is None:
            return None
        return self._world_monitor.get_visualization_url()

    @rpc
    def clear_planned_path(self) -> bool:
        """Clear the stored planned path.

        Returns:
            True if cleared
        """
        robot = self._get_robot()
        if robot is None:
            return False
        robot_name, _, _, _ = robot

        self._planned_paths.pop(robot_name, None)
        self._planned_trajectories.pop(robot_name, None)
        return True

    @rpc
    def list_robots(self) -> list[str]:
        """List all configured robot names.

        Returns:
            List of robot names
        """
        return list(self._robots.keys())

    @rpc
    def get_robot_info(self, robot_name: RobotName | None = None) -> dict[str, object] | None:
        """Get information about a robot.

        Args:
            robot_name: Robot name (uses default if None)

        Returns:
            Dict with robot info or None if not found
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return None

        robot_name, robot_id, config, _ = robot

        return {
            "name": config.name,
            "world_robot_id": robot_id,
            "joint_names": config.joint_names,
            "end_effector_link": config.end_effector_link,
            "base_link": config.base_link,
            "max_velocity": config.max_velocity,
            "max_acceleration": config.max_acceleration,
            "has_joint_name_mapping": bool(config.joint_name_mapping),
            "coordinator_task_name": config.coordinator_task_name,
        }

    # =========================================================================
    # Coordinator Integration RPC Methods
    # =========================================================================

    def _get_coordinator_client(self) -> RPCClient | None:
        """Get or create coordinator RPC client (lazy init)."""
        if not any(c.coordinator_task_name for _, c, _ in self._robots.values()):
            return None
        if self._coordinator_client is None:
            from dimos.control.coordinator import ControlCoordinator
            from dimos.core.rpc_client import RPCClient

            self._coordinator_client = RPCClient(None, ControlCoordinator)
        return self._coordinator_client

    def _translate_trajectory_to_coordinator(
        self,
        trajectory: JointTrajectory,
        robot_config: RobotModelConfig,
    ) -> JointTrajectory:
        """Translate trajectory joint names from URDF to coordinator namespace.

        Args:
            trajectory: Trajectory with URDF joint names
            robot_config: Robot config with joint name mapping

        Returns:
            Trajectory with coordinator joint names
        """
        if not robot_config.joint_name_mapping:
            return trajectory  # No translation needed

        # Translate joint names
        coordinator_names = [
            robot_config.get_coordinator_joint_name(j) for j in trajectory.joint_names
        ]

        # Create new trajectory with translated names
        # Note: duration is computed automatically from points in JointTrajectory.__init__
        return JointTrajectory(
            joint_names=coordinator_names,
            points=trajectory.points,
            timestamp=trajectory.timestamp,
        )

    @rpc
    def execute(self, robot_name: RobotName | None = None) -> bool:
        """Execute planned trajectory via ControlCoordinator."""
        if (robot := self._get_robot(robot_name)) is None:
            return False
        robot_name, _, config, _ = robot

        if (traj := self._planned_trajectories.get(robot_name)) is None:
            logger.warning("No planned trajectory")
            return False
        if not config.coordinator_task_name:
            logger.error(f"No coordinator_task_name for '{robot_name}'")
            return False
        if (client := self._get_coordinator_client()) is None:
            logger.error("No coordinator client")
            return False

        translated = self._translate_trajectory_to_coordinator(traj, config)
        logger.info(
            f"Executing: task='{config.coordinator_task_name}', {len(translated.points)} pts, {translated.duration:.2f}s"
        )

        self._state = ManipulationState.EXECUTING
        if client.execute_trajectory(config.coordinator_task_name, translated):
            logger.info("Trajectory accepted")
            self._state = ManipulationState.COMPLETED
            return True
        else:
            return self._fail("Coordinator rejected trajectory")

    @rpc
    def get_trajectory_status(
        self, robot_name: RobotName | None = None
    ) -> dict[str, object] | None:
        """Get trajectory execution status."""
        if (robot := self._get_robot(robot_name)) is None:
            return None
        _, _, config, _ = robot
        if not config.coordinator_task_name or (client := self._get_coordinator_client()) is None:
            return None
        status = client.get_trajectory_status(config.coordinator_task_name)
        return dict(status) if status else None

    @property
    def world_monitor(self) -> WorldMonitor | None:
        """Access the world monitor for advanced obstacle/world operations."""
        return self._world_monitor

    @rpc
    def add_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: str,
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
    ) -> str:
        """Add obstacle: shape='box'|'sphere'|'cylinder'|'mesh'. Returns obstacle_id."""
        if not self._world_monitor:
            return ""

        # Map shape string to ObstacleType
        shape_map = {
            "box": ObstacleType.BOX,
            "sphere": ObstacleType.SPHERE,
            "cylinder": ObstacleType.CYLINDER,
            "mesh": ObstacleType.MESH,
        }
        obstacle_type = shape_map.get(shape)
        if obstacle_type is None:
            logger.warning(f"Unknown obstacle shape: {shape}")
            return ""

        # Validate mesh_path for mesh type
        if obstacle_type == ObstacleType.MESH and not mesh_path:
            logger.warning("mesh_path required for mesh obstacles")
            return ""

        # Import PoseStamped here to avoid circular imports
        from dimos.msgs.geometry_msgs import PoseStamped

        obstacle = Obstacle(
            name=name,
            obstacle_type=obstacle_type,
            pose=PoseStamped(position=pose.position, orientation=pose.orientation),
            dimensions=tuple(dimensions) if dimensions else (),
            mesh_path=mesh_path,
        )
        return self._world_monitor.add_obstacle(obstacle)

    @rpc
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the planning world."""
        if self._world_monitor is None:
            return False
        return self._world_monitor.remove_obstacle(obstacle_id)

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        # Stop world monitor (includes visualization thread)
        if self._world_monitor is not None:
            self._world_monitor.stop_all_monitors()

        super().stop()


# Expose blueprint for declarative composition
manipulation_module = ManipulationModule.blueprint
