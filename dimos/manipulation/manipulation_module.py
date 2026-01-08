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
Manipulation Module

A dimos Module that integrates Drake motion planning into the streaming architecture.
Coordinates planning, trajectory generation, and execution for robotic manipulation.

## Architecture

```
Driver (joint_state) → ManipulationModule → JointTrajectory → Controller → Driver
```

## Usage

```python
from dimos.manipulation import ManipulationModule

# Deploy module
manip = cluster.deploy(ManipulationModule, config={
    "robot_urdf_path": "/path/to/robot.urdf",
    "robot_name": "xarm6",
    "joint_names": ["joint1", "joint2", ...],
    "end_effector_link": "link6",
})

# Configure transports
manip.joint_state.transport = LCMTransport("/joint_state", JointState)
manip.trajectory.transport = LCMTransport("/trajectory", JointTrajectory)

# Start and use
manip.start()
manip.move_to_pose(0.3, 0.0, 0.4, 0.0, np.pi, 0.0)
```
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.manipulation.planning import (
    JointTrajectoryGenerator,
    RobotModelConfig,
    create_kinematics,
    create_planner,
)
from dimos.manipulation.planning.monitor import WorldMonitor
from dimos.manipulation.utils import pose_from_xyzrpy

# These must be imported at runtime (not TYPE_CHECKING) for In/Out port creation
from dimos.msgs.sensor_msgs import JointState  # noqa: TC001
from dimos.msgs.trajectory_msgs import JointTrajectory  # noqa: TC001
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


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

    robot_urdf_path: str = ""
    robot_name: str = "robot"
    joint_names: list[str] = field(default_factory=list)
    end_effector_link: str = "ee_link"
    base_link: str = "base_link"
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    planning_timeout: float = 10.0
    enable_viz: bool = False
    # For xacro files that use $(find package_name)
    package_paths: dict[str, str] = field(default_factory=dict)
    xacro_args: dict[str, str] = field(default_factory=dict)
    # Collision exclusion pairs for parallel linkage mechanisms (e.g., grippers)
    collision_exclusion_pairs: list[tuple[str, str]] = field(default_factory=list)


class ManipulationModule(Module):
    """Motion planning module using Drake backend.

    This module:
    1. Subscribes to joint_state from the arm driver
    2. Maintains a planning world synchronized with real robot state
    3. Plans collision-free paths using RRT-Connect
    4. Generates time-parameterized trajectories
    5. Publishes trajectories for the controller to execute

    ## State Machine

    ```
    IDLE → (move_to_*) → PLANNING → (success) → EXECUTING → (done) → COMPLETED
                              ↓ (fail)                         ↓ (cancel)
                           FAULT ←─────────────────────────────┘
    ```

    ## Inputs
    - joint_state: JointState from driver (100Hz)

    ## Outputs
    - trajectory: JointTrajectory to controller

    ## RPC Methods
    - move_to_pose(x, y, z, roll, pitch, yaw) -> bool
    - move_to_joints(joints) -> bool
    - get_state() -> int
    - cancel() -> bool
    - reset() -> bool
    """

    default_config = ManipulationModuleConfig

    # Input: Joint state feedback from driver
    joint_state: In[JointState] = None

    # Output: Trajectory to controller
    trajectory: Out[JointTrajectory] = None

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # State machine
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner = None
        self._kinematics = None
        self._trajectory_gen: JointTrajectoryGenerator | None = None
        self._robot_id: str = ""

        # Stored path for plan/preview/execute workflow
        self._planned_path: list[NDArray[np.float64]] | None = None
        self._planned_trajectory: JointTrajectory | None = None

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
        """Initialize Drake world, planner, and trajectory generator."""
        config = self.config

        if not config.robot_urdf_path:
            logger.warning("No robot_urdf_path configured, planning disabled")
            return

        if not config.joint_names:
            logger.warning("No joint_names configured, planning disabled")
            return

        logger.info(f"Initializing planning for {config.robot_name}")

        # Create robot configuration
        robot_config = RobotModelConfig(
            name=config.robot_name,
            urdf_path=config.robot_urdf_path,
            base_pose=np.eye(4),
            joint_names=config.joint_names,
            end_effector_link=config.end_effector_link,
            base_link=config.base_link,
            package_paths=config.package_paths,
            xacro_args=config.xacro_args,
            auto_convert_meshes=True,
            collision_exclusion_pairs=config.collision_exclusion_pairs,
        )

        # Create world monitor
        self._world_monitor = WorldMonitor(enable_viz=config.enable_viz)
        self._robot_id = self._world_monitor.add_robot(robot_config)
        self._world_monitor.finalize()

        # Start state monitor to receive joint state updates
        self._world_monitor.start_state_monitor(self._robot_id, config.joint_names)
        logger.info(f"State monitor started for '{self._robot_id}'")

        if config.enable_viz:
            url = self._world_monitor.get_meshcat_url()
            if url:
                logger.info(f"Meshcat visualization: {url}")
            # Start visualization thread for live updates (10Hz to avoid overhead)
            self._world_monitor.start_visualization_thread(rate_hz=10.0)

        # Create planner
        self._planner = create_planner(name="rrt_connect")

        # Create kinematics solver
        self._kinematics = create_kinematics(backend="drake")

        # Create trajectory generator
        self._trajectory_gen = JointTrajectoryGenerator(
            num_joints=len(config.joint_names),
            max_velocity=config.max_velocity,
            max_acceleration=config.max_acceleration,
        )

        logger.info(
            f"Planning initialized for {config.robot_name} ({len(config.joint_names)} joints)"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback when joint state received from driver."""
        try:
            # Store latest positions
            self._latest_joint_positions = list(msg.position[:6])

            # Periodic debug logging (every 100 messages)
            self._js_count = getattr(self, "_js_count", 0) + 1
            if self._js_count % 100 == 0:
                logger.debug(
                    f"[JointState #{self._js_count}] positions: {[f'{p:.3f}' for p in self._latest_joint_positions]}"
                )

            # Forward to world monitor for Drake state synchronization
            if self._world_monitor is not None:
                self._world_monitor.on_joint_state(msg, self._robot_id)

        except Exception as e:
            logger.error(f"[ManipulationModule] Exception in _on_joint_state: {e}")
            import traceback

            logger.error(traceback.format_exc())

    # =========================================================================
    # RPC Methods
    # =========================================================================

    @rpc
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> bool:
        """Plan and execute motion to end-effector pose.

        Args:
            x: X position in meters
            y: Y position in meters
            z: Z position in meters
            roll: Rotation about X axis in radians
            pitch: Rotation about Y axis in radians
            yaw: Rotation about Z axis in radians

        Returns:
            True if planning and trajectory publish succeeded
        """
        if self._world_monitor is None or self._kinematics is None:
            logger.error("Planning not initialized")
            return False

        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot move: state is {self._state.name}")
                return False
            self._state = ManipulationState.PLANNING

        logger.info(
            f"Planning motion to pose ({x:.3f}, {y:.3f}, {z:.3f}, {roll:.2f}, {pitch:.2f}, {yaw:.2f})"
        )

        # Build 4x4 pose matrix from xyzrpy
        target_pose = pose_from_xyzrpy(x, y, z, roll, pitch, yaw)

        # Get current joint positions as seed
        current = self._world_monitor.get_current_positions(self._robot_id)
        if current is None:
            logger.error("No current joint state available")
            self._state = ManipulationState.FAULT
            self._error_message = "No joint state"
            return False

        # Solve IK
        ik_result = self._kinematics.solve(
            world=self._world_monitor.world,
            robot_id=self._robot_id,
            target_pose=target_pose,
            seed=current,
            check_collision=True,
        )

        if not ik_result.is_success():
            logger.warning(f"IK failed: {ik_result.status.name}")
            self._state = ManipulationState.FAULT
            self._error_message = f"IK failed: {ik_result.status.name}"
            return False

        logger.info(f"IK solution found, position error: {ik_result.position_error:.4f}m")

        return self._plan_and_execute(ik_result.joint_positions)

    @rpc
    def move_to_joints(self, joints: list[float]) -> bool:
        """Plan and execute motion to joint configuration.

        Args:
            joints: Target joint positions in radians

        Returns:
            True if planning and trajectory publish succeeded
        """
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return False

        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot move: state is {self._state.name}")
                return False
            self._state = ManipulationState.PLANNING

        logger.info(f"Planning motion to joints: {[f'{j:.3f}' for j in joints]}")

        return self._plan_and_execute(np.array(joints))

    def _plan_and_execute(self, goal: NDArray[np.float64]) -> bool:
        """Internal: plan path and publish trajectory.

        Args:
            goal: Goal joint configuration

        Returns:
            True if planning and trajectory publish succeeded
        """
        if self._world_monitor is None or self._planner is None or self._trajectory_gen is None:
            self._state = ManipulationState.FAULT
            return False

        # Get current position
        start = self._world_monitor.get_current_positions(self._robot_id)
        if start is None:
            logger.error("No current joint state available")
            self._state = ManipulationState.FAULT
            self._error_message = "No joint state"
            return False

        # DEBUG: Log the start position being used for planning
        logger.info(f"[DEBUG] Planning from q_start: {start}")
        logger.info(f"[DEBUG] Planning to q_goal: {goal}")

        # Plan collision-free path
        result = self._planner.plan_joint_path(
            world=self._world_monitor.world,
            robot_id=self._robot_id,
            q_start=start,
            q_goal=goal,
            timeout=self.config.planning_timeout,
        )

        if not result.is_success():
            logger.warning(f"Planning failed: {result.status.name}")
            self._state = ManipulationState.FAULT
            self._error_message = f"Planning failed: {result.status.name}"
            return False

        logger.info(f"Path found with {len(result.path)} waypoints")

        # Generate time-parameterized trajectory
        waypoints = [list(q) for q in result.path]
        trajectory = self._trajectory_gen.generate(waypoints)

        logger.info(
            f"Trajectory generated: {trajectory.duration:.3f}s duration, {len(trajectory.points)} points"
        )

        # Publish trajectory
        self._state = ManipulationState.EXECUTING
        if self.trajectory is not None:
            self.trajectory.publish(trajectory)
            logger.info("Trajectory published")

        # Note: We immediately transition to COMPLETED since we don't have
        # feedback from the controller about execution completion.
        # A more sophisticated implementation would wait for controller feedback.
        self._state = ManipulationState.COMPLETED
        return True

    @rpc
    def get_state(self) -> int:
        """Get current manipulation state.

        Returns:
            State value (0=IDLE, 1=PLANNING, 2=EXECUTING, 3=COMPLETED, 4=FAULT)
        """
        return self._state.value

    @rpc
    def get_state_name(self) -> str:
        """Get current manipulation state name.

        Returns:
            State name string
        """
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
        """Cancel current motion.

        Returns:
            True if cancelled, False if no active motion
        """
        with self._lock:
            if self._state == ManipulationState.EXECUTING:
                self._state = ManipulationState.IDLE
                logger.info("Motion cancelled")
                return True
            return False

    @rpc
    def reset(self) -> bool:
        """Reset from FAULT or COMPLETED state to IDLE.

        Returns:
            True if reset, False if currently EXECUTING
        """
        with self._lock:
            if self._state == ManipulationState.EXECUTING:
                logger.warning("Cannot reset while executing")
                return False
            self._state = ManipulationState.IDLE
            self._error_message = ""
            logger.info("State reset to IDLE")
            return True

    @rpc
    def get_current_joints(self) -> list[float] | None:
        """Get current joint positions.

        Returns:
            List of joint positions or None if not available
        """
        if self._world_monitor is None:
            return None
        positions = self._world_monitor.get_current_positions(self._robot_id)
        if positions is None:
            return None
        return list(positions)

    @rpc
    def get_ee_pose(self) -> list[float] | None:
        """Get current end-effector pose as [x, y, z, roll, pitch, yaw].

        Returns:
            List of [x, y, z, roll, pitch, yaw] or None if not available
        """
        if self._world_monitor is None:
            return None

        from dimos.manipulation.utils import xyzrpy_from_pose

        pose = self._world_monitor.get_ee_pose(self._robot_id)
        x, y, z, roll, pitch, yaw = xyzrpy_from_pose(pose)
        return [x, y, z, roll, pitch, yaw]

    @rpc
    def is_collision_free(self, joints: list[float]) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check

        Returns:
            True if collision-free
        """
        if self._world_monitor is None:
            return False
        return self._world_monitor.is_state_valid(self._robot_id, np.array(joints))

    # =========================================================================
    # Plan/Preview/Execute Workflow RPC Methods
    # =========================================================================

    @rpc
    def plan_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> bool:
        """Plan motion to pose WITHOUT executing.

        Use preview_path_in_drake() to visualize, then execute_planned() to run.

        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians

        Returns:
            True if planning succeeded
        """
        if self._world_monitor is None or self._kinematics is None:
            logger.error("Planning not initialized")
            return False

        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot plan: state is {self._state.name}")
                return False
            self._state = ManipulationState.PLANNING

        logger.info(
            f"Planning to pose ({x:.3f}, {y:.3f}, {z:.3f}, {roll:.2f}, {pitch:.2f}, {yaw:.2f})"
        )

        # Build pose matrix
        target_pose = pose_from_xyzrpy(x, y, z, roll, pitch, yaw)

        # Get current positions
        current = self._world_monitor.get_current_positions(self._robot_id)
        if current is None:
            logger.error("No current joint state available")
            self._state = ManipulationState.FAULT
            self._error_message = "No joint state"
            return False

        # Solve IK
        ik_result = self._kinematics.solve(
            world=self._world_monitor.world,
            robot_id=self._robot_id,
            target_pose=target_pose,
            seed=current,
            check_collision=True,
        )

        if not ik_result.is_success():
            logger.warning(f"IK failed: {ik_result.status.name}")
            self._state = ManipulationState.FAULT
            self._error_message = f"IK failed: {ik_result.status.name}"
            return False

        logger.info(f"IK solution found, position error: {ik_result.position_error:.4f}m")

        return self._plan_path_only(ik_result.joint_positions)

    @rpc
    def plan_to_joints(self, joints: list[float]) -> bool:
        """Plan motion to joint configuration WITHOUT executing.

        Use preview_path_in_drake() to visualize, then execute_planned() to run.

        Args:
            joints: Target joint positions in radians

        Returns:
            True if planning succeeded
        """
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return False

        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot plan: state is {self._state.name}")
                return False
            self._state = ManipulationState.PLANNING

        logger.info(f"Planning to joints: {[f'{j:.3f}' for j in joints]}")

        return self._plan_path_only(np.array(joints))

    def _plan_path_only(self, goal: NDArray[np.float64]) -> bool:
        """Internal: plan path and store it (don't publish trajectory).

        Args:
            goal: Goal joint configuration

        Returns:
            True if planning succeeded
        """
        if self._world_monitor is None or self._planner is None or self._trajectory_gen is None:
            self._state = ManipulationState.FAULT
            return False

        # Get current position
        start = self._world_monitor.get_current_positions(self._robot_id)
        if start is None:
            logger.error("No current joint state available")
            self._state = ManipulationState.FAULT
            self._error_message = "No joint state"
            return False

        # DEBUG: Log the start position being used for planning
        logger.info(f"[DEBUG] Planning from q_start: {start}")
        logger.info(f"[DEBUG] Planning to q_goal: {goal}")

        # Plan collision-free path
        result = self._planner.plan_joint_path(
            world=self._world_monitor.world,
            robot_id=self._robot_id,
            q_start=start,
            q_goal=goal,
            timeout=self.config.planning_timeout,
        )

        if not result.is_success():
            logger.warning(f"Planning failed: {result.status.name}")
            self._state = ManipulationState.FAULT
            self._error_message = f"Planning failed: {result.status.name}"
            return False

        logger.info(f"Path found with {len(result.path)} waypoints")

        # Store path and generate trajectory
        self._planned_path = result.path
        waypoints = [list(q) for q in result.path]
        self._planned_trajectory = self._trajectory_gen.generate(waypoints)

        logger.info(f"Trajectory generated: {self._planned_trajectory.duration:.3f}s duration")

        # Stay in COMPLETED state (path is ready but not executing)
        self._state = ManipulationState.COMPLETED
        return True

    @rpc
    def preview_path_in_drake(
        self,
        speed: float = 1.0,
        duration: float | None = None,
        interpolation_resolution: float = 0.02,
    ) -> bool:
        """Preview the planned path in Drake/Meshcat visualizer.

        The path is interpolated for smooth animation. You can control playback
        either by speed multiplier or by specifying a total duration.

        Args:
            speed: Playback speed multiplier (1.0 = ~3 second animation). Ignored if duration is set.
            duration: Total animation duration in seconds. Overrides speed if set.
            interpolation_resolution: Resolution for path interpolation (radians).
                Smaller = smoother but more frames.

        Returns:
            True if preview succeeded
        """
        import time

        from dimos.manipulation.planning.utils.path_utils import interpolate_path

        if self._planned_path is None or len(self._planned_path) == 0:
            logger.warning("No planned path to preview")
            return False

        if self._world_monitor is None:
            return False

        # Interpolate path for smooth animation
        interpolated = interpolate_path(self._planned_path, resolution=interpolation_resolution)
        num_frames = len(interpolated)

        # Calculate frame delay
        if duration is not None:
            # Use specified duration
            total_time = duration
        else:
            # Default: ~3 seconds at speed=1.0
            base_duration = 3.0
            total_time = base_duration / speed

        dt = total_time / max(num_frames - 1, 1)

        logger.info(
            f"Previewing path: {len(self._planned_path)} waypoints -> "
            f"{num_frames} frames, {total_time:.1f}s duration"
        )

        # Animate through interpolated waypoints
        for q in interpolated:
            # Update scratch context and publish it directly to meshcat
            with self._world_monitor.scratch_context() as ctx:
                self._world_monitor.world.set_positions(ctx, self._robot_id, q)
                # Publish this specific context to meshcat (not the live context)
                self._world_monitor.world.publish_to_meshcat(ctx)
            time.sleep(dt)

        logger.info("Preview complete")
        return True

    @rpc
    def execute_planned(self) -> bool:
        """Execute the planned trajectory (publish to controller).

        Returns:
            True if trajectory was published
        """
        if self._planned_trajectory is None:
            logger.warning("No planned trajectory to execute")
            return False

        logger.info("Executing planned trajectory")

        self._state = ManipulationState.EXECUTING
        if self.trajectory is not None:
            self.trajectory.publish(self._planned_trajectory)
            logger.info("Trajectory published")

        self._state = ManipulationState.COMPLETED
        return True

    @rpc
    def has_planned_path(self) -> bool:
        """Check if there's a planned path ready.

        Returns:
            True if a path is planned and ready
        """
        return self._planned_path is not None and len(self._planned_path) > 0

    @rpc
    def get_visualization_url(self) -> str | None:
        """Get the Meshcat visualization URL.

        Returns:
            URL string or None if visualization not enabled
        """
        if self._world_monitor is None:
            return None
        return self._world_monitor.get_meshcat_url()

    @rpc
    def clear_planned_path(self) -> bool:
        """Clear the stored planned path.

        Returns:
            True if cleared
        """
        self._planned_path = None
        self._planned_trajectory = None
        return True

    @rpc
    def get_debug_info(self) -> dict:
        """Get debug info about joint state flow.

        Returns:
            Dict with debug information
        """
        info = {
            "robot_id": self._robot_id,
            "world_monitor_exists": self._world_monitor is not None,
            "state_monitor_exists": False,
            "state_monitor_running": False,
            "state_age": None,
            "js_msg_count": getattr(self, "_js_count", 0),
            "module_positions": None,
            "monitor_positions": None,
            "live_context_positions": None,
        }

        # Positions stored directly in ManipulationModule
        if hasattr(self, "_latest_joint_positions") and self._latest_joint_positions:
            info["module_positions"] = [round(p, 4) for p in self._latest_joint_positions]

        if self._world_monitor is not None:
            state_monitor = self._world_monitor.get_state_monitor(self._robot_id)
            info["state_monitor_exists"] = state_monitor is not None
            if state_monitor is not None:
                info["state_monitor_running"] = state_monitor.is_running()
                age = state_monitor.get_state_age()
                info["state_age"] = round(age, 3) if age is not None else None
                info["wsm_msg_count"] = getattr(state_monitor, "_msg_count", 0)
                positions = state_monitor.get_current_positions()
                if positions is not None:
                    info["monitor_positions"] = [round(p, 4) for p in positions]

            # Also get positions from live context (fallback that always works)
            live_positions = self._world_monitor.get_current_positions(self._robot_id)
            if live_positions is not None:
                info["live_context_positions"] = [round(p, 4) for p in live_positions]

        return info

    # =========================================================================
    # Obstacle Management RPC Methods
    # =========================================================================

    @rpc
    def add_box_obstacle(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        width: float,
        height: float,
        depth: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> str:
        """Add a box obstacle to the planning world.

        Args:
            name: Unique name for the obstacle
            x, y, z: Position in meters
            width, height, depth: Dimensions in meters
            roll, pitch, yaw: Orientation in radians

        Returns:
            Obstacle ID or empty string if failed
        """
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return ""

        pose = pose_from_xyzrpy(x, y, z, roll, pitch, yaw)
        obstacle_id = self._world_monitor.add_box_obstacle(
            name=name,
            pose=pose,
            dimensions=(width, height, depth),
        )
        logger.info(f"Added box obstacle '{name}' at ({x:.2f}, {y:.2f}, {z:.2f})")
        return obstacle_id

    @rpc
    def add_sphere_obstacle(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        radius: float,
    ) -> str:
        """Add a sphere obstacle to the planning world.

        Args:
            name: Unique name for the obstacle
            x, y, z: Position in meters
            radius: Sphere radius in meters

        Returns:
            Obstacle ID or empty string if failed
        """
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return ""

        pose = pose_from_xyzrpy(x, y, z, 0, 0, 0)
        obstacle_id = self._world_monitor.add_sphere_obstacle(
            name=name,
            pose=pose,
            radius=radius,
        )
        logger.info(f"Added sphere obstacle '{name}' at ({x:.2f}, {y:.2f}, {z:.2f})")
        return obstacle_id

    @rpc
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the planning world.

        Args:
            obstacle_id: ID of obstacle to remove

        Returns:
            True if removed successfully
        """
        if self._world_monitor is None:
            return False
        result = self._world_monitor.remove_obstacle(obstacle_id)
        if result:
            logger.info(f"Removed obstacle '{obstacle_id}'")
        return result

    @rpc
    def clear_obstacles(self) -> bool:
        """Remove all obstacles from the planning world.

        Returns:
            True if cleared successfully
        """
        if self._world_monitor is None:
            return False
        self._world_monitor.clear_obstacles()
        logger.info("Cleared all obstacles")
        return True

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
