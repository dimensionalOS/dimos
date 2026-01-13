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
World Monitor

MoveIt-like monitor that keeps a WorldSpec synchronized with the real world.
This is the WorldSpec-based replacement for PlanningSceneMonitor.

Uses factory functions to create WorldSpec, so all code uses Protocol types.

Example:
    # Create monitor
    monitor = WorldMonitor(enable_viz=True)

    # Add robot
    robot_id = monitor.add_robot(robot_config)

    # Finalize
    monitor.finalize()

    # Start monitoring
    monitor.start_state_monitor("piper", ["joint1", "joint2", ...])
    monitor.start_obstacle_monitor()

    # Use thread-safe accessors
    positions = monitor.get_current_positions(robot_id)
    is_valid = monitor.is_state_valid(robot_id, target_joints)

    # Get scratch context for planning
    with monitor.scratch_context() as ctx:
        # Do planning operations
        pass
"""

from __future__ import annotations

from contextlib import contextmanager
import threading
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.planning.monitor.world_obstacle_monitor import WorldObstacleMonitor
from dimos.manipulation.planning.monitor.world_state_monitor import WorldStateMonitor
from dimos.manipulation.planning.spec import (
    CollisionObjectMessage,
    Detection3D,
    Obstacle,
    ObstacleType,
    RobotModelConfig,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec import WorldSpec
    from dimos.msgs.sensor_msgs import JointState

logger = setup_logger()


class WorldMonitor:
    """MoveIt-like monitor for keeping a WorldSpec synchronized.

    This class manages:
    - WorldSpec instance (created via factory)
    - WorldStateMonitor: Subscribes to joint states, syncs robot configurations
    - WorldObstacleMonitor: Subscribes to collision objects, syncs obstacles

    ## Key Differences from PlanningSceneMonitor

    1. Uses factory pattern - no concrete type references
    2. Uses scratch_context() for thread-safe operations
    3. State is synced via sync_from_joint_state() instead of set_robot_state()

    ## Usage Pattern

    ```python
    # 1. Create monitor
    monitor = WorldMonitor(enable_viz=True)

    # 2. Add robots
    robot_id = monitor.add_robot(robot_config)

    # 3. Add static obstacles (optional)
    monitor.add_obstacle(table_obstacle)

    # 4. Finalize (locks topology)
    monitor.finalize()

    # 5. Start monitoring
    monitor.start_state_monitor(robot_id, joint_names)
    monitor.start_obstacle_monitor()

    # 6. Use thread-safe accessors
    positions = monitor.get_current_positions(robot_id)
    is_valid = monitor.is_state_valid(robot_id, target_joints)

    # 7. Get scratch context for planning
    with monitor.scratch_context() as ctx:
        world.set_positions(ctx, robot_id, target_joints)
        if world.is_collision_free(ctx, robot_id):
            # Valid configuration
            pass
    ```

    ## Thread Safety

    All public methods are thread-safe via an internal RLock.
    For planning operations, use scratch_context() to get an independent context.
    """

    def __init__(
        self,
        backend: str = "drake",
        enable_viz: bool = False,
        **kwargs: Any,
    ):
        """Create a world monitor.

        Args:
            backend: Backend to use ("drake")
            enable_viz: Enable visualization (Meshcat for Drake)
            **kwargs: Additional arguments passed to create_world()
        """
        self._backend = backend

        # Create world via factory (Protocol type)
        self._world: WorldSpec = create_world(
            backend=backend,
            enable_viz=enable_viz,
            **kwargs,
        )

        # Thread safety
        self._lock = threading.RLock()

        # Robot tracking: robot_id -> joint_names
        self._robot_joints: dict[str, list[str]] = {}

        # Sub-monitors
        self._state_monitors: dict[str, WorldStateMonitor] = {}
        self._obstacle_monitor: WorldObstacleMonitor | None = None

        # Visualization thread (publishes to Meshcat at a fixed rate)
        self._viz_thread: threading.Thread | None = None
        self._viz_stop_event = threading.Event()
        self._viz_rate_hz: float = 10.0  # Default 10Hz

    # ============= Robot Management =============

    def add_robot(
        self,
        config: RobotModelConfig,
    ) -> str:
        """Add a robot to the monitored world.

        Args:
            config: Robot configuration

        Returns:
            robot_id: Unique identifier for the robot
        """
        with self._lock:
            robot_id = self._world.add_robot(config)
            self._robot_joints[robot_id] = config.joint_names
            logger.info(f"Added robot '{config.name}' as '{robot_id}'")
            return robot_id

    def get_robot_ids(self) -> list[str]:
        """Get all robot IDs."""
        with self._lock:
            return self._world.get_robot_ids()

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        """Get robot configuration."""
        with self._lock:
            return self._world.get_robot_config(robot_id)

    def get_joint_limits(self, robot_id: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits for a robot."""
        with self._lock:
            return self._world.get_joint_limits(robot_id)

    # ============= Obstacle Management =============

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle to the world.

        Args:
            obstacle: Obstacle specification

        Returns:
            obstacle_id: Unique identifier for the obstacle
        """
        with self._lock:
            return self._world.add_obstacle(obstacle)

    def add_box_obstacle(
        self,
        name: str,
        pose: NDArray[np.float64],
        dimensions: tuple[float, float, float],
        color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8),
    ) -> str:
        """Add a box obstacle.

        Args:
            name: Obstacle name
            pose: 4x4 homogeneous transform
            dimensions: (width, height, depth) in meters
            color: RGBA color

        Returns:
            obstacle_id
        """
        obstacle = Obstacle(
            name=name,
            obstacle_type=ObstacleType.BOX,
            pose=pose,
            dimensions=dimensions,
            color=color,
        )
        return self.add_obstacle(obstacle)

    def add_sphere_obstacle(
        self,
        name: str,
        pose: NDArray[np.float64],
        radius: float,
        color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8),
    ) -> str:
        """Add a sphere obstacle."""
        obstacle = Obstacle(
            name=name,
            obstacle_type=ObstacleType.SPHERE,
            pose=pose,
            dimensions=(radius,),
            color=color,
        )
        return self.add_obstacle(obstacle)

    def add_cylinder_obstacle(
        self,
        name: str,
        pose: NDArray[np.float64],
        radius: float,
        height: float,
        color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8),
    ) -> str:
        """Add a cylinder obstacle."""
        obstacle = Obstacle(
            name=name,
            obstacle_type=ObstacleType.CYLINDER,
            pose=pose,
            dimensions=(radius, height),
            color=color,
        )
        return self.add_obstacle(obstacle)

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle."""
        with self._lock:
            return self._world.remove_obstacle(obstacle_id)

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        with self._lock:
            self._world.clear_obstacles()

    # ============= Monitor Control =============

    def start_state_monitor(
        self,
        robot_id: str,
        joint_names: list[str] | None = None,
        joint_name_mapping: dict[str, str] | None = None,
    ) -> None:
        """Start monitoring joint states for a robot.

        Creates a WorldStateMonitor that will sync joint state messages
        to the world's live context.

        Args:
            robot_id: ID of the robot to monitor
            joint_names: Joint names in URDF namespace (uses config joint_names if None)
            joint_name_mapping: Maps orchestrator joint names to URDF joint names.
                Example: {"left_joint1": "joint1"} means messages with "left_joint1"
                will be mapped to URDF "joint1". If None, uses config mapping if available,
                otherwise names must match exactly.
        """
        with self._lock:
            if robot_id in self._state_monitors:
                logger.warning(f"State monitor for '{robot_id}' already started")
                return

            # Get config for defaults
            config = self._world.get_robot_config(robot_id)

            # Get joint names from config if not provided
            if joint_names is None:
                if robot_id in self._robot_joints:
                    joint_names = self._robot_joints[robot_id]
                else:
                    joint_names = config.joint_names

            # Get joint name mapping from config if not provided
            if joint_name_mapping is None and config.joint_name_mapping:
                joint_name_mapping = config.joint_name_mapping

            monitor = WorldStateMonitor(
                world=self._world,
                lock=self._lock,
                robot_id=robot_id,
                joint_names=joint_names,
                joint_name_mapping=joint_name_mapping,
            )
            monitor.start()
            self._state_monitors[robot_id] = monitor
            logger.info(f"State monitor started for '{robot_id}'")

    def start_obstacle_monitor(self) -> None:
        """Start monitoring obstacle updates.

        Creates a WorldObstacleMonitor that will sync obstacle messages
        to the world.
        """
        with self._lock:
            if self._obstacle_monitor is not None:
                logger.warning("Obstacle monitor already started")
                return

            self._obstacle_monitor = WorldObstacleMonitor(
                world=self._world,
                lock=self._lock,
            )
            self._obstacle_monitor.start()
            logger.info("Obstacle monitor started")

    def stop_all_monitors(self) -> None:
        """Stop all monitors and visualization thread."""
        # Stop visualization thread first (outside lock to avoid deadlock)
        self.stop_visualization_thread()

        with self._lock:
            for _robot_id, monitor in self._state_monitors.items():
                monitor.stop()
            self._state_monitors.clear()

            if self._obstacle_monitor is not None:
                self._obstacle_monitor.stop()
                self._obstacle_monitor = None

            logger.info("All monitors stopped")

    # ============= Message Handlers =============

    def on_joint_state(self, msg: JointState, robot_id: str | None = None) -> None:
        """Handle joint state message.

        Args:
            msg: JointState message
            robot_id: Specific robot to update (broadcasts to all if None)
        """
        try:
            if robot_id is not None:
                if robot_id in self._state_monitors:
                    self._state_monitors[robot_id].on_joint_state(msg)
                else:
                    logger.warning(f"No state monitor for robot_id: {robot_id}")
            else:
                # Broadcast to all monitors
                for monitor in self._state_monitors.values():
                    monitor.on_joint_state(msg)
        except Exception as e:
            logger.error(f"[WorldMonitor] Exception in on_joint_state: {e}")
            import traceback

            logger.error(traceback.format_exc())

    def on_collision_object(self, msg: CollisionObjectMessage) -> None:
        """Handle collision object message."""
        if self._obstacle_monitor is not None:
            self._obstacle_monitor.on_collision_object(msg)

    def on_detections(self, detections: list[Detection3D]) -> None:
        """Handle perception detections."""
        if self._obstacle_monitor is not None:
            self._obstacle_monitor.on_detections(detections)

    # ============= State Access =============

    def get_current_positions(self, robot_id: str) -> NDArray[np.float64] | None:
        """Get current joint positions (thread-safe).

        Args:
            robot_id: ID of the robot

        Returns:
            Current positions or None if not yet received
        """
        # Try state monitor first
        if robot_id in self._state_monitors:
            positions = self._state_monitors[robot_id].get_current_positions()
            if positions is not None:
                return positions

        # Fall back to world's live context
        with self._lock:
            ctx = self._world.get_live_context()
            return self._world.get_positions(ctx, robot_id)

    def get_current_velocities(self, robot_id: str) -> NDArray[np.float64] | None:
        """Get current joint velocities (thread-safe).

        Args:
            robot_id: ID of the robot

        Returns:
            Current velocities or None if not available
        """
        if robot_id in self._state_monitors:
            return self._state_monitors[robot_id].get_current_velocities()
        return None

    def wait_for_state(self, robot_id: str, timeout: float = 1.0) -> bool:
        """Wait until a state is received for the robot.

        Args:
            robot_id: ID of the robot
            timeout: Maximum time to wait

        Returns:
            True if state was received, False if timeout
        """
        if robot_id in self._state_monitors:
            return self._state_monitors[robot_id].wait_for_state(timeout)
        return False

    def is_state_stale(self, robot_id: str, max_age: float = 1.0) -> bool:
        """Check if state is stale."""
        if robot_id in self._state_monitors:
            return self._state_monitors[robot_id].is_state_stale(max_age)
        return True

    # ============= Context Management =============

    @contextmanager
    def scratch_context(self) -> Generator[Any, None, None]:
        """Get a scratch context for planning (thread-safe).

        The scratch context is a clone of the live context that can be
        modified without affecting the live state.

        Example:
            with monitor.scratch_context() as ctx:
                world.set_positions(ctx, robot_id, q_test)
                if world.is_collision_free(ctx, robot_id):
                    # Valid configuration
                    pass

        Yields:
            Context that can be used with world operations
        """
        with self._world.scratch_context() as ctx:
            yield ctx

    def get_live_context(self) -> Any:
        """Get the live context (use with caution).

        The live context is synced with real robot state. Modifications
        will affect collision checking for all users.

        For planning, prefer scratch_context() instead.

        Returns:
            Live context
        """
        return self._world.get_live_context()

    # ============= Collision Checking =============

    def is_state_valid(
        self,
        robot_id: str,
        joint_positions: NDArray[np.float64],
    ) -> bool:
        """Check if configuration is collision-free (thread-safe).

        Args:
            robot_id: ID of the robot
            joint_positions: Joint configuration to check

        Returns:
            True if collision-free
        """
        with self._world.scratch_context() as ctx:
            self._world.set_positions(ctx, robot_id, joint_positions)
            return self._world.is_collision_free(ctx, robot_id)

    def is_path_valid(
        self,
        robot_id: str,
        path: list[NDArray[np.float64]],
        step_size: float = 0.05,
    ) -> bool:
        """Check if path is collision-free (thread-safe).

        Args:
            robot_id: ID of the robot
            path: List of joint configurations
            step_size: Interpolation resolution

        Returns:
            True if path is collision-free
        """
        with self._world.scratch_context() as ctx:
            for i in range(len(path) - 1):
                q_start = path[i]
                q_end = path[i + 1]

                # Number of interpolation steps
                dist = np.linalg.norm(q_end - q_start)
                num_steps = max(2, int(np.ceil(dist / step_size)))

                for j in range(num_steps):
                    alpha = j / (num_steps - 1)
                    q = q_start + alpha * (q_end - q_start)

                    self._world.set_positions(ctx, robot_id, q)
                    if not self._world.is_collision_free(ctx, robot_id):
                        return False

        return True

    def get_min_distance(self, robot_id: str) -> float:
        """Get minimum distance to obstacles for current state."""
        with self._world.scratch_context() as ctx:
            return self._world.get_min_distance(ctx, robot_id)

    # ============= Kinematics =============

    def get_ee_pose(
        self,
        robot_id: str,
        joint_positions: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Get end-effector pose (thread-safe).

        Args:
            robot_id: ID of the robot
            joint_positions: Joint configuration (uses current if None)

        Returns:
            4x4 homogeneous transform
        """
        with self._world.scratch_context() as ctx:
            # If no positions provided, fetch current from state monitor
            if joint_positions is None:
                joint_positions = self.get_current_positions(robot_id)

            if joint_positions is not None:
                self._world.set_positions(ctx, robot_id, joint_positions)

            return self._world.get_ee_pose(ctx, robot_id)

    def get_jacobian(
        self,
        robot_id: str,
        joint_positions: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Get Jacobian (thread-safe).

        Args:
            robot_id: ID of the robot
            joint_positions: Joint configuration

        Returns:
            6 x n_joints Jacobian matrix
        """
        with self._world.scratch_context() as ctx:
            self._world.set_positions(ctx, robot_id, joint_positions)
            return self._world.get_jacobian(ctx, robot_id)

    # ============= Lifecycle =============

    def finalize(self) -> None:
        """Finalize world for collision checking.

        Must be called after adding all robots and before starting monitors
        or performing collision checking.
        """
        with self._lock:
            self._world.finalize()
            logger.info("World finalized")

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        return self._world.is_finalized

    # ============= Visualization =============

    def get_meshcat_url(self) -> str | None:
        """Get Meshcat visualization URL (Drake only).

        Returns:
            URL string or None if visualization not enabled
        """
        if hasattr(self._world, "get_meshcat_url"):
            url = self._world.get_meshcat_url()
            return str(url) if url else None
        return None

    def publish_visualization(self) -> None:
        """Force publish current state to visualization."""
        if hasattr(self._world, "publish_to_meshcat"):
            self._world.publish_to_meshcat()

    def start_visualization_thread(self, rate_hz: float = 10.0) -> None:
        """Start background thread that updates Meshcat visualization.

        This allows live visualization without blocking the LCM callback thread.
        The thread publishes the current robot state to Meshcat at the specified rate.

        Args:
            rate_hz: Visualization update rate in Hz (default: 10Hz)
        """
        if self._viz_thread is not None and self._viz_thread.is_alive():
            logger.warning("Visualization thread already running")
            return

        if not hasattr(self._world, "publish_to_meshcat"):
            logger.warning("World does not support Meshcat visualization")
            return

        self._viz_rate_hz = rate_hz
        self._viz_stop_event.clear()
        self._viz_thread = threading.Thread(
            target=self._visualization_loop,
            name="MeshcatVizThread",
            daemon=True,
        )
        self._viz_thread.start()
        logger.info(f"Visualization thread started at {rate_hz}Hz")

    def stop_visualization_thread(self) -> None:
        """Stop the visualization thread."""
        if self._viz_thread is None:
            return

        self._viz_stop_event.set()
        self._viz_thread.join(timeout=1.0)
        if self._viz_thread.is_alive():
            logger.warning("Visualization thread did not stop cleanly")
        self._viz_thread = None
        logger.info("Visualization thread stopped")

    def _visualization_loop(self) -> None:
        """Internal: Visualization update loop."""
        import time

        period = 1.0 / self._viz_rate_hz
        while not self._viz_stop_event.is_set():
            try:
                if hasattr(self._world, "publish_to_meshcat"):
                    self._world.publish_to_meshcat()
            except Exception as e:
                logger.debug(f"Visualization publish failed: {e}")
            time.sleep(period)

    # ============= Direct World Access =============

    @property
    def world(self) -> WorldSpec:
        """Get underlying WorldSpec instance.

        Direct access to the world is NOT thread-safe for state modifications.
        Prefer using scratch_context() for planning operations.
        """
        return self._world

    def get_state_monitor(self, robot_id: str) -> WorldStateMonitor | None:
        """Get state monitor for a robot (may be None)."""
        return self._state_monitors.get(robot_id)

    @property
    def obstacle_monitor(self) -> WorldObstacleMonitor | None:
        """Get obstacle monitor (may be None if not started)."""
        return self._obstacle_monitor
