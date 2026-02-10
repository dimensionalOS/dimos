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

"""World Monitor - keeps WorldSpec synchronized with real robot state and obstacles."""

from __future__ import annotations

from contextlib import contextmanager
import threading
from typing import TYPE_CHECKING, Any

from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.planning.monitor.world_obstacle_monitor import WorldObstacleMonitor
from dimos.manipulation.planning.monitor.world_state_monitor import WorldStateMonitor
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec import (
        CollisionObjectMessage,
        JointPath,
        Obstacle,
        RobotModelConfig,
        WorldRobotID,
        WorldSpec,
    )
    from dimos.msgs.vision_msgs import Detection3D

logger = setup_logger()


class WorldMonitor:
    """Manages WorldSpec with state/obstacle monitors. Thread-safe via RLock."""

    def __init__(
        self,
        backend: str = "drake",
        enable_viz: bool = False,
        **kwargs: Any,
    ):
        self._backend = backend
        self._world: WorldSpec = create_world(backend=backend, enable_viz=enable_viz, **kwargs)
        self._lock = threading.RLock()
        self._robot_joints: dict[WorldRobotID, list[str]] = {}
        self._state_monitors: dict[WorldRobotID, WorldStateMonitor] = {}
        self._obstacle_monitor: WorldObstacleMonitor | None = None
        self._viz_thread: threading.Thread | None = None
        self._viz_stop_event = threading.Event()
        self._viz_rate_hz: float = 10.0

    # ============= Robot Management =============

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a robot. Returns robot_id."""
        with self._lock:
            robot_id = self._world.add_robot(config)
            self._robot_joints[robot_id] = config.joint_names
            logger.info(f"Added robot '{config.name}' as '{robot_id}'")
            return robot_id

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs."""
        with self._lock:
            return self._world.get_robot_ids()

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get robot configuration."""
        with self._lock:
            return self._world.get_robot_config(robot_id)

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits for a robot."""
        with self._lock:
            return self._world.get_joint_limits(robot_id)

    # ============= Obstacle Management =============

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle. Returns obstacle_id."""
        with self._lock:
            return self._world.add_obstacle(obstacle)

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
        robot_id: WorldRobotID,
        joint_names: list[str] | None = None,
        joint_name_mapping: dict[str, str] | None = None,
    ) -> None:
        """Start monitoring joint states. Uses config defaults if args are None."""
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
        """Start monitoring obstacle updates."""
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

        self._world.close()

    # ============= Message Handlers =============

    def on_joint_state(self, msg: JointState, robot_id: WorldRobotID | None = None) -> None:
        """Handle joint state message. Broadcasts to all monitors if robot_id is None."""
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
        """Handle perception detections (Detection3D from dimos.msgs.vision_msgs)."""
        if self._obstacle_monitor is not None:
            self._obstacle_monitor.on_detections(detections)

    def on_objects(self, objects: object) -> None:
        """Handle Object detections from ObjectDB (preserves object_id)."""
        if self._obstacle_monitor is not None and isinstance(objects, list):
            self._obstacle_monitor.on_objects(objects)

    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, object]]:
        """Refresh perception obstacles from cache. Returns list of added obstacles."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.refresh_obstacles(min_duration)
        return []

    def clear_perception_obstacles(self) -> int:
        """Remove all perception obstacles. Returns count removed."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.clear_perception_obstacles()
        return 0

    def get_perception_status(self) -> dict[str, int]:
        """Get perception obstacle status."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.get_perception_status()
        return {"cached": 0, "added": 0}

    def list_cached_detections(self) -> list[dict[str, object]]:
        """List cached detections from perception."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.list_cached_detections()
        return []

    def list_added_obstacles(self) -> list[dict[str, object]]:
        """List perception obstacles currently in the planning world."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.list_added_obstacles()
        return []

    # ============= State Access =============

    def get_current_joint_state(self, robot_id: WorldRobotID) -> JointState | None:
        """Get current joint state. Returns None if not yet received."""
        # Try state monitor first for positions
        if robot_id in self._state_monitors:
            positions = self._state_monitors[robot_id].get_current_positions()
            velocities = self._state_monitors[robot_id].get_current_velocities()
            if positions is not None:
                joint_names = self._robot_joints.get(robot_id, [])
                return JointState(
                    name=joint_names,
                    position=positions.tolist(),
                    velocity=velocities.tolist() if velocities is not None else [],
                )

        # Fall back to world's live context
        with self._lock:
            ctx = self._world.get_live_context()
            return self._world.get_joint_state(ctx, robot_id)

    def get_current_velocities(self, robot_id: WorldRobotID) -> JointState | None:
        """Get current joint velocities as JointState. Returns None if not available."""
        if robot_id in self._state_monitors:
            velocities = self._state_monitors[robot_id].get_current_velocities()
            if velocities is not None:
                joint_names = self._robot_joints.get(robot_id, [])
                return JointState(name=joint_names, velocity=velocities.tolist())
        return None

    def wait_for_state(self, robot_id: WorldRobotID, timeout: float = 1.0) -> bool:
        """Wait until state is received. Returns False on timeout."""
        if robot_id in self._state_monitors:
            return self._state_monitors[robot_id].wait_for_state(timeout)
        return False

    def is_state_stale(self, robot_id: WorldRobotID, max_age: float = 1.0) -> bool:
        """Check if state is stale."""
        if robot_id in self._state_monitors:
            return self._state_monitors[robot_id].is_state_stale(max_age)
        return True

    # ============= Context Management =============

    @contextmanager
    def scratch_context(self) -> Generator[Any, None, None]:
        """Thread-safe scratch context for planning."""
        with self._world.scratch_context() as ctx:
            yield ctx

    def get_live_context(self) -> Any:
        """Get live context. Prefer scratch_context() for planning."""
        return self._world.get_live_context()

    # ============= Collision Checking =============

    def is_state_valid(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check if configuration is collision-free."""
        return self._world.check_config_collision_free(robot_id, joint_state)

    def is_path_valid(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool:
        """Check if path is collision-free with interpolation.

        Args:
            robot_id: Robot to check
            path: List of JointState waypoints
            step_size: Max step size for interpolation (radians)

        Returns:
            True if entire path is collision-free
        """
        if len(path) < 2:
            return len(path) == 0 or self._world.check_config_collision_free(robot_id, path[0])

        # Check each edge
        for i in range(len(path) - 1):
            if not self._world.check_edge_collision_free(robot_id, path[i], path[i + 1], step_size):
                return False

        return True

    def get_min_distance(self, robot_id: WorldRobotID) -> float:
        """Get minimum distance to obstacles for current state."""
        with self._world.scratch_context() as ctx:
            return self._world.get_min_distance(ctx, robot_id)

    # ============= Kinematics =============

    def get_ee_pose(
        self, robot_id: WorldRobotID, joint_state: JointState | None = None
    ) -> PoseStamped:
        """Get end-effector pose. Uses current state if joint_state is None."""
        with self._world.scratch_context() as ctx:
            # If no state provided, fetch current from state monitor
            if joint_state is None:
                joint_state = self.get_current_joint_state(robot_id)

            if joint_state is not None:
                self._world.set_joint_state(ctx, robot_id, joint_state)

            return self._world.get_ee_pose(ctx, robot_id)

    def get_link_pose(
        self, robot_id: WorldRobotID, link_name: str, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        """Get arbitrary link pose as PoseStamped.

        Args:
            robot_id: Robot to query
            link_name: Name of the link in the URDF
            joint_state: Joint state to use (uses current if None)
        """
        from dimos.msgs.geometry_msgs import Quaternion

        with self._world.scratch_context() as ctx:
            if joint_state is None:
                joint_state = self.get_current_joint_state(robot_id)
            if joint_state is not None:
                self._world.set_joint_state(ctx, robot_id, joint_state)
            try:
                mat = self._world.get_link_pose(ctx, robot_id, link_name)
            except KeyError:
                logger.warning(f"Link '{link_name}' not found in robot '{robot_id}'")
                return None

            pos = mat[:3, 3]
            rot = mat[:3, :3]
            quat = Quaternion.from_rotation_matrix(rot)
            return PoseStamped(
                frame_id="world",
                position=[float(pos[0]), float(pos[1]), float(pos[2])],
                orientation=[float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
            )

    def get_jacobian(self, robot_id: WorldRobotID, joint_state: JointState) -> NDArray[np.float64]:
        """Get 6xN Jacobian matrix."""
        with self._world.scratch_context() as ctx:
            self._world.set_joint_state(ctx, robot_id, joint_state)
            return self._world.get_jacobian(ctx, robot_id)

    # ============= Lifecycle =============

    def finalize(self) -> None:
        """Finalize world. Must be called before collision checking."""
        with self._lock:
            self._world.finalize()
            logger.info("World finalized")

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        return self._world.is_finalized

    # ============= Visualization =============

    def get_visualization_url(self) -> str | None:
        """Get visualization URL or None if not enabled."""
        if hasattr(self._world, "get_visualization_url"):
            url = self._world.get_visualization_url()
            return str(url) if url else None
        return None

    def publish_visualization(self) -> None:
        """Force publish current state to visualization."""
        if hasattr(self._world, "publish_visualization"):
            self._world.publish_visualization()

    def start_visualization_thread(self, rate_hz: float = 10.0) -> None:
        """Start background thread for visualization updates at given rate."""
        if self._viz_thread is not None and self._viz_thread.is_alive():
            logger.warning("Visualization thread already running")
            return

        if not hasattr(self._world, "publish_visualization"):
            logger.warning("World does not support visualization")
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
                if hasattr(self._world, "publish_visualization"):
                    self._world.publish_visualization()
            except Exception as e:
                logger.debug(f"Visualization publish failed: {e}")
            time.sleep(period)

    # ============= Direct World Access =============

    @property
    def world(self) -> WorldSpec:
        """Get underlying WorldSpec. Not thread-safe for modifications."""
        return self._world

    def get_state_monitor(self, robot_id: str) -> WorldStateMonitor | None:
        """Get state monitor for a robot (may be None)."""
        return self._state_monitors.get(robot_id)

    @property
    def obstacle_monitor(self) -> WorldObstacleMonitor | None:
        """Get obstacle monitor (may be None if not started)."""
        return self._obstacle_monitor
