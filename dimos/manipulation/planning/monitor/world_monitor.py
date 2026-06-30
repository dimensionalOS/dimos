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

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.manipulation.planning.groups.identifiers import (
    is_global_joint_name,
    make_global_joint_names,
)
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.monitor.robot_state_monitor import RobotStateMonitor
from dimos.manipulation.planning.monitor.world_obstacle_monitor import WorldObstacleMonitor
from dimos.manipulation.planning.spec.models import CollisionCheckResult, PlanningSceneInfo
from dimos.manipulation.planning.spec.protocols import VisualizationSpec, WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator, Sequence

    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        CollisionObjectMessage,
        GeneratedPlan,
        JointPath,
        Obstacle,
        PlanningGroupID,
        WorldRobotID,
    )
    from dimos.msgs.vision_msgs.Detection3D import Detection3D
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


class WorldMonitor:
    """Manages WorldSpec with state/obstacle monitors. Thread-safe via RLock."""

    def __init__(
        self,
        world: WorldSpec,
        visualization: VisualizationSpec | None = None,
    ) -> None:
        self._world = world
        self._visualization = visualization
        self._lock = threading.RLock()
        self._robot_joints: dict[WorldRobotID, list[str]] = {}
        self._robot_ids_by_name: dict[str, WorldRobotID] = {}
        self._planning_groups = PlanningGroupRegistry()
        self._robot_configs: dict[WorldRobotID, RobotModelConfig] = {}
        self._state_monitors: dict[WorldRobotID, RobotStateMonitor] = {}
        self._obstacle_monitor: WorldObstacleMonitor | None = None
        self._viz_thread: threading.Thread | None = None
        self._viz_stop_event = threading.Event()
        self._viz_rate_hz: float = 10.0

    # Robot Management

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a robot. Returns robot_id."""
        with self._lock:
            robot_id = self._world.add_robot(config)
            self._robot_joints[robot_id] = config.joint_names
            if config.name in self._robot_ids_by_name:
                raise ValueError(f"Robot name '{config.name}' is already registered")
            self._robot_ids_by_name[config.name] = robot_id
            self._planning_groups.add_robot(config)
            self._robot_configs[robot_id] = config
            logger.info(f"Added robot '{config.name}' as '{robot_id}'")
        return robot_id

    def planning_scene_info(self) -> PlanningSceneInfo:
        """Return a stable metadata snapshot of the initialized planning scene."""
        with self._lock:
            return PlanningSceneInfo(robots=dict(self._robot_configs))

    def sync_visualization_scene(self) -> None:
        """Synchronize startup scene metadata to the attached visualization."""
        visualization = self._visualization
        if visualization is None:
            return
        visualization.initialize_scene(self.planning_scene_info())

    @property
    def planning_groups(self) -> PlanningGroupRegistry:
        """Backend-independent planning-group registry for added robots."""
        return self._planning_groups

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

    # Obstacle Management

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

    # Monitor Control

    def start_state_monitor(
        self,
        robot_id: WorldRobotID,
        joint_names: list[str] | None = None,
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

            monitor = RobotStateMonitor(
                world=self._world,
                lock=self._lock,
                robot_id=robot_id,
                joint_names=joint_names,
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

        if self._visualization is not None:
            self._visualization.close()

    # Message Handlers

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

    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]:
        """Refresh perception obstacles from cache. Returns list of added obstacles."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.refresh_obstacles(min_duration)
        return []

    def remove_object_obstacle(self, object_id: str) -> bool:
        """Remove a single object's obstacle from the planning world."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.remove_object_obstacle(object_id)
        return False

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

    def get_cached_objects(self) -> list[Object]:
        """Get cached Object instances from perception."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.get_cached_objects()
        return []

    def list_cached_detections(self) -> list[dict[str, Any]]:
        """List cached detections from perception."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.list_cached_detections()
        return []

    def list_added_obstacles(self) -> list[dict[str, Any]]:
        """List perception obstacles currently in the planning world."""
        if self._obstacle_monitor is not None:
            return self._obstacle_monitor.list_added_obstacles()
        return []

    # State Access

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

    def current_global_joint_state(self, max_age: float = 1.0) -> JointState | None:
        """Return fresh monitored state for every robot using global joint names.

        This method is intentionally stricter than ``get_current_joint_state``:
        it never falls back to backend live/default state. Callers use it when
        stale or absent telemetry should stop a planning-world operation.
        """
        names: list[str] = []
        positions: list[float] = []
        for robot_id, config in self._robot_configs.items():
            monitor = self._state_monitors.get(robot_id)
            if monitor is None or monitor.is_state_stale(max_age):
                return None
            current = self.get_current_joint_state(robot_id)
            if current is None or len(current.name) != len(current.position):
                return None
            positions_by_local_name = dict(zip(current.name, current.position, strict=True))
            for local_name, global_name in zip(
                config.joint_names,
                make_global_joint_names(config.name, config.joint_names),
                strict=True,
            ):
                if local_name not in positions_by_local_name:
                    return None
                names.append(global_name)
                positions.append(float(positions_by_local_name[local_name]))
        return JointState(name=names, position=positions)

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

    # Context Management

    @contextmanager
    def scratch_context(self) -> Generator[Any, None, None]:
        """Thread-safe scratch context for planning."""
        with self._world.scratch_context() as ctx:
            yield ctx

    def get_live_context(self) -> Any:
        """Get live context. Prefer scratch_context() for planning."""
        return self._world.get_live_context()

    # Collision Checking

    def is_state_valid(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check if configuration is collision-free."""
        return self._world.check_config_collision_free(robot_id, joint_state)

    def check_collision(
        self,
        target_joints: JointState,
        max_age: float = 1.0,
    ) -> CollisionCheckResult:
        """Check a partial global target against the full planning-world state."""
        if not self._world.is_finalized:
            return CollisionCheckResult(
                status="UNAVAILABLE",
                collision_free=None,
                message="Planning world is not finalized",
            )
        if not target_joints.name:
            return CollisionCheckResult(
                status="INVALID",
                collision_free=None,
                message="Collision target must include global joint names",
            )
        if len(target_joints.name) != len(target_joints.position):
            return CollisionCheckResult(
                status="INVALID",
                collision_free=None,
                message="Collision target name and position lengths must match",
            )
        if len(set(target_joints.name)) != len(target_joints.name):
            return CollisionCheckResult(
                status="INVALID",
                collision_free=None,
                message="Collision target contains duplicate joint names",
            )
        invalid_names = [name for name in target_joints.name if not is_global_joint_name(name)]
        if invalid_names:
            return CollisionCheckResult(
                status="INVALID",
                collision_free=None,
                message=f"Expected global joint names; got invalid names: {invalid_names}",
            )

        current = self.current_global_joint_state(max_age=max_age)
        if current is None:
            return CollisionCheckResult(
                status="STALE_STATE",
                collision_free=None,
                message="Fresh monitored planning-world joint state is unavailable",
            )

        positions_by_global_name = dict(zip(current.name, current.position, strict=True))
        unknown = [name for name in target_joints.name if name not in positions_by_global_name]
        if unknown:
            return CollisionCheckResult(
                status="INVALID",
                collision_free=None,
                message=f"Unknown global joint names: {unknown}",
            )
        for name, position in zip(target_joints.name, target_joints.position, strict=True):
            positions_by_global_name[name] = float(position)

        try:
            with self._world.scratch_context() as ctx:
                for robot_id, config in self._robot_configs.items():
                    global_names = make_global_joint_names(config.name, config.joint_names)
                    local_state = JointState(
                        name=list(config.joint_names),
                        position=[positions_by_global_name[name] for name in global_names],
                    )
                    self._world.set_joint_state(ctx, robot_id, local_state)
                collision_free = all(
                    self._world.is_collision_free(ctx, robot_id) for robot_id in self._robot_configs
                )
        except Exception as exc:
            return CollisionCheckResult(
                status="UNAVAILABLE",
                collision_free=None,
                message=f"Collision check failed: {exc}",
            )

        if collision_free:
            return CollisionCheckResult(
                status="VALID",
                collision_free=True,
                message="Target is collision-free",
            )
        return CollisionCheckResult(
            status="COLLISION",
            collision_free=False,
            message="Target is in collision",
        )

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

    # Kinematics

    def get_ee_pose(
        self, robot_id: WorldRobotID, joint_state: JointState | None = None
    ) -> PoseStamped:
        """Get end-effector pose for the robot's unique pose-targetable group."""
        return self.get_group_ee_pose(
            self._unique_pose_group_id_for_robot(robot_id),
            joint_state=joint_state,
        )

    def get_group_ee_pose(
        self, group_id: PlanningGroupID, joint_state: JointState | None = None
    ) -> PoseStamped:
        """Get planning group target-frame pose using current state by default."""
        robot_id = self._robot_id_for_group(group_id)
        with self._world.scratch_context() as ctx:
            if joint_state is None:
                joint_state = self.get_current_joint_state(robot_id)
            if joint_state is not None:
                self._world.set_joint_state(ctx, robot_id, joint_state)

            return self._world.get_group_ee_pose(ctx, group_id)

    def get_link_pose(
        self, robot_id: WorldRobotID, link_name: str, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        """Get arbitrary link pose as PoseStamped.

        Args:
            robot_id: Robot to query
            link_name: Name of the link in the URDF
            joint_state: Joint state to use (uses current if None)
        """
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
        """Get 6xN Jacobian for the robot's unique pose-targetable group."""
        return self.get_group_jacobian(
            self._unique_pose_group_id_for_robot(robot_id),
            joint_state=joint_state,
        )

    def get_group_jacobian(
        self, group_id: PlanningGroupID, joint_state: JointState
    ) -> NDArray[np.float64]:
        """Get planning group target-frame 6xN Jacobian matrix."""
        self._planning_groups.get(group_id)
        robot_id = self._robot_id_for_group(group_id)
        with self._world.scratch_context() as ctx:
            self._world.set_joint_state(ctx, robot_id, joint_state)
            return self._world.get_group_jacobian(ctx, group_id)

    def _unique_pose_group_id_for_robot(self, robot_id: WorldRobotID) -> PlanningGroupID:
        robot_name = self._world.get_robot_config(robot_id).name
        pose_group_ids = [
            group.id
            for group in self._planning_groups.groups_for_robot(robot_name)
            if group.has_pose_target
        ]
        if len(pose_group_ids) != 1:
            raise ValueError(
                f"Robot '{robot_name}' has {len(pose_group_ids)} pose-targetable planning groups; "
                "call get_group_ee_pose/get_group_jacobian with an explicit planning group ID"
            )
        return pose_group_ids[0]

    def _robot_id_for_group(self, group_id: PlanningGroupID) -> WorldRobotID:
        group = self._planning_groups.get(group_id)
        try:
            return self._robot_ids_by_name[group.robot_name]
        except KeyError as exc:
            raise KeyError(
                f"Robot '{group.robot_name}' not found for planning group {group_id}"
            ) from exc

    # Lifecycle

    def finalize(self) -> None:
        """Finalize world. Must be called before collision checking."""
        with self._lock:
            self._world.finalize()
            logger.info("World finalized")

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        return self._world.is_finalized

    # Visualization

    def get_visualization_url(self) -> str | None:
        """Get visualization URL or None if not enabled."""
        if self._visualization is not None:
            url = self._visualization.get_visualization_url()
            return str(url) if url else None
        return None

    def publish_visualization(self) -> None:
        """Force publish current state to visualization."""
        if self._visualization is not None:
            self._visualization.publish_visualization()

    def show_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        """Show preview representation for planning groups if visualization is available."""
        if self._visualization is not None:
            self._visualization.show_preview(group_ids)

    def hide_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        """Hide preview representation for planning groups if visualization is available."""
        if self._visualization is not None:
            self._visualization.hide_preview(group_ids)

    def animate_plan(self, plan: GeneratedPlan, duration: float = 3.0) -> None:
        """Animate a generated plan if visualization is available."""
        if self._visualization is not None:
            self._visualization.animate_plan(plan, duration)

    def start_visualization_thread(self, rate_hz: float = 10.0) -> None:
        """Start background thread for visualization updates at given rate."""
        if self._viz_thread is not None and self._viz_thread.is_alive():
            logger.warning("Visualization thread already running")
            return

        if self._visualization is None:
            logger.warning("World does not support visualization")
            return

        self._viz_rate_hz = rate_hz
        self._viz_stop_event.clear()
        self._viz_thread = threading.Thread(
            target=self._visualization_loop,
            name="ManipulationVizThread",
            daemon=True,
        )
        self._viz_thread.start()
        logger.info(f"Visualization thread started at {rate_hz}Hz")

    def stop_visualization_thread(self) -> None:
        """Stop the visualization thread."""
        if self._viz_thread is None:
            return

        self._viz_stop_event.set()
        self._viz_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
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
                self.publish_visualization()
            except Exception as e:
                logger.debug(f"Visualization publish failed: {e}")
            time.sleep(period)

    # Direct World Access

    @property
    def world(self) -> WorldSpec:
        """Get underlying WorldSpec. Not thread-safe for modifications."""
        return self._world

    @property
    def visualization(self) -> VisualizationSpec | None:
        """Get optional visualization backend."""
        return self._visualization

    def set_visualization(self, visualization: VisualizationSpec | None) -> None:
        """Set optional visualization backend after monitor construction."""
        self._visualization = visualization

    def get_state_monitor(self, robot_id: str) -> RobotStateMonitor | None:
        """Get state monitor for a robot (may be None)."""
        return self._state_monitors.get(robot_id)

    @property
    def obstacle_monitor(self) -> WorldObstacleMonitor | None:
        """Get obstacle monitor (may be None if not started)."""
        return self._obstacle_monitor
