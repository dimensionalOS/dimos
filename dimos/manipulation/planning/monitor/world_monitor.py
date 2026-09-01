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
from dimos.manipulation.planning.groups.identifiers import assert_valid_group_id
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.groups.utils import filter_joint_state_to_selected_joints
from dimos.manipulation.planning.monitor.robot_state_monitor import RobotStateMonitor
from dimos.manipulation.planning.monitor.world_obstacle_monitor import WorldObstacleMonitor
from dimos.manipulation.planning.spec.models import (
    PlanningSceneInfo,
    VisualizationSession,
    VisualizationStateFrame,
)
from dimos.manipulation.planning.spec.protocols import VisualizationSpec, WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        CollisionObjectMessage,
        JointPath,
        Obstacle,
        PlanningGroupID,
    )
    from dimos.msgs.vision_msgs.Detection3D import Detection3D

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
        # Keep renderer mutations and periodic publishes ordered.  Cancellation is
        # deliberately issued outside this lock so it can interrupt an animation.
        self._visualization_lock = threading.RLock()
        self._model_config: RobotModelConfig | None = None
        self._planning_groups = PlanningGroupRegistry()
        self._state_monitor: RobotStateMonitor | None = None
        self._obstacle_monitor: WorldObstacleMonitor | None = None
        self._viz_thread: threading.Thread | None = None
        self._viz_stop_event = threading.Event()
        self._viz_rate_hz: float = 10.0

    # Model Management

    def load_model(self, config: RobotModelConfig) -> None:
        """Load the one logical robot model."""
        with self._lock:
            if self._model_config is not None:
                raise ValueError("A model is already loaded")
            self._validate_planning_group_config(config)
            self._world.load_model(config)
            self._model_config = config
            self._planning_groups.add_model(config)

    @property
    def planning_groups(self) -> PlanningGroupRegistry:
        """Registered public planning groups."""
        return self._planning_groups

    def planning_scene_info(self) -> PlanningSceneInfo:
        """Return a stable metadata snapshot of the initialized planning scene."""
        with self._lock:
            return PlanningSceneInfo(
                model=self.get_model_config(),
                planning_groups=tuple(self._planning_groups.list()),
            )

    def get_model_config(self) -> RobotModelConfig:
        """Get the configured model."""
        with self._lock:
            if self._model_config is None:
                raise RuntimeError("Model is not loaded")
            return self._model_config

    def get_joint_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get model joint limits."""
        with self._lock:
            return self._world.get_joint_limits()

    # Obstacle Management

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle and visualize it only when the world accepts it."""
        with self._lock:
            obstacle_id = self._world.add_obstacle(obstacle)
            if not obstacle_id:
                return ""
            if self._visualization is not None:
                try:
                    self._visualization.add_vis_obstacle(obstacle_id, obstacle)
                except Exception:
                    logger.exception("Obstacle visualization add failed for '%s'", obstacle_id)
            return obstacle_id

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle."""
        with self._lock:
            removed = self._world.remove_obstacle(obstacle_id)
            if removed and self._visualization is not None:
                try:
                    self._visualization.remove_vis_obstacle(obstacle_id)
                except Exception:
                    logger.exception("Obstacle visualization remove failed for '%s'", obstacle_id)
            return removed

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update an obstacle through the authoritative planning-world mutation path."""
        with self._lock:
            updated = self._world.update_obstacle_pose(obstacle_id, pose)
            if updated and self._visualization is not None:
                try:
                    self._visualization.update_vis_obstacle_pose(obstacle_id, pose)
                except Exception:
                    logger.exception("Obstacle visualization update failed for '%s'", obstacle_id)
            return updated

    def update_obstacle(self, obstacle: Obstacle) -> bool:
        """Replace an obstacle and then forward the accepted value to visualization."""
        with self._lock:
            updated = self._world.update_obstacle(obstacle)
            if updated and self._visualization is not None:
                try:
                    self._visualization.update_vis_obstacle(obstacle)
                except Exception:
                    logger.exception("Obstacle visualization update failed for '%s'", obstacle.name)
            return updated

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        with self._lock:
            self._world.clear_obstacles()
            if self._obstacle_monitor is not None:
                self._obstacle_monitor.clear_tracking()
            if self._visualization is not None:
                try:
                    self._visualization.clear_vis_obstacles()
                except Exception:
                    logger.exception("Obstacle visualization clear failed")

    # Monitor Control

    def start_state_monitor(self) -> None:
        """Start monitoring canonical model joint state."""
        with self._lock:
            if self._state_monitor is not None:
                logger.warning("State monitor already started")
                return
            config = self.get_model_config()
            monitor = RobotStateMonitor(
                world=self._world,
                lock=self._lock,
                joint_names=config.joint_names,
            )
            monitor.start()
            self._state_monitor = monitor

    def start_obstacle_monitor(self) -> None:
        """Start monitoring obstacle updates."""
        with self._lock:
            if self._obstacle_monitor is not None:
                logger.warning("Obstacle monitor already started")
                return

            self._obstacle_monitor = WorldObstacleMonitor(
                parent=self,
            )
            self._obstacle_monitor.start()
            logger.info("Obstacle monitor started")

    def stop_all_monitors(self) -> None:
        """Stop all monitors and visualization thread."""
        # Stop visualization thread first (outside lock to avoid deadlock)
        self.stop_visualization_thread()

        with self._lock:
            if self._state_monitor is not None:
                self._state_monitor.stop()
                self._state_monitor = None

            if self._obstacle_monitor is not None:
                self._obstacle_monitor.stop()
                self._obstacle_monitor = None

            logger.info("All monitors stopped")

        if self._visualization is not None:
            self._visualization.cancel_preview_animation()
            # Wait for cancelled animation cleanup before releasing renderer resources.
            with self._visualization_lock:
                self._visualization.close()

    # Message Handlers

    def on_joint_state(self, msg: JointState) -> None:
        """Handle a canonical model joint-state message."""
        try:
            if self._state_monitor is not None:
                self._state_monitor.on_joint_state(msg)
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

    # State Access

    def get_current_joint_state(self) -> JointState | None:
        """Get current joint state. Returns None if not yet received."""
        if self._state_monitor is not None:
            positions = self._state_monitor.get_current_positions()
            velocities = self._state_monitor.get_current_velocities()
            if positions is not None:
                return JointState(
                    name=self.get_model_config().joint_names,
                    position=positions.tolist(),
                    velocity=velocities.tolist() if velocities is not None else [],
                )

        # Fall back to world's live context
        with self._lock:
            ctx = self._world.get_live_context()
            return self._world.get_joint_state(ctx)

    def current_model_joint_state(self, max_age: float = 1.0) -> JointState:
        """Return the fresh canonical model state."""
        if self._state_monitor is not None and self.is_state_stale(max_age):
            return JointState()
        return self.get_current_joint_state() or JointState()

    def current_group_joint_state(
        self, group_id: PlanningGroupID, max_age: float = 1.0
    ) -> JointState:
        """Return current joint state scoped and ordered for one planning group."""
        group = self._planning_groups.get(group_id)
        if self._state_monitor is not None and self.is_state_stale(max_age):
            raise ValueError("Current model state is stale")
        state = self.get_current_joint_state()
        if state is None:
            raise ValueError("Current model state is unavailable")
        return filter_joint_state_to_selected_joints(state, group.joint_names)

    def _validate_planning_group_config(self, config: RobotModelConfig) -> None:
        """Validate planning groups before mutating world/backend state."""
        seen_group_names: set[str] = set()
        model_joint_names = set(config.joint_names)
        for definition in config.planning_groups:
            group_id = definition.name
            assert_valid_group_id(group_id)
            if definition.name in seen_group_names:
                raise ValueError(f"Planning group '{group_id}' is already registered")
            unknown_joint_names = set(definition.joint_names) - model_joint_names
            if unknown_joint_names:
                raise ValueError(
                    f"Planning group '{group_id}' contains unknown model joints: "
                    f"{sorted(unknown_joint_names)}"
                )
            seen_group_names.add(definition.name)

    def get_current_velocities(self) -> JointState | None:
        """Get current joint velocities as JointState. Returns None if not available."""
        if self._state_monitor is not None:
            velocities = self._state_monitor.get_current_velocities()
            if velocities is not None:
                return JointState(
                    name=self.get_model_config().joint_names, velocity=velocities.tolist()
                )
        return None

    def wait_for_state(self, timeout: float = 1.0) -> bool:
        """Wait until state is received. Returns False on timeout."""
        if self._state_monitor is not None:
            return self._state_monitor.wait_for_state(timeout)
        return False

    def is_state_stale(self, max_age: float = 1.0) -> bool:
        """Check if state is stale."""
        if self._state_monitor is not None:
            return self._state_monitor.is_state_stale(max_age)
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

    def is_state_valid(self, joint_state: JointState) -> bool:
        """Check if configuration is collision-free."""
        return self._world.check_config_collision_free(joint_state)

    def is_path_valid(self, path: JointPath, step_size: float = 0.05) -> bool:
        """Check if path is collision-free with interpolation.

        Args:
            path: List of JointState waypoints
            step_size: Max step size for interpolation (radians)

        Returns:
            True if entire path is collision-free
        """
        if len(path) < 2:
            return len(path) == 0 or self._world.check_config_collision_free(path[0])

        # Check each edge
        for i in range(len(path) - 1):
            if not self._world.check_edge_collision_free(path[i], path[i + 1], step_size):
                return False

        return True

    def get_min_distance(self) -> float:
        """Get minimum distance to obstacles for current state."""
        with self._world.scratch_context() as ctx:
            return self._world.get_min_distance(ctx)

    # Kinematics

    def get_ee_pose(self, joint_state: JointState | None = None) -> PoseStamped:
        """Get end-effector pose. Uses current state if joint_state is None."""
        group_id = self._planning_groups.primary_pose_group_id()
        if group_id is None:
            raise ValueError("Model has no unique pose-targetable planning group")
        return self.get_group_ee_pose(group_id, joint_state)

    def get_group_ee_pose(
        self, group_id: PlanningGroupID, joint_state: JointState | None = None
    ) -> PoseStamped:
        """Get planning-group tip pose. Uses current robot state if joint_state is None."""
        self._planning_groups.get(group_id)
        with self._world.scratch_context() as ctx:
            if joint_state is None:
                if self._state_monitor is not None and self.is_state_stale():
                    raise ValueError("Current model state is stale")
                joint_state = self.get_current_joint_state()
                if joint_state is None:
                    raise ValueError("Current model state is unavailable")
            self._world.set_joint_state(ctx, joint_state)

            return self._world.get_group_ee_pose(ctx, group_id)

    def get_link_pose(
        self, link_name: str, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        """Get arbitrary link pose as PoseStamped.

        Args:
            link_name: Name of the link in the URDF
            joint_state: Joint state to use (uses current if None)
        """
        with self._world.scratch_context() as ctx:
            if joint_state is None:
                joint_state = self.get_current_joint_state()
            if joint_state is not None:
                self._world.set_joint_state(ctx, joint_state)
            try:
                mat = self._world.get_link_pose(ctx, link_name)
            except KeyError:
                logger.warning(f"Link '{link_name}' not found in model")
                return None

            pos = mat[:3, 3]
            rot = mat[:3, :3]
            quat = Quaternion.from_rotation_matrix(rot)
            return PoseStamped(
                frame_id="world",
                position=[float(pos[0]), float(pos[1]), float(pos[2])],
                orientation=[float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
            )

    def get_jacobian(self, joint_state: JointState) -> NDArray[np.float64]:
        """Get 6xN Jacobian matrix."""
        group_id = self._planning_groups.primary_pose_group_id()
        if group_id is None:
            raise ValueError("Model has no unique pose-targetable planning group")
        return self.get_group_jacobian(group_id, joint_state)

    def get_group_jacobian(
        self, group_id: PlanningGroupID, joint_state: JointState
    ) -> NDArray[np.float64]:
        """Get 6xN planning-group Jacobian matrix."""
        self._planning_groups.get(group_id)
        with self._world.scratch_context() as ctx:
            self._world.set_joint_state(ctx, joint_state)
            return self._world.get_group_jacobian(ctx, group_id)

    # Lifecycle

    def finalize(
        self, visualization: VisualizationSpec | None = None, *, operator: object | None = None
    ) -> None:
        """Finalize the world and initialize visualization before later mutations."""
        session: VisualizationSession | None = None
        attached_visualization: VisualizationSpec | None = None
        with self._lock:
            self._world.finalize()
            if visualization is not None:
                self._visualization = visualization
            attached_visualization = self._visualization
            if attached_visualization is not None:
                session = VisualizationSession(
                    scene=PlanningSceneInfo(
                        model=self.get_model_config(),
                        planning_groups=tuple(self._planning_groups.list()),
                    ),
                    operator=operator,
                )
            logger.info("World finalized")
        if attached_visualization is not None and session is not None:
            attached_visualization.initialize(session)

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

    def visualization_state_frame(self) -> VisualizationStateFrame:
        """Build a pushed visualization state frame without freshness policy."""
        return VisualizationStateFrame(joint_state=self.get_current_joint_state())

    def update_visualization_state(self) -> None:
        """Push current state to visualization."""
        if self._visualization is not None:
            with self._visualization_lock:
                self._visualization.update_state(self.visualization_state_frame())

    def cancel_preview_animation(self) -> None:
        """Cancel active visualization preview animation."""
        if self._visualization is not None:
            self._visualization.cancel_preview_animation()

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        """Animate a raw generated-plan trajectory if visualization is available."""
        if self._visualization is not None:
            self._visualization.cancel_preview_animation()
            with self._visualization_lock:
                self._visualization.animate_trajectory(trajectory, duration)

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
                self.update_visualization_state()
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

    def get_state_monitor(self) -> RobotStateMonitor | None:
        """Get the state monitor (may be None)."""
        return self._state_monitor

    @property
    def obstacle_monitor(self) -> WorldObstacleMonitor | None:
        """Get obstacle monitor (may be None if not started)."""
        return self._obstacle_monitor
