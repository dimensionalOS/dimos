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
from pathlib import Path
import threading
from typing import TYPE_CHECKING, TypeAlias

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core import In, Module, rpc
from dimos.core.docker_runner import DockerModule as DockerRunner
from dimos.core.module import ModuleConfig
from dimos.manipulation.grasping.graspgen_module import GraspGenModule
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
from dimos.perception.detection.type.detection3d.object import Object as DetObject  # noqa: TC001
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import RPCClient
    from dimos.msgs.geometry_msgs import Pose, PoseArray
    from dimos.msgs.sensor_msgs import PointCloud2

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

# The host-side path (graspgen_visualization_output_path) is volume-mounted here.
_GRASPGEN_VIZ_CONTAINER_DIR = "/output/graspgen"
_GRASPGEN_VIZ_CONTAINER_PATH = f"{_GRASPGEN_VIZ_CONTAINER_DIR}/visualization.json"


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

    # GraspGen Docker settings (optional)
    graspgen_docker_image: str = "dimos-graspgen:latest"
    graspgen_gripper_type: str = "robotiq_2f_140"
    graspgen_num_grasps: int = 400
    graspgen_topk_num_grasps: int = 100
    graspgen_grasp_threshold: float = -1.0
    graspgen_filter_collisions: bool = False
    graspgen_save_visualization_data: bool = False
    graspgen_visualization_output_path: Path = field(
        default_factory=lambda: Path.home() / ".dimos" / "graspgen" / "visualization.json"
    )


class ManipulationModule(Module):
    """Motion planning module with ControlCoordinator execution."""

    default_config = ManipulationModuleConfig

    # Type annotation for the config attribute (mypy uses this)
    config: ManipulationModuleConfig

    # Input: Joint state from coordinator (for world sync)
    joint_state: In[JointState]

    # Input: Objects from perception (for obstacle integration)
    objects: In[list[DetObject]]

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

        # GraspGen Docker runner (lazy initialized on first generate_grasps call)
        self._graspgen: DockerRunner | None = None

        # TF publishing thread
        self._tf_stop_event = threading.Event()
        self._tf_thread: threading.Thread | None = None

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

        # Subscribe to objects port for perception obstacle integration
        if self.objects is not None:
            self.objects.observable().subscribe(self._on_objects)  # type: ignore[no-untyped-call]
            logger.info("Subscribed to objects port (async)")

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

        # Start obstacle monitor for perception integration
        self._world_monitor.start_obstacle_monitor()

        if self.config.enable_viz:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info(f"Visualization: {url}")

        self._planner = create_planner(name=self.config.planner_name)
        self._kinematics = create_kinematics(name=self.config.kinematics_name)

        # Start TF publishing thread if any robot has tf_extra_links
        if any(c.tf_extra_links for _, c, _ in self._robots.values()):
            _ = self.tf  # Eager init — lazy init blocks in Dask workers
            self._tf_stop_event.clear()
            self._tf_thread = threading.Thread(
                target=self._tf_publish_loop, name="ManipTFThread", daemon=True
            )
            self._tf_thread.start()
            logger.info("TF publishing thread started")

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

    def _on_objects(self, objects: list[DetObject]) -> None:
        """Callback when objects received from perception (runs on RxPY thread pool)."""
        try:
            if self._world_monitor is not None:
                self._world_monitor.on_objects(objects)
        except Exception as e:
            logger.error(f"Exception in _on_objects: {e}")

    def _tf_publish_loop(self) -> None:
        """Publish TF transforms at 10Hz for EE and extra links."""
        from dimos.msgs.geometry_msgs import Transform

        period = 0.1  # 10Hz
        while not self._tf_stop_event.is_set():
            try:
                if self._world_monitor is None:
                    break
                transforms: list[Transform] = []
                for robot_id, config, _ in self._robots.values():
                    # Publish world → EE
                    ee_pose = self._world_monitor.get_ee_pose(robot_id)
                    if ee_pose is not None:
                        ee_tf = Transform.from_pose(config.end_effector_link, ee_pose)
                        ee_tf.frame_id = "world"
                        transforms.append(ee_tf)

                    # Publish world → each extra link
                    for link_name in config.tf_extra_links:
                        link_pose = self._world_monitor.get_link_pose(robot_id, link_name)
                        if link_pose is not None:
                            link_tf = Transform.from_pose(link_name, link_pose)
                            link_tf.frame_id = "world"
                            transforms.append(link_tf)

                if transforms:
                    self.tf.publish(*transforms)
            except Exception as e:
                logger.debug(f"TF publish error: {e}")

            self._tf_stop_event.wait(period)

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

    def _dismiss_preview(self, robot_id: WorldRobotID) -> None:
        """Hide the preview ghost if the world supports it."""
        if self._world_monitor is None:
            return
        world = self._world_monitor.world
        if hasattr(world, "hide_preview"):
            world.hide_preview(robot_id)  # type: ignore[attr-defined]
            world.publish_visualization()

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
        self._dismiss_preview(robot_id)
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
        interpolated = interpolate_path(planned_path, resolution=0.1)
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
        if not any(
            c.coordinator_task_name or c.gripper_hardware_id for _, c, _ in self._robots.values()
        ):
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
        result = client.task_invoke(
            config.coordinator_task_name, "execute", {"trajectory": translated}
        )
        if result:
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

    def _get_graspgen(self) -> DockerRunner:
        """Get or create GraspGen Docker module (lazy init, thread-safe)."""
        # Fast path: already initialized (no lock needed for read)
        if self._graspgen is not None:
            return self._graspgen

        # Slow path: need to initialize (acquire lock to prevent race condition)
        with self._lock:
            # Double-check: another thread may have initialized while we waited for lock
            if self._graspgen is not None:
                return self._graspgen

            # Ensure GraspGen model checkpoints are pulled from LFS
            get_data("models_graspgen")

            docker_file = (
                DIMOS_PROJECT_ROOT
                / "dimos"
                / "manipulation"
                / "grasping"
                / "docker_context"
                / "Dockerfile"
            )

            # Auto-mount host directory for visualization output when enabled.
            docker_volumes: list[tuple[str, str, str]] = []
            if self.config.graspgen_save_visualization_data:
                host_dir = self.config.graspgen_visualization_output_path.parent
                host_dir.mkdir(parents=True, exist_ok=True)
                docker_volumes.append((str(host_dir), _GRASPGEN_VIZ_CONTAINER_DIR, "rw"))

            graspgen = DockerRunner(
                GraspGenModule,  # type: ignore[arg-type]
                docker_file=docker_file,
                docker_build_context=DIMOS_PROJECT_ROOT,
                docker_image=self.config.graspgen_docker_image,
                docker_env={"CI": "1"},  # skip interactive system config prompt in container
                docker_volumes=docker_volumes,
                gripper_type=self.config.graspgen_gripper_type,
                num_grasps=self.config.graspgen_num_grasps,
                topk_num_grasps=self.config.graspgen_topk_num_grasps,
                grasp_threshold=self.config.graspgen_grasp_threshold,
                filter_collisions=self.config.graspgen_filter_collisions,
                save_visualization_data=self.config.graspgen_save_visualization_data,
                visualization_output_path=_GRASPGEN_VIZ_CONTAINER_PATH,
            )
            graspgen.start()
            self._graspgen = graspgen  # cache only after successful start
            return self._graspgen

    @rpc
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None:
        """Generate grasp poses for the given point cloud via GraspGen Docker module."""
        try:
            graspgen = self._get_graspgen()
            return graspgen.generate_grasps(pointcloud, scene_pointcloud)  # type: ignore[no-any-return]
        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

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

    # =========================================================================
    # Perception RPC Methods
    # =========================================================================

    @rpc
    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, object]]:
        """Refresh perception obstacles. Returns the list of obstacles added."""
        if self._world_monitor is None:
            return []
        return self._world_monitor.refresh_obstacles(min_duration)

    @rpc
    def clear_perception_obstacles(self) -> int:
        """Remove all perception obstacles. Returns count removed."""
        if self._world_monitor is None:
            return 0
        return self._world_monitor.clear_perception_obstacles()

    @rpc
    def get_perception_status(self) -> dict[str, int]:
        """Get perception obstacle status (cached/added counts)."""
        if self._world_monitor is None:
            return {"cached": 0, "added": 0}
        return self._world_monitor.get_perception_status()

    @rpc
    def list_cached_detections(self) -> list[dict[str, object]]:
        """List cached detections from perception."""
        if self._world_monitor is None:
            return []
        return self._world_monitor.list_cached_detections()

    @rpc
    def list_added_obstacles(self) -> list[dict[str, object]]:
        """List perception obstacles currently in the planning world."""
        if self._world_monitor is None:
            return []
        return self._world_monitor.list_added_obstacles()

    # =========================================================================
    # Gripper RPC Methods
    # =========================================================================

    def _get_gripper_hardware_id(self, robot_name: RobotName | None = None) -> str | None:
        """Get gripper hardware ID for a robot."""
        robot = self._get_robot(robot_name)
        if robot is None:
            return None
        _, _, config, _ = robot
        if not config.gripper_hardware_id:
            logger.warning(f"No gripper_hardware_id configured for '{config.name}'")
            return None
        return str(config.gripper_hardware_id)

    @rpc
    def set_gripper(self, position: float, robot_name: RobotName | None = None) -> bool:
        """Set gripper position in meters.

        Args:
            position: Gripper position in meters
            robot_name: Robot to control (required if multiple robots configured)
        """
        hw_id = self._get_gripper_hardware_id(robot_name)
        if hw_id is None:
            return False
        client = self._get_coordinator_client()
        if client is None:
            logger.error("No coordinator client for gripper control")
            return False
        return bool(client.set_gripper_position(hw_id, position))

    @rpc
    def get_gripper(self, robot_name: RobotName | None = None) -> float | None:
        """Get gripper position in meters.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        hw_id = self._get_gripper_hardware_id(robot_name)
        if hw_id is None:
            return None
        client = self._get_coordinator_client()
        if client is None:
            return None
        result = client.get_gripper_position(hw_id)
        return float(result) if result is not None else None

    @rpc
    def open_gripper(self, robot_name: RobotName | None = None) -> bool:
        """Open gripper fully (0.85m opening).

        Args:
            robot_name: Robot to control (required if multiple robots configured)
        """
        return bool(self.set_gripper(0.85, robot_name))

    @rpc
    def close_gripper(self, robot_name: RobotName | None = None) -> bool:
        """Close gripper fully.

        Args:
            robot_name: Robot to control (required if multiple robots configured)
        """
        return bool(self.set_gripper(0.0, robot_name))

    # =========================================================================
    # Lifecycle
    # =========================================================================

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        # Stop GraspGen Docker container (thread-safe access to shared state)
        with self._lock:
            if self._graspgen is not None:
                self._graspgen.stop()
                self._graspgen = None

        # Stop TF thread
        if self._tf_thread is not None:
            self._tf_stop_event.set()
            self._tf_thread.join(timeout=1.0)
            self._tf_thread = None

        # Stop world monitor (includes visualization thread)
        if self._world_monitor is not None:
            self._world_monitor.stop_all_monitors()

        super().stop()


# Expose blueprint for declarative composition
manipulation_module = ManipulationModule.blueprint
