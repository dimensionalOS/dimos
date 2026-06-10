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

"""Manipulation Module - Motion planning with ControlCoordinator execution.

Base module providing core manipulation infrastructure:
- @rpc: Low-level building blocks (plan_to_pose, plan_to_joints, execute)
- @skill (short-horizon): Single-step actions (move_to_pose, open_gripper, go_home, go_init)

Subclass PickAndPlaceModule (pick_and_place_module.py) adds perception integration
(scan_objects, get_scene_info) and long-horizon skills (pick, place, pick_and_place).
"""

from __future__ import annotations

from enum import Enum
import threading
import time
from typing import TYPE_CHECKING, Any, TypeAlias

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.planning.backends import PlannedMotion, create_planning_backend
from dimos.manipulation.planning.backends.base import PlanningBackend, SceneFacade
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import (
    IKResult,
    JointPath,
    Obstacle,
    PlanningResult,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import RPCClient

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


class ManipulationModuleConfig(ModuleConfig):
    """Configuration for ManipulationModule."""

    robots: list[RobotModelConfig] = Field(default_factory=list)
    planning_timeout: float = 10.0
    enable_viz: bool = False
    planning_backend: str = "drake"
    planning_backend_options: dict[str, Any] = Field(default_factory=dict)
    planner_name: str = "rrt_connect"  # "rrt_connect"
    kinematics_name: str = "jacobian"
    # Floor plane Z height (meters). When set, a box obstacle is added at startup
    # to prevent the planner from routing trajectories below this height.
    # Set to None to disable.
    floor_z: float | None = None


class ManipulationModule(Module):
    """Base motion planning module with ControlCoordinator execution.

    - @rpc: Low-level building blocks (plan, execute, gripper)
    - @skill (short-horizon): Single-step actions (move_to_pose, open_gripper, go_home)

    Subclass PickAndPlaceModule adds perception integration and long-horizon skills.
    """

    config: ManipulationModuleConfig

    # Input: Joint state from coordinator (for backend scene sync)
    joint_state: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # State machine
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""

        # Planning components (initialized in start())
        self._planning_backend: PlanningBackend | None = None

        # Robot registry: maps robot_name -> (world_robot_id, config, trajectory_gen)
        self._robots: RobotRegistry = {}

        # Stored path for plan/preview/execute workflow (per robot)
        self._planned_paths: PlannedPaths = {}
        self._planned_trajectories: PlannedTrajectories = {}

        # Coordinator integration (lazy initialized)
        self._coordinator_client: RPCClient | None = None

        # Init joints: captured from first joint state per robot, used by go_init
        self._init_joints: dict[RobotName, JointState] = {}

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

        logger.info("ManipulationModule started")

    def _initialize_planning(self) -> None:
        """Initialize active backend and trajectory generators."""
        if not self.config.robots:
            logger.warning("No robots configured, planning disabled")
            return

        self._planning_backend = create_planning_backend(
            name=self.config.planning_backend,
            planner_name=self.config.planner_name,
            kinematics_name=self.config.kinematics_name,
            options=self.config.planning_backend_options,
        )
        logger.info(
            "Planning backend selected",
            planning_backend=self.config.planning_backend,
            backend_class=type(self._planning_backend).__name__,
        )
        scene = self._planning_backend.scene()

        for robot_config in self.config.robots:
            backend_robot = scene.add_robot(robot_config)
            robot_id = backend_robot.robot_id
            traj_gen = JointTrajectoryGenerator(
                num_joints=len(robot_config.joint_names),
                max_velocity=robot_config.max_velocity,
                max_acceleration=robot_config.max_acceleration,
            )
            self._robots[robot_config.name] = (robot_id, robot_config, traj_gen)

        scene.finalize()

        # Add floor obstacle to prevent trajectories below the table surface
        if self.config.floor_z is not None:
            fz = self.config.floor_z
            thickness = 0.2
            floor_pose = Pose(
                Vector3(0.7, 0.0, fz - thickness / 2),
                Quaternion(0.0, 0.0, 0.0, 1.0),
            )
            floor_obs = Obstacle(
                name="floor",
                pose=floor_pose,
                obstacle_type=ObstacleType.BOX,
                dimensions=(0.6, 1.2, thickness),
            )
            scene.add_obstacle(floor_obs)
            logger.info(f"Floor obstacle added at z={fz:.3f}")

        for _, (robot_id, _, _) in self._robots.items():
            scene.start_state_monitor(robot_id)

        # Start TF publishing thread if any robot has tf_extra_links
        if any(c.tf_extra_links for _, c, _ in self._robots.values()):
            _ = self.tf  # Eager init
            self._tf_stop_event.clear()
            self._tf_thread = threading.Thread(
                target=self._tf_publish_loop, name="ManipTFThread", daemon=True
            )
            self._tf_thread.start()
            logger.info("TF publishing thread started")

    def _planning_scene(self) -> SceneFacade | None:
        if self._planning_backend is not None:
            return self._planning_backend.scene()
        return None

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
        if not robot_name:  # None or empty string (LLMs often pass "")
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
        """Callback when joint state received from driver.

        Splits the aggregated JointState by robot using each robot's
        coordinator joint names, then routes to the correct monitor.
        """
        try:
            scene = self._planning_scene()
            if scene is None:
                return

            # Build name → index map once for the whole message
            name_to_idx = {name: i for i, name in enumerate(msg.name)}

            for robot_name, (robot_id, config, _) in self._robots.items():
                coord_names = config.get_coordinator_joint_names()
                indices = [name_to_idx.get(cn) for cn in coord_names]
                if any(idx is None for idx in indices):
                    missing = [
                        cn for cn, idx in zip(coord_names, indices, strict=False) if idx is None
                    ]
                    logger.warning(f"Skipping '{robot_name}': missing joints {missing}")
                    continue

                # Build per-robot sub-message (coordinator namespace)
                sub_positions = [msg.position[idx] for idx in indices]  # type: ignore[index]
                sub_velocities = (
                    [msg.velocity[idx] for idx in indices]  # type: ignore[index]
                    if msg.velocity and len(msg.velocity) == len(msg.name)
                    else []
                )
                sub_msg = JointState(
                    name=list(coord_names),
                    position=sub_positions,
                    velocity=sub_velocities,
                )

                # Route to specific monitor
                scene.on_joint_state(sub_msg, robot_id=robot_id)

                # Capture per-robot init joints on first receipt
                if robot_name not in self._init_joints:
                    self._init_joints[robot_name] = sub_msg
                    logger.info(
                        f"Init joints captured for '{robot_name}': "
                        f"[{', '.join(f'{j:.3f}' for j in sub_positions)}]"
                    )

        except Exception as e:
            logger.error(f"Exception in _on_joint_state: {e}")
            import traceback

            logger.error(traceback.format_exc())

    def _tf_publish_loop(self) -> None:
        """Publish TF transforms at 10Hz for EE and extra links."""
        from dimos.msgs.geometry_msgs.Transform import Transform

        period = 0.1  # 10Hz
        while not self._tf_stop_event.is_set():
            try:
                scene = self._planning_scene()
                if scene is None:
                    break
                transforms: list[Transform] = []
                for robot_id, config, _ in self._robots.values():
                    # Publish world → EE
                    ee_pose = scene.get_ee_pose(robot_id)
                    if ee_pose is not None:
                        ee_tf = Transform.from_pose(config.end_effector_link, ee_pose)
                        ee_tf.frame_id = "world"
                        transforms.append(ee_tf)

                    # Publish world → each extra link
                    for link_name in config.tf_extra_links:
                        link_pose = scene.get_link_pose(robot_id, link_name)
                        if link_pose is not None:
                            link_tf = Transform.from_pose(link_name, link_pose)
                            link_tf.frame_id = "world"
                            transforms.append(link_tf)

                if transforms:
                    self.tf.publish(*transforms)
            except Exception as e:
                logger.debug(f"TF publish error: {e}")

            self._tf_stop_event.wait(period)

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
    @skill
    def reset(self) -> SkillResult[ManipulationSkillError]:
        """Reset the robot module to IDLE state, clearing any fault.

        Use this after an error or fault to allow new commands.
        Cannot reset while a motion is executing — cancel first.
        """
        if self._state == ManipulationState.EXECUTING:
            return SkillResult.fail(
                "INVALID_STATE",
                "Cannot reset while executing — cancel the motion first",
            )
        self._state = ManipulationState.IDLE
        self._error_message = ""
        return SkillResult.ok("Reset to IDLE — ready for new commands")

    @rpc
    def get_current_joints(self, robot_name: RobotName | None = None) -> list[float] | None:
        """Get current joint positions.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and (scene := self._planning_scene()):
            state = scene.get_current_joint_state(robot[1])
            if state is not None:
                return list(state.position)
        return None

    @rpc
    def get_ee_pose(self, robot_name: RobotName | None = None) -> Pose | None:
        """Get current end-effector pose.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and (scene := self._planning_scene()):
            return scene.get_ee_pose(robot[1], joint_state=None)
        return None

    @rpc
    def is_collision_free(self, joints: list[float], robot_name: RobotName | None = None) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check
            robot_name: Robot to check (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and (scene := self._planning_scene()):
            _, robot_id, config, _ = robot
            joint_state = JointState(name=config.joint_names, position=joints)
            return scene.is_state_valid(robot_id, joint_state)
        return False

    def _begin_planning(
        self, robot_name: RobotName | None = None
    ) -> tuple[RobotName, WorldRobotID] | None:
        """Check state and begin planning. Returns (robot_name, robot_id) or None.

        Args:
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._planning_scene() is None:
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
        """Plan motion to pose. Use Viser or get_planned_path() to inspect before execute().

        Args:
            pose: Target end-effector pose
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._planning_backend is None or (r := self._begin_planning(robot_name)) is None:
            return False
        robot_name, robot_id = r
        _, _, traj_gen = self._robots[robot_name]

        # Convert Pose to PoseStamped for the IK solver
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )

        result = self._planning_backend.planner().plan_to_pose(
            robot_id=robot_id,
            target_pose=target_pose,
            timeout=self.config.planning_timeout,
            trajectory_generator=traj_gen,
        )
        if isinstance(result, PlannedMotion):
            if result.ik_result is not None:
                logger.info(f"IK solved, error: {result.ik_result.position_error:.4f}m")
            return self._store_planned_motion(robot_name, result)
        if isinstance(result, IKResult):
            return self._fail(f"IK failed: {result.status.name}")
        if isinstance(result, PlanningResult):
            return self._fail(f"Planning failed: {result.status.name}")
        return self._fail("Planning failed")

    @rpc
    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        """Plan motion to joint config. Use Viser or get_planned_path() to inspect before execute().

        Args:
            joints: Target joint state (names + positions)
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if (r := self._begin_planning(robot_name)) is None:
            return False
        robot_name, robot_id = r
        logger.info(f"Planning to joints for {robot_name}: {[f'{j:.3f}' for j in joints.position]}")
        return self._plan_path_only(robot_name, robot_id, joints)

    def _plan_path_only(
        self, robot_name: RobotName, robot_id: WorldRobotID, goal: JointState
    ) -> bool:
        """Plan path from current position to goal, store result."""
        assert self._planning_backend  # guaranteed by _begin_planning
        _, _, traj_gen = self._robots[robot_name]
        result = self._planning_backend.planner().plan_to_joints(
            robot_id=robot_id,
            goal=goal,
            timeout=self.config.planning_timeout,
            trajectory_generator=traj_gen,
        )
        if isinstance(result, PlanningResult):
            return self._fail(f"Planning failed: {result.status.name}")
        return self._store_planned_motion(robot_name, result)

    def _store_planned_motion(self, robot_name: RobotName, motion: PlannedMotion) -> bool:
        logger.info(f"Path: {len(motion.path)} waypoints")
        self._planned_paths[robot_name] = motion.path
        self._planned_trajectories[robot_name] = motion.trajectory
        logger.info(f"Trajectory: {motion.trajectory.duration:.3f}s")

        self._state = ManipulationState.COMPLETED
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
    def get_planned_path(self, robot_name: RobotName | None = None) -> JointPath | None:
        """Get the currently stored planned path for a robot.

        Args:
            robot_name: Robot to query (required if multiple robots configured)

        Returns:
            Planned path waypoints or None if no path is stored
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return None
        return self._planned_paths.get(robot[0])

    @rpc
    def get_planned_path_poses(self, robot_name: RobotName | None = None) -> list[Pose] | None:
        """Get end-effector poses for the stored planned path.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        scene = self._planning_scene()
        if scene is None:
            return None
        robot = self._get_robot(robot_name)
        if robot is None:
            return None
        robot_name, robot_id, _, _ = robot
        path = self._planned_paths.get(robot_name)
        if path is None:
            return None
        poses = [scene.get_ee_pose(robot_id, joint_state=waypoint) for waypoint in path]
        return [pose for pose in poses if pose is not None]

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
    def get_robot_info(self, robot_name: RobotName | None = None) -> dict[str, Any] | None:
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

        joint_limits = None
        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            joint_limits = list(
                zip(config.joint_limits_lower, config.joint_limits_upper, strict=False)
            )
        elif (scene := self._planning_scene()) is not None:
            try:
                lower, upper = scene.get_joint_limits(robot_id)
                joint_limits = [
                    (float(lower_value), float(upper_value))
                    for lower_value, upper_value in zip(lower, upper, strict=False)
                ]
            except Exception as e:
                logger.debug(f"Could not load joint limits for '{robot_name}': {e}")

        return {
            "name": config.name,
            "world_robot_id": robot_id,
            "joint_names": config.joint_names,
            "end_effector_link": config.end_effector_link,
            "base_link": config.base_link,
            "model_path": str(config.model_path),
            "base_pose": config.base_pose,
            "joint_limits": joint_limits,
            "package_paths": {package: str(path) for package, path in config.package_paths.items()},
            "xacro_args": dict(config.xacro_args),
            "max_velocity": config.max_velocity,
            "max_acceleration": config.max_acceleration,
            "has_joint_name_mapping": bool(config.joint_name_mapping),
            "coordinator_task_name": config.coordinator_task_name,
            "home_joints": config.home_joints,
            "pre_grasp_offset": config.pre_grasp_offset,
            "init_joints": list(init.position)
            if (init := self._init_joints.get(robot_name))
            else None,
        }

    @rpc
    def evaluate_pose_target(
        self, pose: Pose, robot_name: RobotName | None = None
    ) -> dict[str, Any]:
        """Evaluate a pose target without storing, previewing, executing, or moving.

        Args:
            pose: Target end-effector pose in world coordinates
            robot_name: Robot to solve for (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNKNOWN_ROBOT",
                "message": "Robot not found",
                "position_error": None,
                "orientation_error": None,
                "collision_free": False,
            }
        robot_name, robot_id, _, traj_gen = robot
        scene = self._planning_scene()
        if scene is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized",
                "position_error": None,
                "orientation_error": None,
                "collision_free": False,
            }
        current = scene.get_current_joint_state(robot_id)
        if current is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "NO_JOINT_STATE",
                "message": "No joint state",
                "position_error": None,
                "orientation_error": None,
                "collision_free": False,
            }

        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )
        if self._planning_backend is not None:
            result = self._planning_backend.planner().plan_to_pose(
                robot_id=robot_id,
                target_pose=target_pose,
                timeout=getattr(
                    getattr(self, "config", None),
                    "planning_timeout",
                    ManipulationModuleConfig().planning_timeout,
                ),
                trajectory_generator=traj_gen,
            )
            if isinstance(result, PlannedMotion) and result.ik_result is not None:
                ik = result.ik_result
                joint_state = ik.joint_state if ik.is_success() else None
            elif isinstance(result, IKResult):
                ik = result
                joint_state = None
            elif isinstance(result, PlanningResult):
                return {
                    "success": False,
                    "joint_state": None,
                    "status": result.status.name,
                    "message": result.message,
                    "position_error": None,
                    "orientation_error": None,
                    "collision_free": False,
                }
            else:
                return {
                    "success": False,
                    "joint_state": None,
                    "status": "NO_SOLUTION",
                    "message": "Pose evaluation failed",
                    "position_error": None,
                    "orientation_error": None,
                    "collision_free": False,
                }
        else:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized",
                "position_error": None,
                "orientation_error": None,
                "collision_free": False,
            }
        collision_free = bool(
            joint_state is not None and scene.is_state_valid(robot_id, joint_state)
        )
        return {
            "success": joint_state is not None and collision_free,
            "joint_state": joint_state,
            "status": ik.status.name,
            "message": ik.message,
            "position_error": ik.position_error,
            "orientation_error": ik.orientation_error,
            "collision_free": collision_free,
        }

    @rpc
    def evaluate_joint_target(
        self, joints: JointState, robot_name: RobotName | None = None
    ) -> dict[str, Any]:
        """Evaluate candidate joints without planning or moving.

        Args:
            joints: Candidate joint state
            robot_name: Robot to solve for (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return {
                "success": False,
                "pose": None,
                "joint_state": None,
                "status": "UNKNOWN_ROBOT",
                "message": "Robot not found",
                "collision_free": False,
            }
        _, robot_id, config, _ = robot
        scene = self._planning_scene()
        if scene is None:
            return {
                "success": False,
                "pose": None,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized",
                "collision_free": False,
            }
        joint_state = joints
        if not joint_state.name:
            joint_state = JointState.__new__(JointState)
            joint_state.ts = joints.ts
            joint_state.frame_id = joints.frame_id
            joint_state.name = config.joint_names
            joint_state.position = list(joints.position)
            joint_state.velocity = list(joints.velocity or [])
            joint_state.effort = list(joints.effort or [])
        pose = scene.get_ee_pose(robot_id, joint_state=joint_state)
        collision_free = scene.is_state_valid(robot_id, joint_state)
        return {
            "success": collision_free,
            "pose": pose,
            "joint_state": joint_state,
            "status": "SUCCESS" if collision_free else "COLLISION",
            "message": "" if collision_free else "Joint state is in collision or invalid",
            "collision_free": collision_free,
        }

    @rpc
    def get_init_joints(self, robot_name: RobotName | None = None) -> JointState | None:
        """Get the init joint state (captured at startup or set manually).

        Args:
            robot_name: Robot name (uses default if None and only one robot)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return None
        return self._init_joints.get(robot[0])

    @rpc
    def set_init_joints(self, joint_state: JointState, robot_name: RobotName | None = None) -> bool:
        """Set the init joint state.

        Args:
            joint_state: New init joint state (names + positions)
            robot_name: Robot name (uses default if None and only one robot)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        self._init_joints[robot[0]] = joint_state
        logger.info(
            f"Init joints set for '{robot[0]}': "
            f"[{', '.join(f'{j:.3f}' for j in joint_state.position)}]"
        )
        return True

    @rpc
    def set_init_joints_to_current(self, robot_name: RobotName | None = None) -> bool:
        """Set init joints to the current joint positions.

        Args:
            robot_name: Robot to capture from (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        robot_name_resolved, robot_id, _, _ = robot
        scene = self._planning_scene()
        if scene is None:
            return False
        current = scene.get_current_joint_state(robot_id)
        if current is None:
            logger.error("Cannot capture init joints — no current joint state")
            return False
        self._init_joints[robot_name_resolved] = current
        logger.info(
            f"Init joints set to current for '{robot_name_resolved}': "
            f"[{', '.join(f'{j:.3f}' for j in current.position)}]"
        )
        return True

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
    def get_trajectory_status(self, robot_name: RobotName | None = None) -> dict[str, Any] | None:
        """Get trajectory execution status via coordinator task_invoke."""
        if (robot := self._get_robot(robot_name)) is None:
            return None
        _, _, config, _ = robot
        if not config.coordinator_task_name or (client := self._get_coordinator_client()) is None:
            return None
        try:
            state = client.task_invoke(config.coordinator_task_name, "get_state", {})
            if state is not None:
                return {"state": int(state), "task": config.coordinator_task_name}
            return None
        except Exception:
            return None

    @property
    def planning_backend(self) -> PlanningBackend | None:
        return self._planning_backend

    @rpc
    def get_planning_backend_info(self) -> dict[str, Any] | None:
        if self._planning_backend is None:
            return None
        capabilities = self._planning_backend.capabilities()
        return {
            "name": self._planning_backend.name,
            "capabilities": capabilities.__dict__,
            "diagnostics": self._planning_backend.diagnostics().as_dict(),
        }

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
        scene = self._planning_scene()
        if scene is None:
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
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        obstacle = Obstacle(
            name=name,
            obstacle_type=obstacle_type,
            pose=PoseStamped(position=pose.position, orientation=pose.orientation),
            dimensions=tuple(dimensions) if dimensions else (),
            mesh_path=mesh_path,
        )
        return scene.add_obstacle(obstacle)

    @rpc
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the planning scene."""
        scene = self._planning_scene()
        if scene is None:
            return False
        return scene.remove_obstacle(obstacle_id)

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

    def _set_gripper_position(self, position: float, robot_name: RobotName | None = None) -> bool:
        """Internal: set gripper position in meters."""
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

    @skill
    def set_gripper(
        self, position: float, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Set gripper to a specific opening in meters.

        Args:
            position: Gripper opening in meters (0.0 = closed, 0.85 = fully open).
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        if self._set_gripper_position(position, robot_name):
            return SkillResult.ok(f"Gripper set to {position:.3f}m")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to set gripper position")

    @skill
    def open_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Open the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        if self._set_gripper_position(0.85, robot_name):
            return SkillResult.ok("Gripper opened")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to open gripper")

    @skill
    def close_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Close the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        if self._set_gripper_position(0.0, robot_name):
            return SkillResult.ok("Gripper closed")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to close gripper")

    def _wait_for_trajectory_completion(
        self, robot_name: RobotName | None = None, timeout: float = 60.0, poll_interval: float = 0.2
    ) -> bool:
        """Wait for trajectory execution to complete.

        Polls the coordinator task state via task_invoke. Falls back to waiting
        for the trajectory duration if the coordinator is unavailable.

        Args:
            robot_name: Robot to monitor
            timeout: Maximum wait time in seconds
            poll_interval: Time between status checks

        Returns:
            True if trajectory completed successfully
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return True
        rname, _, config, _ = robot
        client = self._get_coordinator_client()

        if client is None or not config.coordinator_task_name:
            # No coordinator — wait for trajectory duration as fallback
            traj = self._planned_trajectories.get(rname)
            if traj is not None:
                logger.info(f"No coordinator status — waiting {traj.duration:.1f}s for trajectory")
                time.sleep(traj.duration + 0.5)
            return True

        # Poll task state via task_invoke
        start = time.time()
        while (time.time() - start) < timeout:
            try:
                state = client.task_invoke(config.coordinator_task_name, "get_state", {})
                # TrajectoryState is an IntEnum: IDLE=0, EXECUTING=1, COMPLETED=2, ABORTED=3, FAULT=4
                if state is not None:
                    state_val = int(state)
                    if state_val in (0, 2):  # IDLE or COMPLETED
                        return True
                    if state_val in (3, 4):  # ABORTED or FAULT
                        logger.warning(f"Trajectory failed: state={state}")
                        return False
                    # state_val == 1 means EXECUTING, keep polling
                else:
                    # task_invoke returned None — task not found, assume done
                    return True
            except Exception:
                # Fallback: wait for trajectory duration
                traj = self._planned_trajectories.get(rname)
                if traj is not None:
                    remaining = traj.duration - (time.time() - start)
                    if remaining > 0:
                        logger.info(f"Status poll failed — waiting {remaining:.1f}s for trajectory")
                        time.sleep(remaining + 0.5)
                return True
            time.sleep(poll_interval)

        logger.warning(f"Trajectory execution timed out after {timeout}s")
        return False

    def _lift_if_low(
        self, robot_name: RobotName | None = None, min_z: float = 0.05
    ) -> SkillResult[ManipulationSkillError]:
        """If the end-effector is below *min_z*, plan and execute a short lift."""
        ee = self.get_ee_pose(robot_name)
        if ee is None or ee.position.z >= min_z:
            return SkillResult.ok()

        lift_z = min_z + 0.05
        logger.info(f"EE z={ee.position.z:.3f} < {min_z}, lifting to z={lift_z:.3f}")
        lift_pose = Pose(Vector3(ee.position.x, ee.position.y, lift_z), ee.orientation)
        if not self.plan_to_pose(lift_pose, robot_name):
            return SkillResult.fail(
                "PLANNING_FAILED",
                f"Failed to plan lift from z={ee.position.z:.3f}",
            )
        return self._execute_wait(robot_name)

    def _execute_wait(
        self, robot_name: RobotName | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Execute planned path and wait for completion.

        Args:
            robot_name: Robot to operate on
        """
        logger.info("Executing trajectory...")
        if not self.execute(robot_name):
            return SkillResult.fail("EXECUTION_FAILED", "Trajectory execution failed")

        if not self._wait_for_trajectory_completion(robot_name):
            return SkillResult.fail("EXECUTION_TIMEOUT", "Trajectory execution timed out")

        return SkillResult.ok()

    @skill
    def get_robot_state(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Get current robot state: joint positions, end-effector pose, and gripper.

        Args:
            robot_name: Robot to query (only needed for multi-arm setups).
        """
        lines: list[str] = []

        joints = self.get_current_joints(robot_name)
        if joints is not None:
            lines.append(f"Joints: [{', '.join(f'{j:.3f}' for j in joints)}]")
        else:
            lines.append("Joints: unavailable (no state received)")

        ee_pose = self.get_ee_pose(robot_name)
        if ee_pose is not None:
            p = ee_pose.position
            lines.append(f"EE pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
        else:
            lines.append("EE pose: unavailable")

        gripper_pos = self.get_gripper(robot_name)
        if gripper_pos is not None:
            lines.append(f"Gripper: {gripper_pos:.3f}m")
        else:
            lines.append("Gripper: not configured")

        lines.append(f"State: {self.get_state()}")

        return SkillResult.ok("\n".join(lines))

    @skill
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot end-effector to a target pose.

        Plans a collision-free trajectory and executes it.
        If roll/pitch/yaw are omitted, the current EE orientation is preserved.

        Args:
            x: Target X position in meters.
            y: Target Y position in meters.
            z: Target Z position in meters.
            roll: Target roll in radians (omit to keep current orientation).
            pitch: Target pitch in radians (omit to keep current orientation).
            yaw: Target yaw in radians (omit to keep current orientation).
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        logger.info(f"Planning motion to ({x:.3f}, {y:.3f}, {z:.3f})...")

        # If no orientation specified, preserve the current EE orientation.
        # If partially specified, fill unspecified angles from current orientation.
        if roll is None and pitch is None and yaw is None:
            current_pose = self.get_ee_pose(robot_name)
            if current_pose is not None:
                orientation = current_pose.orientation
            else:
                orientation = Quaternion(0, 0, 0, 1)  # identity fallback
        else:
            current_pose = self.get_ee_pose(robot_name)
            if current_pose is not None:
                current_euler = current_pose.orientation.to_euler()
                orientation = Quaternion.from_euler(
                    Vector3(
                        roll if roll is not None else current_euler.x,
                        pitch if pitch is not None else current_euler.y,
                        yaw if yaw is not None else current_euler.z,
                    )
                )
            else:
                orientation = Quaternion.from_euler(Vector3(roll or 0.0, pitch or 0.0, yaw or 0.0))

        pose = Pose(Vector3(x, y, z), orientation)

        # If EE is low, lift up first to clear obstacles
        lift = self._lift_if_low(robot_name)
        if not lift.is_success():
            return lift

        if not self.plan_to_pose(pose, robot_name):
            return SkillResult.fail(
                "PLANNING_FAILED",
                f"Pose ({x:.3f}, {y:.3f}, {z:.3f}) may be unreachable or in collision",
            )

        exec_result = self._execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok(f"Reached target pose ({x:.3f}, {y:.3f}, {z:.3f})")

    @skill
    def move_to_joints(
        self,
        joints: str,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to a target joint configuration.

        Plans a collision-free trajectory and executes it.

        Args:
            joints: Comma-separated joint positions in radians, e.g. "0.1, -0.5, 1.2, 0.0, 0.3, -0.1".
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        try:
            joint_values = [float(j.strip()) for j in joints.split(",")]
        except ValueError:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"Invalid joints format '{joints}'. Expected comma-separated floats.",
            )

        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot
        goal = JointState(name=config.joint_names, position=joint_values)

        logger.info(f"Planning motion to joints [{', '.join(f'{j:.3f}' for j in joint_values)}]...")
        if not self.plan_to_joints(goal, rname):
            return SkillResult.fail(
                "PLANNING_FAILED",
                "Joint configuration may be unreachable or in collision",
            )

        exec_result = self._execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached target joint configuration")

    @skill
    def go_home(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its home/observe joint configuration.

        Opens the gripper and moves to the predefined home position.

        Args:
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot

        if config.home_joints is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "No home_joints configured for this robot",
            )

        logger.info("Opening gripper...")
        self._set_gripper_position(0.85, rname)
        time.sleep(0.5)

        goal = JointState(name=config.joint_names, position=config.home_joints)
        logger.info("Planning motion to home position...")
        if not self.plan_to_joints(goal, rname):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to home position")

        exec_result = self._execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached home position")

    @skill
    def go_init(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its init position (captured at startup or set manually).

        The init position is the joint configuration the robot was in when the
        module first received joint state. It can be changed with set_init_joints().

        Args:
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, robot_id, _, _ = robot

        init = self._init_joints.get(rname)
        if init is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "No init joints captured — robot may not have reported joint state yet",
            )

        # Lift if EE is low before moving to init
        lift = self._lift_if_low(robot_name)
        if not lift.is_success():
            return lift

        # Move through a safe waypoint: 10cm above and 5cm in front of init pose.
        # This avoids direct paths through the workspace that could collide with objects.
        if (scene := self._planning_scene()) is not None:
            init_ee = scene.get_ee_pose(robot_id, joint_state=init)
            if init_ee is not None:
                wp = Pose(
                    Vector3(
                        init_ee.position.x + 0.05,
                        init_ee.position.y,
                        init_ee.position.z + 0.10,
                    ),
                    init_ee.orientation,
                )
                if self.plan_to_pose(wp, robot_name):
                    wp_result = self._execute_wait(robot_name)
                    if not wp_result.is_success():
                        return wp_result
                else:
                    logger.warning("Safe waypoint unreachable, going directly to init")

        logger.info(
            f"Planning motion to init position [{', '.join(f'{j:.3f}' for j in init.position)}]..."
        )
        if not self.plan_to_joints(init, robot_name):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to init position")

        exec_result = self._execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached init position")

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        # Stop TF thread
        if self._tf_thread is not None:
            self._tf_stop_event.set()
            self._tf_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._tf_thread = None

        # Stop active planning backend (includes visualization thread)
        if self._planning_backend is not None:
            self._planning_backend.stop()

        super().stop()
