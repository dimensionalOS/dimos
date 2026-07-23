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
- @rpc: Low-level building blocks (plan_to_pose, plan_to_joints, preview_path, execute)
- @skill (short-horizon): Single-step actions (move_to_pose, open_gripper, go_home, go_init)

Subclass PickAndPlaceModule (pick_and_place_module.py) adds perception integration
(scan_objects, get_scene_info) and long-horizon skills (pick, place, pick_and_place).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from enum import Enum
import math
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
from dimos.manipulation.planning.factory import (
    KinematicsName,
    PlannerName,
    WorldBackend,
    create_planning_specs,
    create_world,
)
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    joint_target_to_global_names,
    planning_group_id_from_selector,
)
from dimos.manipulation.planning.kinematics.config import (
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
)
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, ObstacleType
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    IKResult,
    Obstacle,
    PlanningGroupID,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import KinematicsSpec, PlannerSpec
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.factory import create_manipulation_visualization
from dimos.manipulation.visualization.operator import ManipulationOperator
from dimos.manipulation.visualization.types import TargetEvaluation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import RPCClient

logger = setup_logger()

# Composite type aliases for readability (using semantic IDs from planning.spec)
RobotEntry: TypeAlias = tuple[WorldRobotID, RobotModelConfig, JointTrajectoryGenerator]
"""(world_robot_id, config, trajectory_generator)"""

RobotRegistry: TypeAlias = dict[RobotName, RobotEntry]
"""Maps robot_name -> RobotEntry"""

RobotInfoValue: TypeAlias = (
    str | bool | float | list[str] | list[float] | list[PlanningGroup] | None
)
RobotInfoPayload: TypeAlias = dict[str, RobotInfoValue]


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
    world_backend: WorldBackend = "drake"
    visualization: ManipulationVisualizationConfig = Field(
        default_factory=NoManipulationVisualizationConfig
    )
    planner_name: PlannerName = "rrt_connect"
    kinematics: ManipulationKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    # Deprecated: use kinematics.backend instead.
    kinematics_name: KinematicsName | None = None
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

    # Input: Joint state from coordinator (for world sync)
    coordinator_joint_state: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # State machine
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        # State transitions are local, but coordinator calls must be serialized
        # with cancellation.  In particular, do not let cancel race between
        # two arms of a multi-arm dispatch.
        self._execution_dispatch_lock = threading.Lock()
        self._execution_generation = 0
        self._possibly_active_tasks: set[str] = set()
        self._cancellation_in_progress = False
        self._error_message = ""
        self._planning_epoch = 0

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner: PlannerSpec | None = None
        self._kinematics: KinematicsSpec | None = None

        # Robot registry: maps robot_name -> (world_robot_id, config, trajectory_gen)
        self._robots: RobotRegistry = {}

        # Canonical generated plan for plan/preview/execute workflow.
        # Robot-local paths and trajectories are derived from this plan on demand.
        self._last_plan: GeneratedPlan | None = None

        # Coordinator integration (lazy initialized)
        self._coordinator_client: RPCClient | None = None

        # Init joints: captured from first joint state per robot, used by go_init
        self._init_joints: dict[RobotName, JointState] = {}

        # TF publishing thread
        self._tf_stop_event = threading.Event()
        self._tf_thread: threading.Thread | None = None

        logger.info("ManipulationModule initialized")

    def _ensure_execution_transaction_state(self) -> None:
        """Initialize transaction fields for lightweight test harness subclasses."""
        if not hasattr(self, "_execution_dispatch_lock"):
            self._execution_dispatch_lock = threading.Lock()
        if not hasattr(self, "_execution_generation"):
            self._execution_generation = 0
        if not hasattr(self, "_possibly_active_tasks"):
            self._possibly_active_tasks = set()
        if not hasattr(self, "_cancellation_in_progress"):
            self._cancellation_in_progress = False

    @rpc
    def start(self) -> None:
        """Start the manipulation module."""
        super().start()

        # Initialize planning stack
        self._initialize_planning()

        # Subscribe to joint state via port
        if self.coordinator_joint_state is not None:
            self.coordinator_joint_state.subscribe(self._on_joint_state)
            logger.info("Subscribed to coordinator_joint_state port")

        logger.info("ManipulationModule started")

    def _initialize_planning(self) -> None:
        """Initialize world, planner, and trajectory generator."""
        if not self.config.robots:
            logger.warning("No robots configured, planning disabled")
            return

        world = create_world(
            backend=self.config.world_backend,
            visualization=self.config.visualization,
        )
        planning_specs = create_planning_specs(
            world=world,
            world_backend=self.config.world_backend,
            planner_name=self.config.planner_name,
            kinematics_name=self.config.kinematics_name,
            kinematics=self.config.kinematics,
        )
        self._world_monitor = planning_specs.world_monitor
        self._planner = planning_specs.planner
        self._kinematics = planning_specs.kinematics
        visualization = create_manipulation_visualization(
            self.config.visualization,
            world=world,
            world_monitor=self._world_monitor,
            manipulation_module=self,
        )

        for robot_config in self.config.robots:
            robot_id = self._world_monitor.add_robot(robot_config)
            traj_gen = JointTrajectoryGenerator(
                num_joints=len(robot_config.joint_names),
                max_velocity=robot_config.max_velocity,
                max_acceleration=robot_config.max_acceleration,
            )
            self._robots[robot_config.name] = (robot_id, robot_config, traj_gen)

        self._world_monitor.finalize()

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
            self._world_monitor.add_obstacle(floor_obs)
            logger.info(f"Floor obstacle added at z={fz:.3f}")

        for _, (robot_id, _, _) in self._robots.items():
            self._world_monitor.start_state_monitor(robot_id)

        self._world_monitor.set_visualization(visualization)
        operator = ManipulationOperator(self, self._world_monitor)
        self._world_monitor.initialize_visualization(operator=operator)

        if self._world_monitor.visualization is not None:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info(f"Visualization: {url}")

        # Start TF publishing thread if any robot has tf_extra_links
        if any(c.tf_extra_links for _, c, _ in self._robots.values()):
            logger.info(f"Eager-initializing TF: {self.tf}")
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
            if self._world_monitor is None:
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
                self._world_monitor.on_joint_state(sub_msg, robot_id=robot_id)

                # Capture per-robot init joints on first update
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
        """Cancel current motion or invalidate an in-progress plan."""
        self._ensure_execution_transaction_state()
        # Publish the cancellation gate before waiting for dispatch. An
        # execution that arrives while cancellation is in flight must not
        # become a new transaction behind the cancellation barrier.
        with self._lock:
            self._cancellation_in_progress = True
        # Holding this lock waits for an in-flight execute RPC to resolve and
        # prevents another arm from being dispatched after cancellation.
        try:
            with self._execution_dispatch_lock:
                with self._lock:
                    task_names = set(self._possibly_active_tasks)
                    is_planning = self._state == ManipulationState.PLANNING
                    is_executing = self._state == ManipulationState.EXECUTING
                    if not (is_planning or is_executing or task_names):
                        return False
                    self._execution_generation += 1
                    if is_planning:
                        self._planning_epoch += 1
                    plan = self._last_plan

                unresolved = self._cancel_tasks(task_names)
                with self._lock:
                    if unresolved:
                        self._possibly_active_tasks = unresolved
                        self._state = ManipulationState.FAULT
                        self._error_message = "Failed to cancel coordinator tasks: " + ", ".join(
                            sorted(unresolved)
                        )
                        logger.error(self._error_message)
                        return False
                    self._possibly_active_tasks.clear()
                    self._state = ManipulationState.IDLE
        finally:
            with self._lock:
                self._cancellation_in_progress = False
        if plan is not None:
            self._dismiss_preview(plan.group_ids)
        logger.info("Motion cancelled")
        return True

    @rpc
    @skill
    def reset(self) -> SkillResult[ManipulationSkillError]:
        """Reset the robot module to IDLE state, clearing any fault.

        Use this after an error or fault to allow new commands.
        Cannot reset while a motion is executing — cancel first.

        TODO: Planning failures should not enter FAULT in the future; execution
        failures may still require reset because the physical state is uncertain.
        """
        with self._lock:
            if self._state == ManipulationState.EXECUTING:
                return SkillResult.fail(
                    "INVALID_STATE",
                    "Cannot reset while executing — cancel the motion first",
                )
            if self._state == ManipulationState.PLANNING:
                self._planning_epoch += 1
            self._state = ManipulationState.IDLE
            self._error_message = ""
        return SkillResult.ok("Reset to IDLE — ready for new commands")

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
            try:
                return self._world_monitor.get_ee_pose(robot[1], joint_state=None)
            except ValueError as exc:
                logger.warning("End-effector pose unavailable: %s", exc)
                return None
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

    def _begin_planning(
        self, robot_name: RobotName | None = None
    ) -> tuple[RobotName, WorldRobotID] | None:
        """Check state and begin planning. Returns (robot_name, robot_id) or None.

        Args:
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._world_monitor is None:
            self._record_error("Planning not initialized")
            return None
        if (robot := self._get_robot(robot_name)) is None:
            self._record_error("Robot not found or robot_name is required")
            return None
        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                self._record_error(f"Cannot plan while state is {self._state.name}")
                return None
            self._planning_epoch += 1
            self._last_plan = None
            self._state = ManipulationState.PLANNING
        return robot[0], robot[1]

    def _begin_group_planning(self) -> int | None:
        """Check state and begin planning for explicit planning-group APIs."""
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return None
        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot plan: state is {self._state.name}")
                return None
            self._planning_epoch += 1
            self._last_plan = None
            self._state = ManipulationState.PLANNING
            return self._planning_epoch

    def _require_unique_pose_group_id_for_robot(self, robot_name: RobotName) -> PlanningGroupID:
        """Return the unique pose-targetable group or raise if it is ambiguous."""
        if self._world_monitor is None:
            raise ValueError("Planning not initialized")
        group_id = self._world_monitor.planning_groups.primary_pose_group_id_for_robot(robot_name)
        if group_id is None:
            raise ValueError(
                f"Robot '{robot_name}' has no pose-targetable planning group; "
                "use an explicit planning group ID"
            )
        return group_id

    def _split_plan_trajectory_by_robot(
        self, plan: GeneratedPlan
    ) -> dict[RobotName, JointTrajectory] | None:
        """Split a stored global selected-joint trajectory into robot-local subsets."""
        trajectories: dict[RobotName, JointTrajectory] = {}
        robot_indices: dict[RobotName, list[tuple[int, str]]] = {}
        for index, global_name in enumerate(plan.trajectory.joint_names):
            if "/" not in global_name:
                logger.error("Stored trajectory joint '%s' is not globally named", global_name)
                return None
            robot_name, local_name = global_name.split("/", 1)
            robot = self._get_robot(robot_name)
            if robot is None:
                return None
            _, _, config, _ = robot
            if local_name not in config.joint_names:
                logger.error("Stored trajectory joint '%s' is not configured", global_name)
                return None
            robot_indices.setdefault(robot_name, []).append((index, local_name))

        for robot_name, indexed_names in robot_indices.items():
            indices = [index for index, _ in indexed_names]
            local_names = [name for _, name in indexed_names]
            points = [
                TrajectoryPoint(
                    time_from_start=point.time_from_start,
                    positions=[point.positions[index] for index in indices],
                    velocities=[point.velocities[index] for index in indices],
                )
                for point in plan.trajectory.points
            ]
            trajectories[robot_name] = JointTrajectory(
                joint_names=local_names,
                points=points,
                timestamp=plan.trajectory.timestamp,
            )
            logger.info(
                "Stored trajectory for %s: %d selected joints, %.3fs",
                robot_name,
                len(local_names),
                trajectories[robot_name].duration,
            )
        return trajectories

    @staticmethod
    def _assert_finite_sequence(values: Sequence[float], label: str) -> None:
        for value in values:
            if not math.isfinite(value):
                raise ValueError(f"{label} contains non-finite value")

    def _limits_for_global_joints(
        self, joint_names: Sequence[str]
    ) -> tuple[list[float], list[float]]:
        velocities: list[float] = []
        accelerations: list[float] = []
        for global_name in joint_names:
            if "/" not in global_name:
                raise ValueError(f"Joint '{global_name}' is not globally named")
            robot_name, local_name = global_name.split("/", 1)
            robot = self._get_robot(robot_name)
            if robot is None:
                raise ValueError(f"Unknown robot for joint '{global_name}'")
            _, _, config, _ = robot
            if local_name not in config.joint_names:
                raise ValueError(f"Unknown local joint '{global_name}'")
            velocity = float(config.max_velocity)
            acceleration = float(config.max_acceleration)
            if not math.isfinite(velocity) or velocity <= 0.0:
                raise ValueError(f"Invalid velocity limit for '{global_name}'")
            if not math.isfinite(acceleration) or acceleration <= 0.0:
                raise ValueError(f"Invalid acceleration limit for '{global_name}'")
            velocities.append(velocity)
            accelerations.append(acceleration)
        return velocities, accelerations

    def _validate_selected_path(
        self, path: Sequence[JointState], expected_names: Sequence[str]
    ) -> list[list[float]]:
        if len(path) < 2:
            raise ValueError("Planner returned fewer than two waypoints")
        expected = list(expected_names)
        waypoints: list[list[float]] = []
        for waypoint_index, state in enumerate(path):
            if list(state.name) != expected:
                raise ValueError(
                    f"Waypoint {waypoint_index} joint names do not match selected order"
                )
            positions = list(state.position)
            if len(positions) != len(expected):
                raise ValueError(f"Waypoint {waypoint_index} position dimension mismatch")
            self._assert_finite_sequence(positions, f"Waypoint {waypoint_index} positions")
            waypoints.append(positions)
        return waypoints

    def _validate_generated_trajectory(
        self,
        trajectory: JointTrajectory,
        expected_names: Sequence[str],
        waypoints: Sequence[Sequence[float]],
    ) -> None:
        expected = list(expected_names)
        if list(trajectory.joint_names) != expected:
            raise ValueError("Generated trajectory joint names do not match selected order")
        if not trajectory.points:
            raise ValueError("Generated trajectory has no points")
        previous_time: float | None = None
        for point_index, point in enumerate(trajectory.points):
            if len(point.positions) != len(expected) or len(point.velocities) != len(expected):
                raise ValueError(f"Generated point {point_index} dimension mismatch")
            self._assert_finite_sequence(
                point.positions, f"Generated point {point_index} positions"
            )
            self._assert_finite_sequence(
                point.velocities, f"Generated point {point_index} velocities"
            )
            if not math.isfinite(point.time_from_start):
                raise ValueError(f"Generated point {point_index} time is non-finite")
            if point_index == 0 and point.time_from_start != 0.0:
                raise ValueError("Generated trajectory must start at time 0")
            if previous_time is not None and point.time_from_start <= previous_time:
                raise ValueError("Generated trajectory times must be strictly increasing")
            previous_time = point.time_from_start
        non_noop = any(list(waypoint) != list(waypoints[0]) for waypoint in waypoints[1:])
        if non_noop and trajectory.duration <= 0.0:
            raise ValueError("Generated trajectory duration must be positive")
        waypoint_index = 0
        for point in trajectory.points:
            if list(point.positions) == list(waypoints[waypoint_index]):
                waypoint_index += 1
                if waypoint_index == len(waypoints):
                    break
        if waypoint_index != len(waypoints):
            raise ValueError("Generated trajectory does not contain ordered waypoint boundaries")

    def _materialize_generated_plan(
        self, group_ids: tuple[PlanningGroupID, ...], result_path: Sequence[JointState]
    ) -> tuple[list[JointState], JointTrajectory]:
        assert self._world_monitor is not None
        selection = self._world_monitor.planning_groups.select(group_ids)
        expected_names = list(selection.joint_names)
        path = [JointState(state) for state in result_path]
        waypoints = self._validate_selected_path(path, expected_names)
        velocities, accelerations = self._limits_for_global_joints(expected_names)
        generator = JointTrajectoryGenerator(
            num_joints=len(expected_names),
            max_velocity=velocities,
            max_acceleration=accelerations,
        )
        generated = generator.generate(waypoints)
        trajectory = JointTrajectory(
            joint_names=expected_names,
            points=generated.points,
            timestamp=generated.timestamp,
        )
        self._validate_generated_trajectory(trajectory, expected_names, waypoints)
        return path, trajectory

    def _plan_selected_path(
        self,
        group_ids: tuple[PlanningGroupID, ...],
        start: JointState,
        goal: JointState,
        planning_epoch: int,
    ) -> GeneratedPlan | None:
        """Plan over explicit planning groups and store the resulting plan."""
        assert self._world_monitor and self._planner
        result = self._planner.plan_selected_joint_path(
            world=self._world_monitor.world,
            selection=self._world_monitor.planning_groups.select(group_ids),
            start=start,
            goal=goal,
            timeout=self.config.planning_timeout,
        )
        if not result.is_success():
            detail = f": {result.message}" if result.message else ""
            self._fail_planning_epoch(
                planning_epoch, f"Planning failed: {result.status.name}{detail}"
            )
            return None

        logger.info("Path: %d waypoints, groups=%s", len(result.path), group_ids)
        try:
            path, trajectory = self._materialize_generated_plan(group_ids, result.path)
        except Exception as exc:
            self._fail_planning_epoch(planning_epoch, f"Failed to materialize plan: {exc}")
            return None
        plan = GeneratedPlan(
            group_ids=group_ids,
            trajectory=trajectory,
            path=path,
            status=result.status,
            planning_time=result.planning_time,
            path_length=result.path_length,
            iterations=result.iterations,
            message=result.message,
        )
        with self._lock:
            if self._state != ManipulationState.PLANNING or planning_epoch != self._planning_epoch:
                logger.info("Discarding cancelled planning result")
                return None
            self._last_plan = plan
            self._state = ManipulationState.COMPLETED
        return plan

    def _record_error(self, message: str) -> bool:
        """Record an error without changing the manipulation state."""
        logger.warning(message)
        self._error_message = message
        return False

    def _fail(self, msg: str) -> bool:
        """Set FAULT state with error message."""
        self._record_error(msg)
        with self._lock:
            self._state = ManipulationState.FAULT
        return False

    def _fail_planning_epoch(self, planning_epoch: int, msg: str) -> bool:
        """Fault only the still-current planning operation."""
        with self._lock:
            if self._state != ManipulationState.PLANNING or planning_epoch != self._planning_epoch:
                logger.info("Discarding cancelled planning result")
                return False
            logger.warning(msg)
            self._last_plan = None
            self._state = ManipulationState.FAULT
            self._error_message = msg
            return False

    def _dismiss_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        """Hide the preview ghost if the world supports it."""
        if self._world_monitor is None:
            return
        try:
            robot_names = self._world_monitor.planning_groups.select(tuple(group_ids)).robot_names
            robot_ids = tuple(
                robot_id
                for robot_name in robot_names
                if (robot_id := self.robot_id_for_name(robot_name)) is not None
            )
        except (KeyError, ValueError):
            robot_ids = ()
        if robot_ids:
            self._world_monitor.cancel_preview_animation(robot_ids=robot_ids)
        else:
            self._world_monitor.cancel_preview_animation()

    def _solve_ik_for_pose(
        self,
        robot_id: WorldRobotID,
        pose: Pose,
        seed: JointState,
        check_collision: bool,
    ) -> IKResult:
        """Run the configured kinematics backend for a world-frame pose."""
        assert self._world_monitor and self._kinematics

        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )

        return self._kinematics.solve(
            world=self._world_monitor.world,
            robot_id=robot_id,
            target_pose=target_pose,
            seed=seed,
            check_collision=check_collision,
        )

    @rpc
    def inverse_kinematics(
        self,
        pose_targets: Mapping[PlanningGroupID, PoseStamped],
        auxiliary_group_ids: Sequence[PlanningGroupID] = (),
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> IKResult:
        """Solve planning-group pose targets without planning a joint path."""
        if self._kinematics is None or self._world_monitor is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
        if not pose_targets:
            return IKResult(
                status=IKStatus.NO_SOLUTION, message="At least one pose target is required"
            )

        try:
            stamped_targets = dict(pose_targets)
            auxiliary_ids = tuple(auxiliary_group_ids)
            group_ids = tuple(dict.fromkeys((*stamped_targets.keys(), *auxiliary_ids)))
            target_groups = {
                self._world_monitor.planning_groups.get(group_id): pose
                for group_id, pose in stamped_targets.items()
            }
            auxiliary_groups = tuple(
                self._world_monitor.planning_groups.get(group_id) for group_id in auxiliary_ids
            )
            seed_state = seed
            if seed_state is None:
                selection = self._world_monitor.planning_groups.select(group_ids)
                current = self._world_monitor.current_global_joint_state()
                if not current.name and not current.position:
                    return IKResult(status=IKStatus.NO_SOLUTION, message="No joint state")
                seed_state = filter_joint_state_to_selected_joints(current, selection.joint_names)
        except (KeyError, ValueError) as exc:
            return IKResult(status=IKStatus.NO_SOLUTION, message=str(exc))
        if seed_state is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="No joint state")
        return self._kinematics.solve_pose_targets(
            world=self._world_monitor.world,
            pose_targets=target_groups,
            auxiliary_groups=auxiliary_groups,
            seed=seed_state,
            check_collision=check_collision,
        )

    @rpc
    def inverse_kinematics_single(
        self,
        pose: Pose,
        robot_name: RobotName | None = None,
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> IKResult:
        """Solve IK for one robot's unique pose-targetable planning group."""
        if self._world_monitor is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
        robot = self._get_robot(robot_name)
        if robot is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Robot not found")
        selected_robot_name, _, _, _ = robot
        try:
            group_id = self._require_unique_pose_group_id_for_robot(selected_robot_name)
        except ValueError as exc:
            return IKResult(status=IKStatus.NO_SOLUTION, message=str(exc))
        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )
        return self.inverse_kinematics(
            {group_id: target_pose}, seed=seed, check_collision=check_collision
        )

    @rpc
    def solve_ik(
        self,
        pose: Pose,
        robot_name: RobotName | None = None,
        check_collision: bool = True,
        seed: JointState | None = None,
    ) -> IKResult:
        """Solve IK for a pose without planning a joint path.

        Args:
            pose: Target end-effector pose
            robot_name: Robot to solve for (required if multiple robots configured)
            check_collision: Whether to reject IK candidates in collision
            seed: Optional joint state to initialize local IK. Uses current state when omitted.
        """
        if self._kinematics is None or self._world_monitor is None:
            self._record_error("Planning not initialized")
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
        robot = self._get_robot(robot_name)
        if robot is None:
            self._record_error("Robot not found or robot_name is required")
            return IKResult(status=IKStatus.NO_SOLUTION, message="Robot not found")

        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                self._record_error(f"Cannot solve IK while state is {self._state.name}")
                return IKResult(
                    status=IKStatus.NO_SOLUTION,
                    message=f"Cannot solve IK while state is {self._state.name}",
                )
            self._state = ManipulationState.PLANNING
        result = self.inverse_kinematics_single(
            pose,
            robot_name=robot_name,
            seed=seed,
            check_collision=check_collision,
        )
        self._state = ManipulationState.COMPLETED if result.is_success() else ManipulationState.IDLE
        if result.is_success():
            logger.info(f"IK solved, error: {result.position_error:.4f}m")
        else:
            detail = f": {result.message}" if result.message else ""
            self._record_error(f"IK failed: {result.status.name}{detail}")
        return result

    @rpc
    def plan_to_pose(self, pose: Pose, robot_name: RobotName | None = None) -> bool:
        """Plan motion to pose. Use preview_plan() then execute().

        Args:
            pose: Target end-effector pose
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._kinematics is None or self._world_monitor is None:
            self._record_error("Planning not initialized")
            return False
        robot = self._get_robot(robot_name)
        if robot is None:
            self._record_error("Robot not found or robot_name is required")
            return False
        selected_robot_name, _, _, _ = robot
        try:
            group_id = self._require_unique_pose_group_id_for_robot(selected_robot_name)
        except ValueError as exc:
            logger.warning("Pose planning unavailable: %s", exc)
            self._record_error(str(exc))
            return False
        return self.plan_to_pose_targets({group_id: pose})

    @rpc
    def plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
    ) -> bool:
        """Plan to one or more group pose targets with optional auxiliary groups."""
        return self.generate_plan_to_pose_targets(pose_targets, auxiliary_groups) is not None

    @rpc
    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        """Plan motion to joint config. Use preview_plan() then execute().

        Args:
            joints: Target joint state (names + positions)
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        selected_robot_name, _, _, _ = robot
        logger.info(
            f"Planning to joints for {selected_robot_name}: {[f'{j:.3f}' for j in joints.position]}"
        )
        if self._world_monitor is None:
            self._record_error("Planning not initialized")
            return False
        group_id = self._world_monitor.planning_groups.default_group_id_for_robot(
            selected_robot_name
        )
        if group_id is None:
            logger.error(
                "Robot '%s' has no unique default planning group; use explicit group APIs",
                selected_robot_name,
            )
            return False
        return self.plan_to_joint_targets({group_id: joints})

    @rpc
    def plan_to_joint_targets(
        self, joint_targets: Mapping[PlanningGroupID | PlanningGroup, JointState]
    ) -> bool:
        """Plan to joint targets keyed by planning group."""
        return self.generate_plan_to_joint_targets(joint_targets) is not None

    def generate_plan_to_joint_targets(
        self, joint_targets: Mapping[PlanningGroupID | PlanningGroup, JointState]
    ) -> GeneratedPlan | None:
        """Plan to joint targets and return the exact stored GeneratedPlan."""
        if self._world_monitor is None or self._planner is None:
            return None
        if not joint_targets:
            self._fail("At least one joint target is required")
            return None

        group_ids = tuple(
            dict.fromkeys(planning_group_id_from_selector(group) for group in joint_targets)
        )
        planning_epoch = self._begin_group_planning()
        if planning_epoch is None:
            return None

        try:
            selection = self._world_monitor.planning_groups.select(group_ids)
            current = self._world_monitor.current_global_joint_state()
            start = filter_joint_state_to_selected_joints(current, selection.joint_names)
        except Exception as exc:
            self._fail_planning_epoch(planning_epoch, f"Failed to resolve planning groups: {exc}")
            return None

        goal_names: list[str] = []
        goal_positions: list[float] = []
        for group, target in joint_targets.items():
            group_id = planning_group_id_from_selector(group)
            try:
                target_group = self._world_monitor.planning_groups.get(group_id)
                target_global = joint_target_to_global_names(target_group, target)
            except (KeyError, ValueError) as exc:
                logger.error(str(exc))
                self._fail_planning_epoch(planning_epoch, f"Invalid joint target for '{group_id}'")
                return None
            goal_names.extend(target_global.name)
            goal_positions.extend(target_global.position)

        goal = JointState(name=goal_names, position=goal_positions)
        return self._plan_selected_path(group_ids, start, goal, planning_epoch)

    def generate_plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
    ) -> GeneratedPlan | None:
        """Plan to pose targets and return the exact stored GeneratedPlan."""
        if self._world_monitor is None or self._kinematics is None:
            return None
        if not pose_targets:
            self._fail("At least one pose target is required")
            return None
        stamped_targets = {
            planning_group_id_from_selector(group): PoseStamped(
                frame_id="world",
                position=pose.position,
                orientation=pose.orientation,
            )
            for group, pose in pose_targets.items()
        }
        auxiliary_ids = tuple(planning_group_id_from_selector(group) for group in auxiliary_groups)
        group_ids = tuple(dict.fromkeys((*stamped_targets.keys(), *auxiliary_ids)))
        planning_epoch = self._begin_group_planning()
        if planning_epoch is None:
            return None
        try:
            selection = self._world_monitor.planning_groups.select(group_ids)
            current = self._world_monitor.current_global_joint_state()
            start = filter_joint_state_to_selected_joints(current, selection.joint_names)
        except Exception as exc:
            self._fail_planning_epoch(planning_epoch, f"Failed to resolve planning groups: {exc}")
            return None
        ik = self.inverse_kinematics(
            pose_targets=stamped_targets,
            auxiliary_group_ids=auxiliary_ids,
            seed=start,
        )
        if not ik.is_success() or ik.joint_state is None:
            detail = f": {ik.message}" if ik.message else ""
            self._fail_planning_epoch(planning_epoch, f"IK failed: {ik.status.name}{detail}")
            return None
        logger.info(f"IK solved, error: {ik.position_error:.4f}m")
        return self._plan_selected_path(group_ids, start, ik.joint_state, planning_epoch)

    @rpc
    def preview_path(
        self,
        duration: float | None = None,
        robot_name: RobotName | None = None,
        target_fps: float = 30.0,
    ) -> bool:
        """Compatibility wrapper for preview_plan().

        Args:
            duration: Total animation duration in seconds. Defaults to one second.
            robot_name: Compatibility affected-robot validation; does not filter the preview.
            target_fps: Deprecated compatibility argument; shared-clock previews use plan waypoints.
        """
        return self.preview_plan(None, duration, robot_name, target_fps)

    @rpc
    def preview_plan(
        self,
        plan: GeneratedPlan | None = None,
        duration: float | None = None,
        robot_name: RobotName | None = None,
        target_fps: float = 30.0,
    ) -> bool:
        """Preview a complete generated plan in the visualizer."""
        plan = plan or self._last_plan
        if plan is None or not plan.path:
            logger.warning("No generated plan to preview")
            return False
        try:
            assert self._world_monitor is not None
            affected = self._world_monitor.planning_groups.select(plan.group_ids).robot_names
        except Exception as exc:
            logger.error("Generated plan cannot be resolved: %s", exc)
            return False
        if robot_name is not None:
            if robot_name not in affected:
                logger.error("Generated plan does not affect robot '%s'", robot_name)
                return False
        if self._world_monitor is None:
            return False
        self._world_monitor.animate_trajectory(plan.trajectory, duration)
        return True

    @rpc
    def has_planned_path(self) -> bool:
        """Check if there's a planned path ready.

        Returns:
            True if a path is planned and ready
        """
        return self._last_plan is not None and bool(self._last_plan.path)

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
        with self._lock:
            plan = self._last_plan
            self._last_plan = None
            if self._state == ManipulationState.PLANNING:
                self._planning_epoch += 1
                self._state = ManipulationState.IDLE
        if plan is not None:
            # Preserve the group selection until the public visualization
            # transaction has invalidated and hidden its preview.
            self._dismiss_preview(plan.group_ids)
        return True

    @rpc
    def list_robots(self) -> list[str]:
        """List all configured robot names.

        Returns:
            List of robot names
        """
        return list(self._robots.keys())

    @rpc
    def list_planning_groups(self) -> list[PlanningGroup]:
        """Return all configured planning groups."""
        if self._world_monitor is None:
            return []
        return list(self._world_monitor.planning_groups.list())

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        """Return the named robot's current local joint state with names."""
        if self._world_monitor is None:
            return None
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return self._world_monitor.get_current_joint_state(robot_id)

    @rpc
    def get_robot_info(self, robot_name: RobotName | None = None) -> RobotInfoPayload | None:
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
        planning_groups = (
            list(self._world_monitor.planning_groups.groups_for_robot(robot_name))
            if self._world_monitor is not None
            else []
        )
        try:
            end_effector_link = config.end_effector_link
        except ValueError:
            end_effector_link = None

        return {
            "name": config.name,
            "world_robot_id": robot_id,
            "joint_names": config.joint_names,
            "planning_groups": planning_groups,
            "end_effector_link": end_effector_link,
            "base_link": config.base_link,
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

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        """Return configured robots for in-process visualization adapters."""
        return [(name, robot_id, config) for name, (robot_id, config, _) in self._robots.items()]

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        """Return the planning-world robot id for a configured robot name."""
        entry = self._robots.get(robot_name)
        return entry[0] if entry is not None else None

    def robot_name_for_id(self, robot_id: WorldRobotID) -> RobotName | None:
        """Return the configured robot name for a planning-world robot id."""
        for robot_name, (candidate_id, _, _) in self._robots.items():
            if candidate_id == robot_id:
                return robot_name
        return None

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        """Return the robot model config for an in-process visualization adapter."""
        entry = self._robots.get(robot_name)
        return entry[1] if entry is not None else None

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

    def evaluate_joint_target(
        self, joints: JointState | None, robot_name: RobotName
    ) -> TargetEvaluation:
        """Evaluate a joint target for visualization without planning a path."""
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None or self._world_monitor is None:
            return {
                "success": False,
                "status": "NO_ROBOT",
                "message": f"Unknown robot: {robot_name}",
                "collision_free": False,
                "ee_pose": None,
                "joint_state": None,
            }
        if joints is None:
            return {
                "success": False,
                "status": "NO_TARGET",
                "message": "No joint target provided",
                "collision_free": False,
                "ee_pose": None,
                "joint_state": None,
            }
        target = JointState(joints)
        collision_free = self._world_monitor.is_state_valid(robot_id, target)
        return {
            "success": True,
            "status": "FEASIBLE" if collision_free else "COLLISION",
            "message": "Target is collision-free" if collision_free else "Target is in collision",
            "collision_free": collision_free,
            "ee_pose": self._world_monitor.get_ee_pose(robot_id, target),
            "joint_state": target,
        }

    def evaluate_pose_target(self, pose: Pose, robot_name: RobotName) -> TargetEvaluation:
        """Evaluate a Cartesian target for visualization without planning a path."""
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNKNOWN_ROBOT",
                "message": f"Unknown robot: {robot_name}",
                "collision_free": False,
            }
        if self._world_monitor is None or self._kinematics is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized or current state is unavailable",
                "collision_free": False,
            }
        current = self._world_monitor.get_current_joint_state(robot_id)
        if current is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized or current state is unavailable",
                "collision_free": False,
            }
        ik = self._solve_ik_for_pose(robot_id, pose, current, check_collision=True)
        joint_state = JointState(ik.joint_state) if ik.is_success() and ik.joint_state else None
        collision_free = bool(
            joint_state is not None and self._world_monitor.is_state_valid(robot_id, joint_state)
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
        if self._world_monitor is None:
            return False
        current = self._world_monitor.get_current_joint_state(robot_id)
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

    def _execute_robot_trajectory(self, robot_name: RobotName, trajectory: JointTrajectory) -> bool:
        """Execute one robot's local trajectory through ControlCoordinator."""
        if (robot := self._get_robot(robot_name)) is None:
            return False
        resolved_name, _, config, _ = robot

        if not config.coordinator_task_name:
            logger.error(f"No coordinator_task_name for '{resolved_name}'")
            return False
        if (client := self._get_coordinator_client()) is None:
            logger.error("No coordinator client")
            return False

        translated = self._translate_trajectory_to_coordinator(trajectory, config)
        logger.info(
            f"Executing: task='{config.coordinator_task_name}', {len(translated.points)} pts, {translated.duration:.2f}s"
        )

        result = client.task_invoke(
            config.coordinator_task_name, "execute", {"trajectory": translated}
        )
        if result:
            logger.info("Trajectory accepted")
            return True
        return self._fail("Coordinator rejected trajectory")

    @rpc
    def execute(self, robot_name: RobotName | None = None) -> bool:
        """Compatibility wrapper for execute_plan()."""
        return self.execute_plan(robot_name=robot_name)

    @rpc
    def execute_plan(
        self, plan: GeneratedPlan | None = None, robot_name: RobotName | None = None
    ) -> bool:
        """Execute a generated planning-group plan through affected trajectory tasks."""
        execution = self._begin_execution(plan)
        if execution is None:
            return False
        previous_state, target_plan, token = execution
        try:
            freshness_error = self._stored_plan_freshness_error(target_plan)
            if freshness_error is not None:
                self._restore_execution_gate(token, previous_state, freshness_error)
                return False
            prepared = self._prepare_execution(target_plan, robot_name)
            if isinstance(prepared, str):
                self._restore_execution_gate(token, previous_state, prepared)
                return False
        except Exception as exc:
            self._restore_execution_gate(
                token, previous_state, f"Failed to prepare execution: {exc}"
            )
            return False
        return self._dispatch_prepared_plan(prepared, token)

    def _begin_execution(
        self, plan: GeneratedPlan | None
    ) -> tuple[ManipulationState, GeneratedPlan, int] | None:
        """Enter EXECUTING if the module is idle enough to dispatch."""
        self._ensure_execution_transaction_state()
        with self._lock:
            if self._cancellation_in_progress:
                logger.warning("Cancellation is in progress; rejecting execution")
                return None
            target_plan = plan or self._last_plan
            if target_plan is None or not target_plan.path or not target_plan.trajectory.points:
                logger.warning("Stored plan is invalid or not executable")
                self._error_message = "Stored plan is invalid or not executable"
                return None
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning("Manipulation state is not executable")
                return None
            previous_state = self._state
            self._execution_generation += 1
            token = self._execution_generation
            self._state = ManipulationState.EXECUTING
            return previous_state, target_plan, token

    def _restore_execution_gate(
        self, token: int, previous_state: ManipulationState, msg: str
    ) -> None:
        logger.warning(msg)
        with self._lock:
            if self._execution_generation == token and self._state == ManipulationState.EXECUTING:
                self._state = previous_state
                self._error_message = msg

    def _stored_plan_freshness_error(
        self, plan: GeneratedPlan, tolerance: float = 1e-6
    ) -> str | None:
        """Return why current selected joints no longer match the plan start."""
        if self._world_monitor is None:
            return "Planning not initialized"
        if not plan.trajectory.points:
            return "Stored trajectory has no points"
        first = plan.trajectory.points[0]
        names = list(plan.trajectory.joint_names)
        if len(names) != len(first.positions):
            return "Stored trajectory start has malformed joint positions"
        try:
            current = self._world_monitor.current_global_joint_state()
        except (AttributeError, TypeError, ValueError):
            return "Current planned joints are malformed"
        if not isinstance(current, JointState) or len(current.name) != len(current.position):
            return "Current planned joints are malformed"
        values: dict[str, float] = {}
        ordered_subset: list[tuple[str, float]] = []
        planned = set(names)
        for raw_name, raw_position in zip(current.name, current.position, strict=True):
            name = str(raw_name)
            if name in values:
                return "Current planned joints are malformed"
            try:
                value = float(raw_position)
            except (TypeError, ValueError):
                return f"Current planned joint '{name}' is malformed"
            if not math.isfinite(value):
                return f"Current planned joint '{name}' is malformed"
            values[name] = value
            if name in planned:
                ordered_subset.append((name, value))
        if len(ordered_subset) != len(names):
            return "Current planned joints are malformed"
        for (actual_name, actual), name, expected in zip(
            ordered_subset, names, first.positions, strict=True
        ):
            if actual_name != name:
                return "Current planned joints are not in stored plan order"
            if not math.isfinite(actual) or not math.isfinite(float(expected)):
                return f"Current planned joint '{name}' is malformed"
            if abs(actual - float(expected)) > tolerance:
                return f"Current planned joint '{name}' no longer matches the stored plan start"
        return None

    def _prepare_execution(
        self,
        plan: GeneratedPlan,
        robot_name: RobotName | None = None,
    ) -> tuple[tuple[RobotName, RobotModelConfig, JointTrajectory], ...] | str:
        """Resolve execution inputs before dispatching any coordinator task."""
        if self._world_monitor is None:
            return "Planning not initialized"
        if not plan.path:
            return "No generated plan"
        try:
            affected = list(self._world_monitor.planning_groups.select(plan.group_ids).robot_names)
        except Exception as exc:
            return f"Failed to resolve generated plan: {exc}"
        if robot_name is not None:
            if robot_name not in affected:
                return f"No planned path for '{robot_name}'"
            affected = [robot_name]
        trajectories = self._split_plan_trajectory_by_robot(plan)
        if trajectories is None:
            return "Failed to split stored trajectory"
        if self._get_coordinator_client() is None:
            return "No coordinator client"
        prepared: list[tuple[RobotName, RobotModelConfig, JointTrajectory]] = []
        for name in affected:
            robot = self._get_robot(name)
            if robot is None:
                return f"Robot '{name}' not found"
            _, _, config, _ = robot
            if not config.coordinator_task_name:
                return f"No coordinator_task_name for '{name}'"
            traj = trajectories.get(name)
            if traj is None:
                return f"No planned trajectory for '{name}'"
            prepared.append((name, config, traj))
        return tuple(prepared)

    def _dispatch_prepared_plan(
        self,
        prepared: Sequence[tuple[RobotName, RobotModelConfig, JointTrajectory]],
        token: int,
    ) -> bool:
        """Dispatch already-validated robot trajectories; failures here fault."""
        self._ensure_execution_transaction_state()
        with self._execution_dispatch_lock:
            if (client := self._get_coordinator_client()) is None:
                return self._fault_execution(token, "No coordinator client")

            for _robot_name, config, traj in prepared:
                task_name = config.coordinator_task_name
                if task_name is None:
                    return self._fault_execution(token, "No coordinator_task_name")
                with self._lock:
                    if (
                        self._execution_generation != token
                        or self._state != ManipulationState.EXECUTING
                    ):
                        return False
                    # Add before the RPC: a rejected or raised execute may
                    # have been accepted remotely.
                    self._possibly_active_tasks.add(task_name)
                try:
                    translated = self._translate_trajectory_to_coordinator(traj, config)
                    logger.info(
                        "Executing: task='%s', %d pts, %.2fs",
                        task_name,
                        len(translated.points),
                        translated.duration,
                    )
                    result = client.task_invoke(task_name, "execute", {"trajectory": translated})
                except Exception as exc:
                    return self._fault_execution(token, f"Failed to dispatch trajectory: {exc}")
                if not result:
                    return self._fault_execution(token, "Coordinator rejected trajectory")

            logger.info("Trajectory accepted")
            with self._lock:
                if (
                    self._execution_generation != token
                    or self._state != ManipulationState.EXECUTING
                ):
                    return False
                self._state = ManipulationState.COMPLETED
            return True

    def _cancel_tasks(self, task_names: set[str]) -> set[str]:
        """Cancel possibly active tasks, returning names whose safety is unknown."""
        if not task_names:
            return set()
        client = self._get_coordinator_client()
        if client is None:
            return set(task_names)
        unresolved: set[str] = set()
        for task_name in task_names:
            try:
                result = client.task_invoke(task_name, "cancel", {})
                # Coordinator cancel returns False when the task is already
                # non-executing; that is a confirmed-safe result.
                if result is None:
                    unresolved.add(task_name)
            except Exception:
                unresolved.add(task_name)
        return unresolved

    def _fault_execution(self, token: int, message: str) -> bool:
        """Rollback attempted tasks and fault, without overwriting cancellation."""
        self._ensure_execution_transaction_state()
        with self._lock:
            if self._execution_generation != token:
                return False
            task_names = set(self._possibly_active_tasks)
        unresolved = self._cancel_tasks(task_names)
        with self._lock:
            if self._execution_generation != token:
                return False
            if unresolved:
                self._possibly_active_tasks = unresolved
                message += "; unresolved coordinator tasks: " + ", ".join(sorted(unresolved))
            else:
                self._possibly_active_tasks.clear()
            self._state = ManipulationState.FAULT
            self._error_message = message
        logger.error(message)
        return False

    @rpc
    def get_trajectory_status(self, robot_name: RobotName | None = None) -> dict[str, Any] | None:
        """Get trajectory execution status via coordinator task_invoke."""
        last_plan = self._last_plan
        if robot_name is None and last_plan is not None and last_plan.path:
            if self._world_monitor is None:
                return None
            statuses = {
                name: self.get_trajectory_status(name)
                for name in self._world_monitor.planning_groups.select(
                    last_plan.group_ids
                ).robot_names
            }
            return {"robots": statuses}
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
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

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
        last_plan = self._last_plan
        if robot_name is None and last_plan is not None and last_plan.path:
            try:
                assert self._world_monitor is not None
                affected = self._world_monitor.planning_groups.select(
                    last_plan.group_ids
                ).robot_names
            except Exception as exc:
                logger.warning("Failed to resolve generated plan while waiting: %s", exc)
                return False
            return all(
                self._wait_for_trajectory_completion(name, timeout, poll_interval)
                for name in affected
            )

        robot = self._get_robot(robot_name)
        if robot is None:
            return True
        _, _, config, _ = robot
        client = self._get_coordinator_client()
        trajectory = last_plan.trajectory if last_plan is not None else None

        if client is None or not config.coordinator_task_name:
            # No coordinator — wait for trajectory duration as fallback
            if trajectory is not None:
                logger.info(
                    f"No coordinator status — waiting {trajectory.duration:.1f}s for trajectory"
                )
                time.sleep(trajectory.duration + 0.5)
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
                if trajectory is not None:
                    remaining = trajectory.duration - (time.time() - start)
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
        return self._preview_execute_wait(robot_name)

    def _preview_execute_wait(
        self, robot_name: RobotName | None = None, preview_duration: float = 0.5
    ) -> SkillResult[ManipulationSkillError]:
        """Preview planned path, execute, and wait for completion.

        Args:
            robot_name: Robot to operate on
            preview_duration: Duration to animate the preview in Meshcat (seconds)
        """
        logger.info("Previewing trajectory...")
        self.preview_path(preview_duration, robot_name)

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

        exec_result = self._preview_execute_wait(robot_name)
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

        exec_result = self._preview_execute_wait(robot_name)
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

        exec_result = self._preview_execute_wait(robot_name)
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
        if self._world_monitor is not None:
            init_ee = self._world_monitor.get_ee_pose(robot_id, joint_state=init)
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
                    wp_result = self._preview_execute_wait(robot_name)
                    if not wp_result.is_success():
                        return wp_result
                else:
                    logger.warning("Safe waypoint unreachable, going directly to init")

        logger.info(
            f"Planning motion to init position [{', '.join(f'{j:.3f}' for j in init.position)}]..."
        )
        if not self.plan_to_joints(init, robot_name):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to init position")

        exec_result = self._preview_execute_wait(robot_name)
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

        # Stop world monitor (includes visualization thread)
        if self._world_monitor is not None:
            self._world_monitor.stop_all_monitors()

        super().stop()
