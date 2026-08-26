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

"""Group-native motion planning and execution RPC module."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from enum import Enum
import math
import threading
import time
from typing import Any, Literal, TypeAlias, cast

from pydantic import Field

from dimos.agents.skill_result import SkillResult
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.execution_manager import (
    ExecutionTarget,
    PlanExecutionManager,
)
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    MoveResult,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.factory import (
    KinematicsName,
    WorldBackend,
    create_planning_specs,
    create_world,
)
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
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
from dimos.manipulation.planning.planners.config import (
    CartesianPathConfig,
    ManipulationPlannerConfig,
)
from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig,
    RoboPlanPlannerConfig,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, ObstacleType
from dimos.manipulation.planning.spec.models import (
    DEFAULT_OBSTACLE_RGBA,
    CartesianTarget,
    GeneratedPlan,
    IKResult,
    Obstacle,
    PlanningGroupID,
    PlanningResult,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import (
    KinematicsSpec,
    PlannerSpec,
    TrajectoryParametrizerSpec,
)
from dimos.manipulation.planning.trajectory_generator.config import (
    TrajectoryParametrizationConfig,
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
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Composite type aliases for readability (using semantic IDs from planning.spec)
RobotEntry: TypeAlias = tuple[WorldRobotID, RobotModelConfig]
"""(world_robot_id, config)"""

RobotRegistry: TypeAlias = dict[RobotName, RobotEntry]
"""Maps robot_name -> RobotEntry"""

RobotInfoValue: TypeAlias = (
    str | bool | float | list[str] | list[float] | list[PlanningGroup] | None
)
RobotInfoPayload: TypeAlias = dict[str, RobotInfoValue]

ObstacleShape: TypeAlias = Literal["box", "sphere", "cylinder", "mesh"]

_SHAPE_TO_OBSTACLE_TYPE: dict[str, ObstacleType] = {
    "box": ObstacleType.BOX,
    "sphere": ObstacleType.SPHERE,
    "cylinder": ObstacleType.CYLINDER,
    "mesh": ObstacleType.MESH,
}


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
    world_backend: WorldBackend = "roboplan"
    visualization: ManipulationVisualizationConfig = Field(
        default_factory=NoManipulationVisualizationConfig
    )
    planner: ManipulationPlannerConfig = Field(default_factory=RoboPlanPlannerConfig)
    trajectory_parametrization: TrajectoryParametrizationConfig | None = Field(
        default=None,
        description=(
            "Path parametrizer selected at startup. Omit to use roboplan_toppra "
            "with RoboPlanWorld or simple_trapezoid with DrakeWorld."
        ),
    )
    kinematics: ManipulationKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    # Deprecated: use kinematics.backend instead.
    kinematics_name: KinematicsName | None = None
    # Floor plane Z height (meters). When set, a box obstacle is added at startup
    # to prevent the planner from routing trajectories below this height.
    # Set to None to disable.
    floor_z: float | None = None
    default_speed_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    linear_speed_scale: float = Field(default=0.5, gt=0.0, le=1.0)
    execution_timeout: float = Field(default=60.0, gt=0.0)


class ManipulationModule(Module):
    """Primitive manipulation RPCs; agent skills live in a separate adapter."""

    config: ManipulationModuleConfig
    _control_coordinator: ControlCoordinator

    # Input: Joint state from coordinator (for world sync)
    coordinator_joint_state: In[JointState]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # State machine
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""
        self._planning_epoch = 0
        self._started = False

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner: PlannerSpec | None = None
        self._kinematics: KinematicsSpec | None = None
        self._trajectory_parametrizer: TrajectoryParametrizerSpec | None = None

        # Robot registry: maps robot_name -> (world_robot_id, config)
        self._robots: RobotRegistry = {}

        # Canonical generated plan for plan/preview/execute workflow.
        # Robot-local paths and trajectories are derived from this plan on demand.
        self._last_plan: GeneratedPlan | None = None

        # Coordinator integration (initialized in start())
        self._execution_manager: PlanExecutionManager

        # Init joints: captured from first joint state per robot, used by go_init
        self._init_joints: dict[RobotName, JointState] = {}

        # TF publishing thread
        self._tf_stop_event = threading.Event()
        self._tf_thread: threading.Thread | None = None

        logger.info("ManipulationModule initialized")

    @rpc
    def start(self) -> None:
        """Start the manipulation module."""
        if self._started:
            logger.warning("ManipulationModule already running")
            return
        self._started = True
        try:
            super().start()

            # Execution state must exist before planning starts observers such as visualization.
            self._initialize_execution()
            self._initialize_planning()

            # Subscribe to joint state via port
            if self.coordinator_joint_state is not None:
                self.coordinator_joint_state.subscribe(self._on_joint_state)
                logger.info("Subscribed to coordinator_joint_state port")
            logger.info("ManipulationModule started")
        except BaseException:
            self._started = False
            raise

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
            planner=self.config.planner,
            kinematics_name=self.config.kinematics_name,
            kinematics=self.config.kinematics,
            trajectory_parametrization=self.config.trajectory_parametrization,
        )
        self._world_monitor = planning_specs.world_monitor
        self._planner = planning_specs.planner
        self._kinematics = planning_specs.kinematics
        self._trajectory_parametrizer = planning_specs.trajectory_parametrizer
        visualization = create_manipulation_visualization(
            self.config.visualization,
            world=world,
            world_monitor=self._world_monitor,
            manipulation_module=self,
        )

        for robot_config in self.config.robots:
            robot_id = self._world_monitor.add_robot(robot_config)
            self._robots[robot_config.name] = (robot_id, robot_config)

        operator = ManipulationOperator(self, self._world_monitor)
        self._world_monitor.finalize(visualization, operator=operator)

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

        for robot_id, _ in self._robots.values():
            self._world_monitor.start_state_monitor(robot_id)

        if self._world_monitor.visualization is not None:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info(f"Visualization: {url}")

        # Start TF publishing thread if any robot has tf_extra_links
        if any(c.tf_extra_links for _, c in self._robots.values()):
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
    ) -> tuple[RobotName, WorldRobotID, RobotModelConfig] | None:
        """Get robot by name or default.

        Args:
            robot_name: Robot name or None for default (if single robot)

        Returns:
            (robot_name, robot_id, config) or None if not found
        """
        if not robot_name:  # None or empty string (LLMs often pass "")
            robot_name = self._get_default_robot_name()
            if robot_name is None:
                logger.error("Multiple robots configured, must specify robot_name")
                return None

        if robot_name not in self._robots:
            logger.error(f"Unknown robot: {robot_name}")
            return None

        robot_id, config = self._robots[robot_name]
        return (robot_name, robot_id, config)

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

            for robot_name, (robot_id, config) in self._robots.items():
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
                for robot_id, config in self._robots.values():
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
                    self.tf.publish(TFMessage(*transforms))
            except Exception as e:
                logger.debug(f"TF publish error: {e}")

            self._tf_stop_event.wait(period)

    @rpc
    def get_state(self) -> ManipulationSnapshot:
        """Return one snapshot containing every planning group."""

        groups: dict[PlanningGroupID, PlanningGroupState] = {}
        if self._world_monitor is not None:
            for group in self._world_monitor.planning_groups.list():
                joints: JointState | None = None
                pose: PoseStamped | None = None
                try:
                    joints = self._world_monitor.current_group_joint_state(group.id)
                except (KeyError, ValueError):
                    pass
                if group.has_pose_target:
                    try:
                        pose = self._world_monitor.get_group_ee_pose(group.id)
                    except (KeyError, ValueError):
                        pass
                groups[group.id] = PlanningGroupState(
                    joints=joints,
                    end_effector_pose=pose,
                    gripper_position=self._get_group_gripper_position(group),
                    joint_presets=self._group_joint_presets(group),
                )
        with self._lock:
            operation_status = OperationStatus[self._state.name]
            error = self._error_message or None
            has_pending_plan = self._last_plan is not None
        return ManipulationSnapshot(
            timestamp=time.time(),
            operation_status=operation_status,
            error=error,
            has_pending_plan=has_pending_plan,
            execution_status=self._execution_manager.status,
            groups=groups,
        )

    def get_error(self) -> str:
        """Get last error message.

        Returns:
            Error message or empty string
        """
        return self._error_message

    @rpc
    def cancel(self) -> ExecutionResult:
        """Cancel planning or the active trajectory."""
        with self._lock:
            is_planning = self._state == ManipulationState.PLANNING
            if is_planning:
                self._planning_epoch += 1
            plan = self._last_plan
            self._last_plan = None

        result = self._execution_manager.cancel()
        if plan is not None:
            self._dismiss_preview(plan.group_ids)
        if is_planning and result.status is ExecutionStatus.NO_EXECUTION:
            result = ExecutionResult(ExecutionStatus.ABORTED, "Planning cancelled")
        self._apply_execution_result(result)
        return result

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

    def is_collision_free(self, joints: list[float], robot_name: RobotName | None = None) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check
            robot_name: Robot to check (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            _, robot_id, config = robot
            joint_state = JointState(name=config.joint_names, position=joints)
            return self._world_monitor.is_state_valid(robot_id, joint_state)
        return False

    def _begin_group_planning(self, speed_scale: float | None = None) -> tuple[int, float] | None:
        """Check state and begin planning for explicit planning-group APIs."""
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return None
        resolved_speed = self.config.default_speed_scale if speed_scale is None else speed_scale
        if not math.isfinite(resolved_speed) or not 0.0 < resolved_speed <= 1.0:
            self._record_error("speed_scale must be finite, > 0, and <= 1")
            return None
        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot plan: state is {self._state.name}")
                return None
            self._planning_epoch += 1
            self._last_plan = None
            self._error_message = ""
            self._state = ManipulationState.PLANNING
            return self._planning_epoch, float(resolved_speed)

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

    def _resolve_group_plan_start(
        self,
        group_ids: tuple[PlanningGroupID, ...],
        planning_epoch: int,
    ) -> tuple[PlanningGroupSelection, JointState] | None:
        """Resolve an ordered group selection and its authoritative start state."""
        assert self._world_monitor is not None
        try:
            selection = self._world_monitor.planning_groups.select(group_ids)
            current = self._world_monitor.current_global_joint_state()
            start = filter_joint_state_to_selected_joints(current, selection.joint_names)
        except Exception as exc:
            self._fail_planning_epoch(planning_epoch, f"Failed to resolve planning groups: {exc}")
            return None
        return selection, start

    def _store_generated_plan(
        self,
        group_ids: tuple[PlanningGroupID, ...],
        result: PlanningResult,
        planning_epoch: int,
        speed_scale: float,
    ) -> GeneratedPlan | None:
        """Validate, materialize, and atomically store a successful planning result."""
        try:
            if self._world_monitor is None or self._trajectory_parametrizer is None:
                raise ValueError("Trajectory parametrizer is not initialized")
            selection = self._world_monitor.planning_groups.select(group_ids)
            plan = self._trajectory_parametrizer.materialize_plan(
                world=self._world_monitor.world,
                selection=selection,
                result=result,
                speed_scale=speed_scale,
            )
        except Exception as exc:
            self._fail_planning_epoch(planning_epoch, f"Failed to materialize plan: {exc}")
            return None
        with self._lock:
            if self._state != ManipulationState.PLANNING or planning_epoch != self._planning_epoch:
                logger.info("Discarding cancelled planning result")
                return None
            self._last_plan = plan
            self._state = ManipulationState.COMPLETED
            self._error_message = ""
        return plan

    def _plan_selected_path(
        self,
        group_ids: tuple[PlanningGroupID, ...],
        start: JointState,
        goal: JointState,
        planning_epoch: int,
        speed_scale: float,
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
        return self._store_generated_plan(group_ids, result, planning_epoch, speed_scale)

    def _record_error(self, message: str) -> bool:
        """Record an error without changing the manipulation state."""
        logger.warning(message)
        self._error_message = message
        return False

    def _clear_pending_plan(self) -> None:
        """Invalidate the one authoritative pending plan."""

        with self._lock:
            plan = self._last_plan
            self._last_plan = None
        if plan is not None:
            self._dismiss_preview(plan.group_ids)

    def _fail(self, msg: str) -> bool:
        """Finish a planning request with an error while remaining retryable."""
        self._record_error(msg)
        with self._lock:
            self._last_plan = None
            self._state = ManipulationState.IDLE
        return False

    def _fail_planning_epoch(self, planning_epoch: int, msg: str) -> bool:
        """Fault only the still-current planning operation."""
        with self._lock:
            if self._state != ManipulationState.PLANNING or planning_epoch != self._planning_epoch:
                logger.info("Discarding cancelled planning result")
                return False
            logger.warning(msg)
            self._last_plan = None
            self._state = ManipulationState.IDLE
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
        selected_robot_name, _, _ = robot
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
        selected_robot_name, _, _ = robot
        try:
            group_id = self._require_unique_pose_group_id_for_robot(selected_robot_name)
        except ValueError as exc:
            logger.warning("Pose planning unavailable: %s", exc)
            self._record_error(str(exc))
            return False
        return self.plan_to_pose_targets({group_id: pose})

    def plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
    ) -> bool:
        """Plan to one or more group pose targets with optional auxiliary groups."""
        return self.generate_plan_to_pose_targets(pose_targets, auxiliary_groups) is not None

    @rpc
    def plan_to_joints(
        self,
        targets: Mapping[PlanningGroupID, JointState],
        speed_scale: float | None = None,
    ) -> PlanResult:
        """Plan one synchronized joint target set without moving hardware."""

        self._clear_pending_plan()
        if not targets:
            return PlanResult(PlanStatus.INVALID_TARGET, "At least one target is required")
        plan = self.generate_plan_to_joint_targets(
            cast("Mapping[PlanningGroupID | PlanningGroup, JointState]", targets),
            speed_scale=speed_scale,
        )
        if plan is None:
            return PlanResult(PlanStatus.FAILED, self._error_message or "Planning failed")
        return PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)

    @rpc
    def plan_to_poses(
        self,
        targets: Mapping[PlanningGroupID, PoseStamped],
        speed_scale: float | None = None,
    ) -> PlanResult:
        """Plan one synchronized pose target set without moving hardware."""

        self._clear_pending_plan()
        if not targets:
            return PlanResult(PlanStatus.INVALID_TARGET, "At least one target is required")
        plan = self.generate_plan_to_pose_targets(
            cast("Mapping[PlanningGroupID | PlanningGroup, Pose]", targets),
            speed_scale=speed_scale,
        )
        if plan is None:
            return PlanResult(PlanStatus.FAILED, self._error_message or "Planning failed")
        return PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)

    def plan_to_joint_targets(
        self, joint_targets: Mapping[PlanningGroupID | PlanningGroup, JointState]
    ) -> bool:
        """Plan to joint targets keyed by planning group."""
        return self.generate_plan_to_joint_targets(joint_targets) is not None

    def generate_plan_to_joint_targets(
        self,
        joint_targets: Mapping[PlanningGroupID | PlanningGroup, JointState],
        speed_scale: float | None = None,
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
        planning = self._begin_group_planning(speed_scale)
        if planning is None:
            return None
        planning_epoch, resolved_speed_scale = planning

        resolved = self._resolve_group_plan_start(group_ids, planning_epoch)
        if resolved is None:
            return None
        _selection, start = resolved

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
        return self._plan_selected_path(
            group_ids, start, goal, planning_epoch, resolved_speed_scale
        )

    def generate_plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
        speed_scale: float | None = None,
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
        planning = self._begin_group_planning(speed_scale)
        if planning is None:
            return None
        planning_epoch, resolved_speed_scale = planning
        resolved = self._resolve_group_plan_start(group_ids, planning_epoch)
        if resolved is None:
            return None
        _selection, start = resolved
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
        return self._plan_selected_path(
            group_ids, start, ik.joint_state, planning_epoch, resolved_speed_scale
        )

    def plan_cartesian_targets(
        self,
        targets: Mapping[PlanningGroupID | PlanningGroup, CartesianTarget],
        config: CartesianPathConfig,
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
        check_collision: bool = True,
    ) -> bool:
        """Plan TCP motion through absolute or relative Cartesian waypoints."""
        return (
            self.generate_cartesian_plan(
                targets,
                config,
                auxiliary_groups,
                check_collision=check_collision,
            )
            is not None
        )

    def generate_cartesian_plan(
        self,
        targets: Mapping[PlanningGroupID | PlanningGroup, CartesianTarget],
        config: CartesianPathConfig,
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
        speed_scale: float | None = None,
        check_collision: bool = True,
    ) -> GeneratedPlan | None:
        """Generate and store a timed Cartesian plan through PlannerSpec."""
        if self._world_monitor is None or self._planner is None:
            return None
        if not targets:
            self._fail("At least one Cartesian target is required")
            return None
        normalized_targets = {
            planning_group_id_from_selector(group): target for group, target in targets.items()
        }
        if len(normalized_targets) != len(targets):
            self._fail("Cartesian target groups must be unique")
            return None
        auxiliary_ids = tuple(planning_group_id_from_selector(group) for group in auxiliary_groups)
        group_ids = (*normalized_targets.keys(), *auxiliary_ids)
        planning = self._begin_group_planning(speed_scale)
        if planning is None:
            return None
        planning_epoch, resolved_speed_scale = planning
        resolved = self._resolve_group_plan_start(group_ids, planning_epoch)
        if resolved is None:
            return None
        selection, start = resolved
        scaled_config = config.model_copy(
            update={
                "velocity_scale": config.velocity_scale * resolved_speed_scale,
                "acceleration_scale": config.acceleration_scale * resolved_speed_scale,
            }
        )
        result = self._planner.plan_cartesian_path(
            world=self._world_monitor.world,
            selection=selection,
            start=start,
            targets=normalized_targets,
            config=scaled_config,
            auxiliary_groups=auxiliary_ids,
            check_collision=check_collision,
        )
        if not result.is_success():
            detail = f": {result.message}" if result.message else ""
            self._fail_planning_epoch(
                planning_epoch, f"Cartesian planning failed: {result.status.name}{detail}"
            )
            return None
        return self._store_generated_plan(group_ids, result, planning_epoch, resolved_speed_scale)

    @rpc
    def move_linear(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        planning_group: PlanningGroupID | None = None,
        check_collision: bool = False,
        speed_scale: float | None = None,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> MoveResult:
        """Move one end effector by a world-frame translation."""

        delta = (float(dx), float(dy), float(dz))
        self._clear_pending_plan()
        if not all(math.isfinite(value) for value in delta):
            plan_result = PlanResult(PlanStatus.INVALID_TARGET, "delta must be finite")
            return MoveResult(plan_result, None, delta, check_collision)
        if delta == (0.0, 0.0, 0.0):
            plan_result = PlanResult(PlanStatus.NO_MOTION, "Linear displacement is zero")
            return MoveResult(plan_result, None, delta, check_collision)
        group = self._resolve_pose_group(planning_group)
        if isinstance(group, CommandResult):
            plan_result = PlanResult(PlanStatus.AMBIGUOUS_GROUP, group.message)
            return MoveResult(plan_result, None, delta, check_collision)
        resolved_speed = self.config.linear_speed_scale if speed_scale is None else speed_scale
        config = RoboPlanCartesianPathConfig()
        relative = Transform(
            translation=Vector3(*delta),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        plan = self.generate_cartesian_plan(
            {group.id: (Transform.identity(), relative)},
            config,
            speed_scale=resolved_speed,
            check_collision=check_collision,
        )
        if plan is None:
            plan_result = PlanResult(PlanStatus.FAILED, self._error_message or "Planning failed")
            return MoveResult(plan_result, None, delta, check_collision)
        plan_result = PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)
        execution = self.execute(blocking=blocking, timeout=timeout)
        return MoveResult(plan_result, execution, delta, check_collision)

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
        if robot_name is not None and robot_name not in affected:
            logger.error("Generated plan does not affect robot '%s'", robot_name)
            return False
        if self._world_monitor is None:
            return False
        self._world_monitor.animate_trajectory(plan.trajectory, duration)
        return True

    def has_planned_path(self) -> bool:
        """Check if there's a planned path ready.

        Returns:
            True if a path is planned and ready
        """
        return self._last_plan is not None and bool(self._last_plan.path)

    def get_visualization_url(self) -> str | None:
        """Get the visualization URL.

        Returns:
            URL string or None if visualization not enabled
        """
        if self._world_monitor is None:
            return None
        return self._world_monitor.get_visualization_url()

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

    def list_robots(self) -> list[str]:
        """List all configured robot names.

        Returns:
            List of robot names
        """
        return list(self._robots.keys())

    @rpc
    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]:
        """Return public planning-group capabilities."""
        if self._world_monitor is None:
            return ()
        return tuple(
            PlanningGroupInfo(
                id=group.id,
                joint_names=group.joint_names,
                base_frame=group.base_link,
                tip_frame=group.tip_link,
                has_gripper=self._group_robot_config(group).gripper_hardware_id is not None,
            )
            for group in self._world_monitor.planning_groups.list()
        )

    def _group_robot_config(self, group: PlanningGroup) -> RobotModelConfig:
        return self._robots[group.robot_name][1]

    def _group_joint_presets(self, group: PlanningGroup) -> dict[str, JointState]:
        config = self._group_robot_config(group)
        presets: dict[str, JointState] = {}

        def selected(state: JointState, source_names: Sequence[str]) -> JointState:
            positions = dict(zip(state.name, state.position, strict=True))
            return JointState(
                name=list(group.joint_names),
                position=[positions[name] for name in source_names],
            )

        if config.home_joints is not None:
            presets["home"] = selected(
                JointState(name=config.joint_names, position=config.home_joints),
                group.local_joint_names,
            )
        if init := self._init_joints.get(group.robot_name):
            presets["init"] = selected(
                init,
                tuple(config.get_coordinator_joint_name(name) for name in group.local_joint_names),
            )
        return presets

    def _get_group_gripper_position(self, group: PlanningGroup) -> float | None:
        config = self._group_robot_config(group)
        if config.gripper_hardware_id is None:
            return None
        values = self._control_coordinator.task_invoke(
            f"{config.gripper_hardware_id}_gripper", "get_normalized", {}
        )
        return float(values[0]) if values else None

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        """Return the named robot's current local joint state with names."""
        if self._world_monitor is None:
            return None
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return self._world_monitor.get_current_joint_state(robot_id)

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

        robot_name, robot_id, config = robot
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
            "home_joints": config.home_joints,
            "pre_grasp_offset": config.pre_grasp_offset,
            "init_joints": list(init.position)
            if (init := self._init_joints.get(robot_name))
            else None,
        }

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        """Return configured robots for in-process visualization adapters."""
        return [(name, robot_id, config) for name, (robot_id, config) in self._robots.items()]

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        """Return the planning-world robot id for a configured robot name."""
        entry = self._robots.get(robot_name)
        return entry[0] if entry is not None else None

    def robot_name_for_id(self, robot_id: WorldRobotID) -> RobotName | None:
        """Return the configured robot name for a planning-world robot id."""
        for robot_name, (candidate_id, _) in self._robots.items():
            if candidate_id == robot_id:
                return robot_name
        return None

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        """Return the robot model config for an in-process visualization adapter."""
        entry = self._robots.get(robot_name)
        return entry[1] if entry is not None else None

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

    def set_init_joints_to_current(self, robot_name: RobotName | None = None) -> bool:
        """Set init joints to the current joint positions.

        Args:
            robot_name: Robot to capture from (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        robot_name_resolved, robot_id, _ = robot
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

    def _initialize_execution(self) -> None:
        """Initialize coordinator access and planned execution policy."""
        targets = [
            ExecutionTarget.from_coordinator_mapping(
                robot_name=config.name,
                model_joint_names=config.joint_names,
                coordinator_to_model=config.joint_name_mapping,
            )
            for config in self.config.robots
        ]
        self._execution_manager = PlanExecutionManager(
            targets=targets,
            coordinator=self._control_coordinator,
            default_timeout=self.config.execution_timeout,
        )

    @rpc
    def execute(self, blocking: bool = True, timeout: float | None = None) -> ExecutionResult:
        """Dispatch the one pending plan, optionally waiting for JTT completion."""

        with self._lock:
            target_plan = self._last_plan
            self._last_plan = None
            if target_plan is None:
                return ExecutionResult(ExecutionStatus.NO_PLAN, "No pending plan")
            self._state = ManipulationState.EXECUTING
        try:
            result = self._execution_manager.execute(
                target_plan,
                blocking=blocking,
                timeout=timeout,
            )
        except Exception as exc:
            logger.exception("Failed to dispatch generated plan")
            result = ExecutionResult(ExecutionStatus.UNCERTAIN, str(exc))
        self._apply_execution_result(result)
        return result

    @rpc
    def wait_for_execution(self, timeout: float | None = None) -> ExecutionResult:
        """Wait for the active trajectory or return its cached terminal result."""
        result = self._execution_manager.wait(timeout)
        self._apply_execution_result(result)
        return result

    def _apply_execution_result(self, result: ExecutionResult) -> None:
        """Mirror execution ownership into the broader manipulation snapshot."""
        with self._lock:
            if result.status in {ExecutionStatus.ACCEPTED, ExecutionStatus.EXECUTING}:
                self._state = ManipulationState.EXECUTING
                self._error_message = ""
            elif result.status is ExecutionStatus.COMPLETED:
                self._state = ManipulationState.COMPLETED
                self._error_message = ""
            elif result.status in {
                ExecutionStatus.UNCERTAIN,
                ExecutionStatus.FAULT,
            }:
                self._state = ManipulationState.FAULT
                self._error_message = result.message
            elif result.status is not ExecutionStatus.TIMED_OUT:
                self._state = ManipulationState.IDLE
                self._error_message = result.message

    def _execute_generated_plan(self, plan: GeneratedPlan) -> bool:
        """Private in-process bridge for the visualization operator."""

        with self._lock:
            self._last_plan = plan
            self._state = ManipulationState.COMPLETED
        result = self.execute(blocking=False)
        return result.status is ExecutionStatus.ACCEPTED

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

        obstacle_type = _SHAPE_TO_OBSTACLE_TYPE.get(shape)
        if obstacle_type is None:
            logger.warning(f"Unknown obstacle shape: {shape}")
            return ""

        # Validate mesh_path for mesh type
        if obstacle_type == ObstacleType.MESH and not mesh_path:
            logger.warning("mesh_path required for mesh obstacles")
            return ""

        obstacle = Obstacle(
            name=name,
            obstacle_type=obstacle_type,
            pose=PoseStamped(position=pose.position, orientation=pose.orientation),
            dimensions=tuple(dimensions) if dimensions else (),
            mesh_path=mesh_path,
        )
        return self._world_monitor.add_obstacle(obstacle)

    @rpc
    def update_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: ObstacleShape,
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
        color: list[float] | None = None,
    ) -> bool:
        """Replace a complete obstacle identified by name."""
        if self._world_monitor is None:
            return False

        obstacle_type = _SHAPE_TO_OBSTACLE_TYPE.get(shape)
        if obstacle_type is None:
            raise ValueError(f"Unknown obstacle shape: {shape}")
        if obstacle_type == ObstacleType.MESH and not mesh_path:
            raise ValueError("mesh_path required for mesh obstacles")
        if color is None:
            rgba = DEFAULT_OBSTACLE_RGBA
        elif len(color) == 4:
            rgba = (float(color[0]), float(color[1]), float(color[2]), float(color[3]))
        else:
            raise ValueError("Obstacle color must contain four values")

        obstacle = Obstacle(
            name=name,
            obstacle_type=obstacle_type,
            pose=PoseStamped(position=pose.position, orientation=pose.orientation),
            dimensions=tuple(dimensions) if dimensions else (),
            color=rgba,
            mesh_path=mesh_path,
        )
        return self._world_monitor.update_obstacle(obstacle)

    @rpc
    def update_obstacle_pose(self, name: str, pose: Pose) -> bool:
        """Update only an obstacle pose while preserving all other properties."""
        if self._world_monitor is None:
            return False
        return self._world_monitor.update_obstacle_pose(
            name,
            PoseStamped(position=pose.position, orientation=pose.orientation),
        )

    @rpc
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the planning world."""
        if self._world_monitor is None:
            return False
        return self._world_monitor.remove_obstacle(obstacle_id)

    @rpc
    def set_gripper_position(
        self,
        position: float,
        planning_group: PlanningGroupID | None = None,
    ) -> CommandResult:
        """Set normalized gripper opening for one planning group."""

        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            return CommandResult(
                CommandStatus.REJECTED,
                "position must be finite and between 0.0 (closed) and 1.0 (open)",
            )
        group = self._resolve_gripper_group(planning_group)
        if isinstance(group, CommandResult):
            return group
        config = self._group_robot_config(group)
        assert config.gripper_hardware_id is not None
        if self._control_coordinator.task_invoke(
            f"{config.gripper_hardware_id}_gripper",
            "set_normalized",
            {"values": [float(position)]},
        ):
            return CommandResult(CommandStatus.SUCCEEDED, f"Gripper set to {position:.0%} open")
        return CommandResult(CommandStatus.FAILED, "Failed to set gripper position")

    def _resolve_pose_group(
        self,
        planning_group: PlanningGroupID | None,
    ) -> PlanningGroup | CommandResult:
        return self._resolve_group_with_capability(planning_group, "pose")

    def _resolve_gripper_group(
        self,
        planning_group: PlanningGroupID | None,
    ) -> PlanningGroup | CommandResult:
        return self._resolve_group_with_capability(planning_group, "gripper")

    def _resolve_group_with_capability(
        self,
        planning_group: PlanningGroupID | None,
        capability: Literal["pose", "gripper"],
    ) -> PlanningGroup | CommandResult:
        if self._world_monitor is None:
            return CommandResult(CommandStatus.FAILED, "Planning is not initialized")
        groups = self._world_monitor.planning_groups.list()

        def is_capable(group: PlanningGroup) -> bool:
            if capability == "pose":
                return group.has_pose_target
            return self._group_robot_config(group).gripper_hardware_id is not None

        if planning_group is not None:
            try:
                group = self._world_monitor.planning_groups.get(planning_group)
            except KeyError:
                return CommandResult(CommandStatus.REJECTED, f"Unknown group: {planning_group}")
            if not is_capable(group):
                return CommandResult(
                    CommandStatus.REJECTED,
                    f"Planning group '{group.id}' is not {capability}-capable",
                )
            return group

        candidates = tuple(group for group in groups if is_capable(group))
        if len(candidates) != 1:
            return CommandResult(
                CommandStatus.REJECTED,
                f"Expected one {capability}-capable planning group, found {len(candidates)}",
            )
        return candidates[0]

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
        result = self.execute(blocking=True)
        if not result.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", result.message or result.status.name)
        return SkillResult.ok(str(result))

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        execution_manager = getattr(self, "_execution_manager", None)
        if execution_manager is not None:
            cancellation = execution_manager.cancel()
            if cancellation.status is ExecutionStatus.UNCERTAIN:
                logger.error(
                    "Shutdown could not confirm coordinator trajectory safety: %s",
                    cancellation.message,
                )

        # Stop TF thread
        if self._tf_thread is not None:
            self._tf_stop_event.set()
            self._tf_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._tf_thread = None

        # Stop world monitor (includes visualization thread)
        if self._world_monitor is not None:
            self._world_monitor.stop_all_monitors()

        super().stop()
