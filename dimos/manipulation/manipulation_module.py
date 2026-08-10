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
- @rpc: Low-level building blocks (plan_to_pose, plan_to_joints, preview_plan, execute)
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
import traceback
from typing import Any, Literal, TypeAlias

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.execution_manager import (
    ExecutionOutcome,
    PlanExecutionManager,
)
from dimos.manipulation.planning.factory import WorldBackend, create_planning_specs, create_world
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    normalize_joint_target,
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
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanPlannerConfig
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

ModelInfoValue: TypeAlias = (
    str | bool | float | list[str] | list[float] | list[PlanningGroup] | None
)
ModelInfoPayload: TypeAlias = dict[str, ModelInfoValue]

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

    model: RobotModelConfig
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
        self._motion_speed_scale = 1.0

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner: PlannerSpec | None = None
        self._kinematics: KinematicsSpec | None = None
        self._trajectory_parametrizer: TrajectoryParametrizerSpec | None = None

        # Canonical generated plan for plan/preview/execute workflow.
        # Robot-local paths and trajectories are derived from this plan on demand.
        self._last_plan: GeneratedPlan | None = None

        # Coordinator integration (initialized in start())
        self._execution_manager: PlanExecutionManager

        # Init joints captured from the first complete canonical state.
        self._init_joints: JointState | None = None

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
        self._initialize_execution()

        # Subscribe to joint state via port
        if self.coordinator_joint_state is not None:
            self.coordinator_joint_state.subscribe(self._on_joint_state)
            logger.info("Subscribed to coordinator_joint_state port")

        logger.info("ManipulationModule started")

    def _initialize_planning(self) -> None:
        """Initialize world, planner, and trajectory generator."""
        world = create_world(
            backend=self.config.world_backend,
            visualization=self.config.visualization,
        )
        planning_specs = create_planning_specs(
            world=world,
            world_backend=self.config.world_backend,
            planner=self.config.planner,
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

        self._world_monitor.load_model(self.config.model)

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

        self._world_monitor.start_state_monitor()

        if self._world_monitor.visualization is not None:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info(f"Visualization: {url}")

        # Start TF publishing thread if any robot has tf_extra_links
        if self.config.model.tf_extra_links:
            self._tf_stop_event.clear()
            self._tf_thread = threading.Thread(
                target=self._tf_publish_loop, name="ManipTFThread", daemon=True
            )
            self._tf_thread.start()
            logger.info("TF publishing thread started")

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback when joint state received from driver.

        Validates and forwards the complete canonical model state.
        """
        try:
            if self._world_monitor is None:
                return

            name_to_idx = {name: i for i, name in enumerate(msg.name)}
            names = self.config.model.joint_names
            missing = [name for name in names if name not in name_to_idx]
            if missing:
                logger.warning("Skipping incomplete model state; missing joints %s", missing)
                return
            indices = [name_to_idx[name] for name in names]
            state = JointState(
                name=list(names),
                position=[msg.position[index] for index in indices],
                velocity=[msg.velocity[index] for index in indices]
                if len(msg.velocity) == len(msg.name)
                else [],
            )
            self._world_monitor.on_joint_state(state)
            if self._init_joints is None:
                self._init_joints = state
                logger.info("Init joints captured")

        except Exception as e:
            logger.error(f"Exception in _on_joint_state: {e}")
            logger.error(traceback.format_exc())

    def _tf_publish_loop(self) -> None:
        """Publish TF transforms at 10Hz for EE and extra links."""
        period = 0.1  # 10Hz
        while not self._tf_stop_event.is_set():
            try:
                if self._world_monitor is None:
                    break
                transforms: list[Transform] = []
                config = self.config.model
                group_id = self._world_monitor.planning_groups.primary_pose_group_id()
                if group_id is not None:
                    group = self._world_monitor.planning_groups.get(group_id)
                    ee_pose = self._world_monitor.get_ee_pose()
                    if ee_pose is not None and group.tip_link is not None:
                        ee_tf = Transform.from_pose(group.tip_link, ee_pose)
                        ee_tf.frame_id = "world"
                        transforms.append(ee_tf)
                for link_name in config.tf_extra_links:
                    link_pose = self._world_monitor.get_link_pose(link_name)
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
    def get_state(self) -> str:
        """Get current manipulation state name."""
        return self._state.name

    def get_operation_status(self) -> OperationStatus:
        """Return the current operation status without collecting telemetry."""
        with self._lock:
            return OperationStatus[self._state.name]

    @rpc
    def get_error(self) -> str:
        """Get last error message.

        Returns:
            Error message or empty string
        """
        return self._error_message

    @rpc
    def set_motion_speed(self, speed_scale: float) -> bool:
        """Set a runtime speed reduction for plans generated in the future.

        Existing accepted plans and dispatched trajectories remain unchanged.
        Plan again after changing this value.
        """
        if not math.isfinite(speed_scale) or speed_scale <= 0.0 or speed_scale > 1.0:
            self._record_error("motion speed scale must be finite, > 0, and <= 1")
            return False
        with self._lock:
            self._motion_speed_scale = float(speed_scale)
            self._error_message = ""
        return True

    @rpc
    def get_motion_speed(self) -> float:
        """Return the runtime speed reduction used for future plans."""
        with self._lock:
            return self._motion_speed_scale

    @rpc
    def cancel(self) -> bool:
        """Cancel current motion or invalidate an in-progress plan."""
        with self._lock:
            is_planning = self._state == ManipulationState.PLANNING
            is_executing = self._state == ManipulationState.EXECUTING
            if is_planning:
                self._planning_epoch += 1
            plan = self._last_plan

        cancellation = self._execution_manager.cancel()
        had_execution = cancellation.cancelled
        if not (is_planning or is_executing or had_execution):
            return False

        with self._lock:
            if not cancellation.safe:
                self._state = ManipulationState.FAULT
                self._error_message = cancellation.message or (
                    "Failed to confirm coordinator trajectory cancellation"
                )
                logger.error(self._error_message)
                return False
            self._state = ManipulationState.IDLE
            self._error_message = ""
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
    def get_current_joints(self) -> list[float] | None:
        """Get the complete canonical model joint positions."""
        if self._world_monitor:
            state = self._world_monitor.get_current_joint_state()
            if state is not None:
                return list(state.position)
        return None

    @rpc
    def get_ee_pose(self, group_id: PlanningGroupID | None = None) -> Pose | None:
        """Get a planning group's current tip pose."""
        if self._world_monitor:
            try:
                selected = group_id or self._require_unique_pose_group_id()
                return self._world_monitor.get_group_ee_pose(selected)
            except ValueError as exc:
                logger.warning("End-effector pose unavailable: %s", exc)
                return None
        return None

    @rpc
    def is_collision_free(self, joints: list[float]) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check
        """
        if self._world_monitor:
            joint_state = JointState(name=self.config.model.joint_names, position=joints)
            return self._world_monitor.is_state_valid(joint_state)
        return False

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

    def _require_unique_pose_group_id(self) -> PlanningGroupID:
        """Return the unique pose-targetable group or raise if it is ambiguous."""
        if self._world_monitor is None:
            raise ValueError("Planning not initialized")
        group_id = self._world_monitor.planning_groups.primary_pose_group_id()
        if group_id is None:
            raise ValueError(
                "Model has no unique pose-targetable planning group; use an explicit group ID"
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
            current = self._world_monitor.current_model_joint_state()
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
                speed_scale=self.get_motion_speed(),
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
        return plan

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
        return self._store_generated_plan(group_ids, result, planning_epoch)

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
        self._world_monitor.cancel_preview_animation()

    def _solve_ik_for_pose(
        self,
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
                current = self._world_monitor.current_model_joint_state()
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
        group_id: PlanningGroupID | None = None,
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> IKResult:
        """Solve IK for one selected or unambiguous pose-targetable group."""
        if self._world_monitor is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
        try:
            selected_group_id = group_id or self._require_unique_pose_group_id()
        except ValueError as exc:
            return IKResult(status=IKStatus.NO_SOLUTION, message=str(exc))
        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )
        return self.inverse_kinematics(
            {selected_group_id: target_pose}, seed=seed, check_collision=check_collision
        )

    @rpc
    def solve_ik(
        self,
        pose: Pose,
        group_id: PlanningGroupID | None = None,
        check_collision: bool = True,
        seed: JointState | None = None,
    ) -> IKResult:
        """Solve IK for a pose without planning a joint path.

        Args:
            pose: Target end-effector pose
            group_id: Planning group to solve for; omission requires one compatible group
            check_collision: Whether to reject IK candidates in collision
            seed: Optional joint state to initialize local IK. Uses current state when omitted.
        """
        if self._kinematics is None or self._world_monitor is None:
            self._record_error("Planning not initialized")
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
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
            group_id=group_id,
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
    def plan_to_pose(self, pose: Pose, group_id: PlanningGroupID | None = None) -> bool:
        """Plan motion to pose. Use preview_plan() then execute().

        Args:
            pose: Target end-effector pose
            group_id: Planning group to use; omission requires one compatible group
        """
        if self._kinematics is None or self._world_monitor is None:
            self._record_error("Planning not initialized")
            return False
        try:
            selected_group_id = group_id or self._require_unique_pose_group_id()
        except ValueError as exc:
            logger.warning("Pose planning unavailable: %s", exc)
            self._record_error(str(exc))
            return False
        return self.plan_to_pose_targets({selected_group_id: pose})

    @rpc
    def plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
    ) -> bool:
        """Plan to one or more group pose targets with optional auxiliary groups."""
        return self.generate_plan_to_pose_targets(pose_targets, auxiliary_groups) is not None

    @rpc
    def plan_to_joints(self, joints: JointState, group_id: PlanningGroupID | None = None) -> bool:
        """Plan motion to joint config. Use preview_plan() then execute().

        Args:
            joints: Target joint state (names + positions)
            group_id: Planning group to use; omission requires exactly one group
        """
        logger.info("Planning to joints: %s", [f"{j:.3f}" for j in joints.position])
        if self._world_monitor is None:
            self._record_error("Planning not initialized")
            return False
        selected_group_id = group_id or self._world_monitor.planning_groups.default_group_id()
        if selected_group_id is None:
            logger.error("Model has no unique default planning group; select a group explicitly")
            return False
        return self.plan_to_joint_targets({selected_group_id: joints})

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
                target_global = normalize_joint_target(target_group, target)
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
        return self._plan_selected_path(group_ids, start, ik.joint_state, planning_epoch)

    @rpc
    def plan_cartesian_targets(
        self,
        targets: Mapping[PlanningGroupID | PlanningGroup, CartesianTarget],
        config: CartesianPathConfig,
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
    ) -> bool:
        """Plan TCP motion through absolute or relative Cartesian waypoints."""
        return self.generate_cartesian_plan(targets, config, auxiliary_groups) is not None

    def generate_cartesian_plan(
        self,
        targets: Mapping[PlanningGroupID | PlanningGroup, CartesianTarget],
        config: CartesianPathConfig,
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
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
        group_ids = tuple((*normalized_targets.keys(), *auxiliary_ids))
        planning_epoch = self._begin_group_planning()
        if planning_epoch is None:
            return None
        resolved = self._resolve_group_plan_start(group_ids, planning_epoch)
        if resolved is None:
            return None
        selection, start = resolved
        speed_scale = self.get_motion_speed()
        scaled_config = config.model_copy(
            update={
                "velocity_scale": config.velocity_scale * speed_scale,
                "acceleration_scale": config.acceleration_scale * speed_scale,
            }
        )
        result = self._planner.plan_cartesian_path(
            world=self._world_monitor.world,
            selection=selection,
            start=start,
            targets=normalized_targets,
            config=scaled_config,
            auxiliary_groups=auxiliary_ids,
        )
        if not result.is_success():
            detail = f": {result.message}" if result.message else ""
            self._fail_planning_epoch(
                planning_epoch, f"Cartesian planning failed: {result.status.name}{detail}"
            )
            return None
        return self._store_generated_plan(group_ids, result, planning_epoch)

    @rpc
    def preview_plan(
        self,
        plan: GeneratedPlan | None = None,
        duration: float | None = None,
    ) -> bool:
        """Preview a complete generated plan in the visualizer."""
        plan = plan or self._last_plan
        if plan is None or not plan.path:
            logger.warning("No generated plan to preview")
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
    def list_planning_groups(self) -> list[PlanningGroup]:
        """Return all configured planning groups."""
        if self._world_monitor is None:
            return []
        return list(self._world_monitor.planning_groups.list())

    def get_current_joint_state(self) -> JointState | None:
        """Return the complete canonical model joint state."""
        if self._world_monitor is None:
            return None
        return self._world_monitor.get_current_joint_state()

    @rpc
    def get_model_info(self) -> ModelInfoPayload:
        """Get information about the configured logical robot model."""
        config = self.config.model
        planning_groups = self.list_planning_groups()
        pose_tip_links = [
            group.tip_link
            for group in planning_groups
            if group.has_pose_target and group.tip_link is not None
        ]
        end_effector_link = pose_tip_links[0] if len(pose_tip_links) == 1 else None

        return {
            "joint_names": config.joint_names,
            "planning_groups": planning_groups,
            "end_effector_link": end_effector_link,
            "base_link": config.base_link,
            "max_velocity": config.max_velocity,
            "max_acceleration": config.max_acceleration,
            "home_joints": config.home_joints,
            "pre_grasp_offset": config.pre_grasp_offset,
            "init_joints": list(self._init_joints.position)
            if self._init_joints is not None
            else None,
        }

    def get_model_config(self) -> RobotModelConfig:
        """Return the configured model for in-process visualization adapters."""
        return self.config.model

    @rpc
    def get_init_joints(self) -> JointState | None:
        """Get the init joint state captured at startup or set manually."""
        return self._init_joints

    def evaluate_joint_target(self, joints: JointState | None) -> TargetEvaluation:
        """Evaluate a joint target for visualization without planning a path."""
        if self._world_monitor is None:
            return {
                "success": False,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized",
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
        collision_free = self._world_monitor.is_state_valid(target)
        return {
            "success": True,
            "status": "FEASIBLE" if collision_free else "COLLISION",
            "message": "Target is collision-free" if collision_free else "Target is in collision",
            "collision_free": collision_free,
            "ee_pose": self._world_monitor.get_ee_pose(target),
            "joint_state": target,
        }

    def evaluate_pose_target(self, pose: Pose) -> TargetEvaluation:
        """Evaluate a Cartesian target for visualization without planning a path."""
        if self._world_monitor is None or self._kinematics is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized or current state is unavailable",
                "collision_free": False,
            }
        current = self._world_monitor.get_current_joint_state()
        if current is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized or current state is unavailable",
                "collision_free": False,
            }
        ik = self._solve_ik_for_pose(pose, current, check_collision=True)
        joint_state = JointState(ik.joint_state) if ik.is_success() and ik.joint_state else None
        collision_free = bool(
            joint_state is not None and self._world_monitor.is_state_valid(joint_state)
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
    def set_init_joints(self, joint_state: JointState) -> bool:
        """Set the init joint state.

        Args:
            joint_state: New init joint state (names + positions)
        """
        self._init_joints = joint_state
        logger.info("Init joints set: [%s]", ", ".join(f"{j:.3f}" for j in joint_state.position))
        return True

    @rpc
    def set_init_joints_to_current(self) -> bool:
        """Set init joints to the current joint positions."""
        if self._world_monitor is None:
            return False
        current = self._world_monitor.get_current_joint_state()
        if current is None:
            logger.error("Cannot capture init joints — no current joint state")
            return False
        self._init_joints = current
        logger.info("Init joints set to current")
        return True

    def _initialize_execution(self) -> None:
        """Initialize coordinator access and planned execution policy."""
        self._execution_manager = PlanExecutionManager(
            joint_names=self.config.model.joint_names,
            coordinator=self._control_coordinator,
        )

    @rpc
    def execute(self) -> bool:
        """Compatibility wrapper for execute_plan()."""
        return self.execute_plan()

    @rpc
    def execute_plan(self, plan: GeneratedPlan | None = None) -> bool:
        """Execute a generated planning-group plan through the coordinator."""
        target_plan = plan or self._last_plan
        if target_plan is None:
            self._error_message = "Stored plan is invalid or not executable"
            return False

        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning("Manipulation state is not executable")
                return False
            previous_state = self._state
            self._state = ManipulationState.EXECUTING
        try:
            result = self._execution_manager.execute(target_plan)
        except Exception as exc:
            result = None
            message = f"Failed to dispatch generated plan: {exc}"
            logger.exception(message)
        with self._lock:
            if self._state != ManipulationState.EXECUTING:
                return False
            if result is None:
                self._state = previous_state
                self._error_message = message
            else:
                match result.outcome:
                    case ExecutionOutcome.ACCEPTED:
                        self._state = ManipulationState.COMPLETED
                        self._error_message = ""
                    case ExecutionOutcome.REJECTED:
                        self._state = previous_state
                        self._error_message = result.message
                    case ExecutionOutcome.UNCERTAIN:
                        self._state = ManipulationState.FAULT
                        self._error_message = result.message
        return bool(result and result.accepted)

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

    def _get_gripper_hardware_id(self) -> str | None:
        """Get the configured legacy gripper hardware ID."""
        config = self.config.model
        if not config.gripper_hardware_id:
            logger.warning("No gripper_hardware_id configured")
            return None
        return str(config.gripper_hardware_id)

    def _set_gripper_position(self, position: float) -> bool:
        """Internal: set gripper position in meters."""
        hw_id = self._get_gripper_hardware_id()
        if hw_id is None:
            return False
        return self._control_coordinator.set_gripper_position(hw_id, position)

    @rpc
    def get_gripper(self) -> float | None:
        """Get gripper position in meters."""
        hw_id = self._get_gripper_hardware_id()
        if hw_id is None:
            return None
        result = self._control_coordinator.get_gripper_position(hw_id)
        return float(result) if result is not None else None

    @skill
    def set_gripper(self, position: float) -> SkillResult[ManipulationSkillError]:
        """Set gripper to a specific opening in meters.

        Args:
            position: Gripper opening in meters (0.0 = closed, 0.85 = fully open).
        """
        if self._set_gripper_position(position):
            return SkillResult.ok(f"Gripper set to {position:.3f}m")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to set gripper position")

    @skill
    def open_gripper(self) -> SkillResult[ManipulationSkillError]:
        """Open the robot gripper fully."""
        if self._set_gripper_position(0.85):
            return SkillResult.ok("Gripper opened")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to open gripper")

    @skill
    def close_gripper(self) -> SkillResult[ManipulationSkillError]:
        """Close the robot gripper fully."""
        if self._set_gripper_position(0.0):
            return SkillResult.ok("Gripper closed")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to close gripper")

    def _wait_for_trajectory_completion(self, timeout: float = 60.0) -> bool:
        """Wait for the duration of the last accepted trajectory."""
        last_plan = self._last_plan
        if last_plan is None:
            return True
        wait_time = last_plan.trajectory.duration + 0.5
        if wait_time > timeout:
            logger.warning(f"Trajectory duration exceeds timeout of {timeout}s")
            return False
        time.sleep(wait_time)
        return True

    def _lift_if_low(
        self, group_id: PlanningGroupID | None = None, min_z: float = 0.05
    ) -> SkillResult[ManipulationSkillError]:
        """If the end-effector is below *min_z*, plan and execute a short lift."""
        ee = self.get_ee_pose(group_id)
        if ee is None or ee.position.z >= min_z:
            return SkillResult.ok()

        lift_z = min_z + 0.05
        logger.info(f"EE z={ee.position.z:.3f} < {min_z}, lifting to z={lift_z:.3f}")
        lift_pose = Pose(Vector3(ee.position.x, ee.position.y, lift_z), ee.orientation)
        if not self.plan_to_pose(lift_pose, group_id):
            return SkillResult.fail(
                "PLANNING_FAILED",
                f"Failed to plan lift from z={ee.position.z:.3f}",
            )
        return self._preview_execute_wait()

    def _preview_execute_wait(
        self, preview_duration: float = 0.5
    ) -> SkillResult[ManipulationSkillError]:
        """Preview planned path, execute, and wait for completion.

        Args:
            preview_duration: Duration to animate the preview in Meshcat (seconds)
        """
        logger.info("Previewing trajectory...")
        self.preview_plan(duration=preview_duration)

        logger.info("Executing trajectory...")
        if not self.execute():
            return SkillResult.fail("EXECUTION_FAILED", "Trajectory execution failed")

        if not self._wait_for_trajectory_completion():
            return SkillResult.fail("EXECUTION_TIMEOUT", "Trajectory execution timed out")

        return SkillResult.ok()

    @skill
    def get_robot_state(
        self, group_id: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Get model joints, a selected end-effector pose, and gripper state."""
        lines: list[str] = []

        joints = self.get_current_joints()
        if joints is not None:
            lines.append(f"Joints: [{', '.join(f'{j:.3f}' for j in joints)}]")
        else:
            lines.append("Joints: unavailable (no state received)")

        ee_pose = self.get_ee_pose(group_id)
        if ee_pose is not None:
            p = ee_pose.position
            lines.append(f"EE pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
        else:
            lines.append("EE pose: unavailable")

        gripper_pos = self.get_gripper()
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
        group_id: PlanningGroupID | None = None,
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
            group_id: Planning group to move; omission requires one compatible group.
        """
        logger.info(f"Planning motion to ({x:.3f}, {y:.3f}, {z:.3f})...")

        # If no orientation specified, preserve the current EE orientation.
        # If partially specified, fill unspecified angles from current orientation.
        if roll is None and pitch is None and yaw is None:
            current_pose = self.get_ee_pose(group_id)
            if current_pose is not None:
                orientation = current_pose.orientation
            else:
                orientation = Quaternion(0, 0, 0, 1)  # identity fallback
        else:
            current_pose = self.get_ee_pose(group_id)
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
        lift = self._lift_if_low(group_id)
        if not lift.is_success():
            return lift

        if not self.plan_to_pose(pose, group_id):
            return SkillResult.fail(
                "PLANNING_FAILED",
                f"Pose ({x:.3f}, {y:.3f}, {z:.3f}) may be unreachable or in collision",
            )

        exec_result = self._preview_execute_wait()
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok(f"Reached target pose ({x:.3f}, {y:.3f}, {z:.3f})")

    @skill
    def move_to_joints(
        self,
        joints: str,
        group_id: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to a target joint configuration.

        Plans a collision-free trajectory and executes it.

        Args:
            joints: Comma-separated joint positions in radians, e.g. "0.1, -0.5, 1.2, 0.0, 0.3, -0.1".
            group_id: Planning group to move; omission requires exactly one group.
        """
        try:
            joint_values = [float(j.strip()) for j in joints.split(",")]
        except ValueError:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"Invalid joints format '{joints}'. Expected comma-separated floats.",
            )

        if self._world_monitor is None:
            return SkillResult.fail("PLANNING_FAILED", "Planning not initialized")
        selected_group_id = group_id or self._world_monitor.planning_groups.default_group_id()
        if selected_group_id is None:
            return SkillResult.fail("INVALID_INPUT", "Select a planning group explicitly")
        group = self._world_monitor.planning_groups.get(selected_group_id)
        goal = JointState(name=list(group.joint_names), position=joint_values)

        logger.info(f"Planning motion to joints [{', '.join(f'{j:.3f}' for j in joint_values)}]...")
        if not self.plan_to_joints(goal, selected_group_id):
            return SkillResult.fail(
                "PLANNING_FAILED",
                "Joint configuration may be unreachable or in collision",
            )

        exec_result = self._preview_execute_wait()
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached target joint configuration")

    def _model_positions_for_group(
        self, positions: Sequence[float], group_id: PlanningGroupID | None
    ) -> JointState | None:
        return self._model_state_for_group(
            JointState(name=self.config.model.joint_names, position=list(positions)), group_id
        )

    def _model_state_for_group(
        self, state: JointState, group_id: PlanningGroupID | None
    ) -> JointState | None:
        if self._world_monitor is None:
            return None
        selected_group_id = group_id or self._world_monitor.planning_groups.default_group_id()
        if selected_group_id is None:
            return None
        group = self._world_monitor.planning_groups.get(selected_group_id)
        return filter_joint_state_to_selected_joints(state, group.joint_names)

    @skill
    def go_home(
        self, group_id: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its home/observe joint configuration.

        Opens the gripper and moves to the predefined home position.

        Args:
            group_id: Planning group to move; omission requires exactly one group.
        """
        config = self.config.model

        if config.home_joints is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "No home_joints configured for this robot",
            )

        logger.info("Opening gripper...")
        self._set_gripper_position(0.85)
        time.sleep(0.5)

        goal = self._model_positions_for_group(config.home_joints, group_id)
        if goal is None:
            return SkillResult.fail("INVALID_INPUT", "Select a planning group explicitly")
        logger.info("Planning motion to home position...")
        if not self.plan_to_joints(goal, group_id):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to home position")

        exec_result = self._preview_execute_wait()
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached home position")

    @skill
    def go_init(
        self, group_id: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its init position (captured at startup or set manually).

        The init position is the joint configuration the robot was in when the
        module first received joint state. It can be changed with set_init_joints().

        Args:
            group_id: Planning group to move; omission requires exactly one group.
        """
        init = self._init_joints
        if init is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "No init joints captured — robot may not have reported joint state yet",
            )

        # Lift if EE is low before moving to init
        lift = self._lift_if_low(group_id)
        if not lift.is_success():
            return lift

        # Move through a safe waypoint: 10cm above and 5cm in front of init pose.
        # This avoids direct paths through the workspace that could collide with objects.
        if self._world_monitor is not None:
            init_ee = self._world_monitor.get_ee_pose(joint_state=init)
            if init_ee is not None:
                wp = Pose(
                    Vector3(
                        init_ee.position.x + 0.05,
                        init_ee.position.y,
                        init_ee.position.z + 0.10,
                    ),
                    init_ee.orientation,
                )
                if self.plan_to_pose(wp, group_id):
                    wp_result = self._preview_execute_wait()
                    if not wp_result.is_success():
                        return wp_result
                else:
                    logger.warning("Safe waypoint unreachable, going directly to init")

        logger.info(
            f"Planning motion to init position [{', '.join(f'{j:.3f}' for j in init.position)}]..."
        )
        target = self._model_state_for_group(init, group_id)
        if target is None:
            return SkillResult.fail("INVALID_INPUT", "Select a planning group explicitly")
        if not self.plan_to_joints(target, group_id):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to init position")

        exec_result = self._preview_execute_wait()
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached init position")

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        execution_manager = getattr(self, "_execution_manager", None)
        if execution_manager is not None:
            cancellation = execution_manager.cancel()
            if not cancellation.safe:
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
