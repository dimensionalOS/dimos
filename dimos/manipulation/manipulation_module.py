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

import asyncio
from collections import Counter
from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import replace
import math
import threading
import time
import traceback
from typing import Any, Literal, TypeAlias

import numpy as np
from pydantic import Field, model_validator

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.execution_manager import PlanExecutionManager
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
from dimos.manipulation.planning.factory import WorldBackend, create_planning_specs, create_world
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    normalize_joint_target,
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
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.factory import create_manipulation_visualization
from dimos.manipulation.visualization.operator import ManipulationOperator
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.experimental.object import Object as DetObject
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ModelInfoValue: TypeAlias = (
    str | bool | float | list[str] | list[float] | list[PlanningGroupInfo] | None
)
ModelInfoPayload: TypeAlias = dict[str, ModelInfoValue]

ObstacleShape: TypeAlias = Literal["box", "sphere", "cylinder", "mesh"]

# One stable id for the mapped workspace: every map message is complete, so it
# replaces this obstacle rather than adding another.
VOXEL_MAP_OBSTACLE_ID = "mapping/voxel-map"

_SHAPE_TO_OBSTACLE_TYPE: dict[str, ObstacleType] = {
    "box": ObstacleType.BOX,
    "sphere": ObstacleType.SPHERE,
    "cylinder": ObstacleType.CYLINDER,
    "mesh": ObstacleType.MESH,
}


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
    # Fixed mount edges published alongside the robot's own TF, for rigs bolted
    # to a link the model already publishes -- an eye-in-hand camera, say. One
    # publisher for the whole chain: a second module publishing the mount at its
    # own rate leaves the two edges of one chain stamped up to a period apart.
    static_transforms: list[Transform] = Field(default_factory=list)
    # Frame the voxel_map port's clouds must already be expressed in.
    world_frame: str = "world"
    # Edge length of a voxel_map cell (meters). Must match the mapper's
    # voxel_size, or the octree will not line up with what was mapped.
    voxel_map_resolution: float = Field(default=0.05, gt=0.0)
    default_speed_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    linear_speed_scale: float = Field(default=0.5, gt=0.0, le=1.0)
    execution_timeout: float = Field(default=60.0, gt=0.0)
    # Coordinator joint name -> model joint name, so a twist base's odometry
    # joints (chassis/vx, ...) can feed the model's planar base joints.
    joint_state_aliases: dict[str, str] = Field(default_factory=dict)
    # Trajectory task -> the joints it drives. Empty means the joint trajectory
    # task drives every model joint; joints no task is bound to are preview-only.
    trajectory_tasks: dict[str, list[str]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate_trajectory_tasks(self) -> ManipulationModuleConfig:
        model_joints = set(self.model.joint_names)
        owners: dict[str, str] = {}
        for task, joints in self.trajectory_tasks.items():
            if unknown := sorted(set(joints) - model_joints):
                raise ValueError(
                    f"trajectory_tasks['{task}'] joints are not model joints: {unknown}"
                )
            for joint in joints:
                if joint in owners:
                    raise ValueError(
                        f"Joint '{joint}' is bound to both '{owners[joint]}' and '{task}'"
                    )
                owners[joint] = task
        return self

    @model_validator(mode="after")
    def _validate_joint_state_aliases(self) -> ManipulationModuleConfig:
        model_joints = set(self.model.joint_names)
        if unknown := sorted(set(self.joint_state_aliases.values()) - model_joints):
            raise ValueError(f"joint_state_aliases targets are not model joints: {unknown}")
        if shadowed := sorted(set(self.joint_state_aliases) & model_joints):
            raise ValueError(f"joint_state_aliases sources are model joints: {shadowed}")
        # Two sources for one model joint: whichever lands later in a JointState
        # message would win, so the same message could mean two different poses.
        targets = Counter(self.joint_state_aliases.values())
        if collided := sorted(name for name, count in targets.items() if count > 1):
            raise ValueError(f"joint_state_aliases targets are not unique: {collided}")
        return self


class ManipulationModule(Module):
    """Primitive manipulation RPCs; agent skills live in a separate adapter."""

    config: ManipulationModuleConfig
    _control_coordinator: ControlCoordinator

    # Input: Joint state from coordinator (for world sync)
    coordinator_joint_state: In[JointState]
    # Input: occupied cells of a mapped workspace, in the planning frame. Each
    # message is a complete map, so it replaces the obstacle rather than adding.
    voxel_map: In[PointCloud2]
    objects: In[list[DetObject]]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # State machine
        self._state = OperationStatus.IDLE
        self._lock = threading.Lock()
        # Serialize admission/dispatch/cancel; manager calls never hold _lock.
        self._command_lock = threading.RLock()
        self._error_message = ""
        self._state_epoch = 0
        self._started = False

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner: PlannerSpec | None = None
        self._kinematics: KinematicsSpec | None = None
        self._trajectory_parametrizer: TrajectoryParametrizerSpec | None = None

        # Canonical generated plan for the plan/preview/execute workflow.
        self._pending_plan: GeneratedPlan | None = None

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
        if self._started:
            logger.warning("ManipulationModule already running")
            return
        self._started = True
        try:
            super().start()

            # Observers may query execution state as planning and visualization start.
            with self._command_lock:
                # A stopped module may still own motion whose stop was not confirmed.
                # Keep its manager and diagnostic until cancellation confirms an outcome.
                with self._lock:
                    recovering = self._state is OperationStatus.EXECUTING
                if recovering:
                    self.cancel()
                else:
                    self._initialize_execution()
            self._initialize_planning()

            if self.coordinator_joint_state is not None:
                self.coordinator_joint_state.subscribe(self._on_joint_state)
                logger.info("Subscribed to coordinator_joint_state port")
            logger.info("ManipulationModule started")
        except BaseException:
            self._started = False
            raise

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
            logger.info("Floor obstacle added", z=fz)

        self._world_monitor.start_state_monitor()
        self._world_monitor.start_obstacle_monitor()

        if self._world_monitor.visualization is not None:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info("Visualization available", url=url)

        # Publish every pose-group tip plus any explicitly requested extra links.
        has_pose_group = any(group.has_pose_target for group in self.config.model.planning_groups)
        if has_pose_group or self.config.model.tf_extra_links:
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

            aliases = self.config.joint_state_aliases
            name_to_idx = {aliases.get(name, name): i for i, name in enumerate(msg.name)}
            names = self.config.model.joint_names
            canonical_names = [aliases.get(name, name) for name in msg.name]
            if (
                len(msg.name) != len(msg.position)
                or len(set(canonical_names)) != len(canonical_names)
                or not np.isfinite(msg.position).all()
                or not np.isfinite(msg.ts)
            ):
                logger.warning("Skipping malformed model state")
                return
            missing = [name for name in names if name not in name_to_idx]
            if missing:
                logger.warning("Skipping incomplete model state", missing_joints=missing)
                return
            indices = [name_to_idx[name] for name in names]
            state = JointState(
                ts=msg.ts,
                name=list(names),
                position=[msg.position[index] for index in indices],
                velocity=[msg.velocity[index] for index in indices]
                if len(msg.velocity) == len(msg.name)
                else [],
            )
            self._world_monitor.on_joint_state(state)
            if self._init_joints is None:
                self._init_joints = state

        except Exception as e:
            logger.error("Joint-state handling failed", error=str(e))
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
                # Use one measurement for every dynamic edge in this publication.
                state = self._world_monitor.current_model_joint_state()
                for group in self._world_monitor.planning_groups.list():
                    if not state.name or not group.has_pose_target or group.tip_link is None:
                        continue
                    ee_pose = self._world_monitor.get_group_ee_pose(group.id, state)
                    if ee_pose is not None and group.tip_link is not None:
                        ee_tf = Transform.from_pose(group.tip_link, ee_pose)
                        ee_tf.frame_id = "world"
                        transforms.append(ee_tf)
                for link_name in config.tf_extra_links:
                    if not state.name:
                        break
                    link_pose = self._world_monitor.get_link_pose(link_name, state)
                    if link_pose is not None:
                        link_tf = Transform.from_pose(link_name, link_pose)
                        link_tf.frame_id = "world"
                        transforms.append(link_tf)

                # Fixed mounts are valid at the same measurement time as their parent.
                stamp = state.ts if state.name else time.time()
                for static in self.config.static_transforms:
                    static.ts = stamp
                    transforms.append(static)

                if transforms:
                    self.tf.publish(TFMessage(*transforms))
            except Exception as e:
                logger.warning("TF publish failed", error=str(e))

            self._tf_stop_event.wait(period)

    @rpc
    def get_state(self) -> ManipulationSnapshot:
        """Return one snapshot containing every planning group."""
        self._refresh_execution_status()
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
                    gripper_position=self._get_group_gripper_position(),
                    joint_presets=self._group_joint_presets(group),
                )
        with self._lock:
            operation_status = self._state
            error = self._error_message or None
            has_pending_plan = self._state is OperationStatus.HAS_PLAN
        execution_status = (
            self._execution_manager.status
            if hasattr(self, "_execution_manager")
            else ExecutionStatus.IDLE
        )
        return ManipulationSnapshot(
            timestamp=time.time(),
            operation_status=operation_status,
            error=error,
            has_pending_plan=has_pending_plan,
            execution_status=execution_status,
            groups=groups,
        )

    def get_operation_status(self) -> OperationStatus:
        """Return the current operation status without collecting telemetry."""
        self._refresh_execution_status()
        with self._lock:
            return self._state

    def _refresh_execution_status(self) -> None:
        """Poll active nonblocking execution once, without waiting for motion."""
        with self._lock:
            executing = self._state is OperationStatus.EXECUTING
        if executing:
            self.wait_for_execution(timeout=0.0)

    def get_error(self) -> str:
        """Get last error message.

        Returns:
            Error message or empty string
        """
        with self._lock:
            return self._error_message

    @rpc
    def cancel(self) -> ExecutionResult:
        """Cancel current work without latching a module failure state."""
        with self._command_lock:
            epoch, previous = self._invalidate()
            self._dismiss_preview()
            try:
                result = self._execution_manager.cancel()
            except Exception as exc:
                logger.exception("Failed to cancel manipulation")
                result = ExecutionResult(ExecutionStatus.UNCERTAIN, str(exc))
            if (
                previous is OperationStatus.PLANNING
                and result.status is ExecutionStatus.NO_EXECUTION
            ):
                result = ExecutionResult(ExecutionStatus.ABORTED, "Planning cancelled")
            self._apply_execution_result(result, epoch, expected=None)
            return result

    @rpc
    def show_grasp_proposals(self, candidates: GraspCandidateArray) -> None:
        """Display the ranked proposals. The grasp module owns no visualizer."""
        if self._world_monitor is not None:
            self._world_monitor.show_grasp_proposals(candidates)

    def get_current_joints(self) -> list[float] | None:
        """Get the complete canonical model joint positions."""
        if self._world_monitor:
            state = self._world_monitor.get_current_joint_state()
            if state is not None:
                return list(state.position)
        return None

    def get_ee_pose(self, group_id: PlanningGroupID | None = None) -> Pose | None:
        """Get a planning group's current tip pose."""
        if self._world_monitor:
            try:
                selected = group_id or self._require_unique_pose_group_id()
                return self._world_monitor.get_group_ee_pose(selected)
            except ValueError as exc:
                logger.warning("End-effector pose unavailable", error=str(exc))
                return None
        return None

    def is_collision_free(self, joints: list[float]) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check
        """
        if self._world_monitor:
            joint_state = JointState(name=self.config.model.joint_names, position=joints)
            return self._world_monitor.is_state_valid(joint_state)
        return False

    @contextmanager
    def _planning_request(
        self, group_ids: tuple[PlanningGroupID, ...], speed_scale: float | None
    ) -> Iterator[tuple[int, float] | None]:
        """Admit one planning request and finish it on every exit."""
        error = self._planning_input_error(group_ids, speed_scale)
        if error:
            yield None
            return
        resolved_speed = self.config.default_speed_scale if speed_scale is None else speed_scale
        with self._command_lock:
            self._refresh_execution_status()
            epoch = self._begin(OperationStatus.PLANNING)
        if epoch is None:
            yield None
            return
        try:
            self._dismiss_preview()
            yield epoch, float(resolved_speed)
        except Exception as exc:
            logger.exception("Planning request failed")
            self._finish(epoch, f"Planning failed: {exc}")
        finally:
            self._finish(epoch, "Planning failed")

    def _planning_input_error(
        self, group_ids: tuple[PlanningGroupID, ...], speed_scale: float | None
    ) -> str:
        """Validate without overwriting a saved workflow diagnostic."""
        if self._world_monitor is None:
            return "Planning not initialized"
        speed = self.config.default_speed_scale if speed_scale is None else speed_scale
        if not math.isfinite(speed) or not 0.0 < speed <= 1.0:
            return "speed_scale must be finite, > 0, and <= 1"
        try:
            self._world_monitor.planning_groups.select(group_ids)
        except (KeyError, ValueError) as exc:
            return str(exc)
        return ""

    def _joint_goal(self, targets: Mapping[PlanningGroupID, JointState]) -> JointState:
        assert self._world_monitor is not None
        names: list[str] = []
        positions: list[float] = []
        for group_id, target in targets.items():
            group = self._world_monitor.planning_groups.get(group_id)
            normalized = normalize_joint_target(group, target)
            names.extend(normalized.name)
            positions.extend(normalized.position)
        return JointState(name=names, position=positions)

    def _planning_failure(self) -> PlanResult:
        with self._lock:
            if not self._started:
                message = "Manipulation is stopped"
            elif self._state is not OperationStatus.IDLE:
                message = f"Cannot plan while manipulation is {self._state.name}"
            else:
                message = self._error_message or "Planning failed"
        return PlanResult(PlanStatus.FAILED, message)

    def _begin(self, phase: OperationStatus) -> int | None:
        """Admit a workflow under _command_lock without mutating rejected requests."""
        with self._lock:
            allowed = (
                {OperationStatus.IDLE, OperationStatus.HAS_PLAN}
                if phase is OperationStatus.PLANNING
                else {OperationStatus.HAS_PLAN}
            )
            if not self._started or self._state not in allowed:
                return None
            self._state_epoch += 1
            self._pending_plan = None
            self._error_message = ""
            self._state = phase
            return self._state_epoch

    def _invalidate(self, *, planning_only: bool = False) -> tuple[int, OperationStatus]:
        """Clear pending data and invalidate affected callbacks under _command_lock."""
        with self._lock:
            previous = self._state
            self._pending_plan = None
            if not planning_only or previous is OperationStatus.PLANNING:
                self._state_epoch += 1
                if planning_only:
                    self._state = OperationStatus.IDLE
            if planning_only and previous is OperationStatus.HAS_PLAN:
                self._state = OperationStatus.IDLE
            return self._state_epoch, previous

    def _finish(
        self,
        epoch: int,
        message: str = "",
        plan: GeneratedPlan | None = None,
        *,
        active: bool = False,
        expected: OperationStatus | None = OperationStatus.PLANNING,
    ) -> bool:
        """Commit only the current workflow's data and common finish transition."""
        with self._lock:
            if epoch != self._state_epoch or (expected is not None and self._state is not expected):
                return False
            self._pending_plan = plan
            self._error_message = message
            self._state = (
                OperationStatus.EXECUTING
                if active
                else OperationStatus.HAS_PLAN
                if plan is not None
                else OperationStatus.IDLE
            )
            return True

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
            self._finish(planning_epoch, f"Failed to resolve planning groups: {exc}")
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
            self._finish(planning_epoch, f"Failed to materialize plan: {exc}")
            return None
        return plan if self._finish(planning_epoch, plan=plan) else None

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
            self._finish(planning_epoch, f"Planning failed: {result.status.name}{detail}")
            return None

        logger.info("Path generated", waypoint_count=len(result.path), group_ids=group_ids)
        return self._store_generated_plan(group_ids, result, planning_epoch, speed_scale)

    def _dismiss_preview(self) -> None:
        """Hide the preview ghost if the world supports it."""
        if self._world_monitor is None:
            return
        self._world_monitor.cancel_preview_animation()

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
    def plan_to_joints(
        self,
        targets: Mapping[PlanningGroupID, JointState],
        speed_scale: float | None = None,
    ) -> PlanResult:
        """Plan one synchronized joint target set without moving hardware."""
        if not targets:
            return PlanResult(PlanStatus.INVALID_TARGET, "At least one target is required")
        error = self._planning_input_error(tuple(targets), speed_scale)
        if error:
            return PlanResult(PlanStatus.INVALID_TARGET, error)
        try:
            self._joint_goal(targets)
        except (KeyError, ValueError) as exc:
            return PlanResult(PlanStatus.INVALID_TARGET, str(exc))
        plan = self.generate_plan_to_joint_targets(targets, speed_scale=speed_scale)
        if plan is None:
            return self._planning_failure()
        return PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)

    @rpc
    def plan_to_poses(
        self,
        targets: Mapping[PlanningGroupID, PoseStamped],
        speed_scale: float | None = None,
    ) -> PlanResult:
        """Plan one synchronized pose target set without moving hardware."""
        if not targets:
            return PlanResult(PlanStatus.INVALID_TARGET, "At least one target is required")
        error = self._planning_input_error(tuple(targets), speed_scale)
        if error:
            return PlanResult(PlanStatus.INVALID_TARGET, error)
        plan = self.generate_plan_to_pose_targets(targets, speed_scale=speed_scale)
        if plan is None:
            return self._planning_failure()
        return PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)

    def generate_plan_to_joint_targets(
        self,
        joint_targets: Mapping[PlanningGroupID, JointState],
        speed_scale: float | None = None,
    ) -> GeneratedPlan | None:
        """Plan to joint targets and return the exact stored GeneratedPlan."""
        if self._world_monitor is None or self._planner is None:
            return None
        if not joint_targets:
            return None

        group_ids = tuple(joint_targets)
        try:
            goal = self._joint_goal(joint_targets)
        except (KeyError, ValueError) as exc:
            logger.warning("Invalid joint target", error=str(exc))
            return None

        with self._planning_request(group_ids, speed_scale) as planning:
            if planning is None:
                return None
            planning_epoch, resolved_speed_scale = planning

            resolved = self._resolve_group_plan_start(group_ids, planning_epoch)
            if resolved is None:
                return None
            _selection, start = resolved

            return self._plan_selected_path(
                group_ids, start, goal, planning_epoch, resolved_speed_scale
            )

    def generate_plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID, Pose],
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        speed_scale: float | None = None,
    ) -> GeneratedPlan | None:
        """Plan to pose targets and return the exact stored GeneratedPlan."""
        if self._world_monitor is None or self._kinematics is None:
            return None
        if not pose_targets:
            return None
        stamped_targets = {
            group_id: PoseStamped(
                frame_id="world",
                position=pose.position,
                orientation=pose.orientation,
            )
            for group_id, pose in pose_targets.items()
        }
        auxiliary_ids = tuple(auxiliary_groups)
        group_ids = tuple(dict.fromkeys((*stamped_targets.keys(), *auxiliary_ids)))
        with self._planning_request(group_ids, speed_scale) as planning:
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
                self._finish(planning_epoch, f"IK failed: {ik.status.name}{detail}")
                return None
            logger.info("IK solved", position_error=ik.position_error)
            return self._plan_selected_path(
                group_ids, start, ik.joint_state, planning_epoch, resolved_speed_scale
            )

    def generate_cartesian_plan(
        self,
        targets: Mapping[PlanningGroupID, CartesianTarget],
        config: CartesianPathConfig,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        speed_scale: float | None = None,
        check_collision: bool = True,
    ) -> GeneratedPlan | None:
        """Generate and store a timed Cartesian plan through PlannerSpec."""
        if self._world_monitor is None or self._planner is None:
            return None
        if not targets:
            return None
        auxiliary_ids = tuple(auxiliary_groups)
        group_ids = tuple((*targets.keys(), *auxiliary_ids))
        with self._planning_request(group_ids, speed_scale) as planning:
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
                targets=targets,
                config=scaled_config,
                auxiliary_groups=auxiliary_ids,
                check_collision=check_collision,
            )
            if not result.is_success():
                detail = f": {result.message}" if result.message else ""
                self._finish(
                    planning_epoch, f"Cartesian planning failed: {result.status.name}{detail}"
                )
                return None
            return self._store_generated_plan(
                group_ids, result, planning_epoch, resolved_speed_scale
            )

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
        error = self._planning_input_error((group.id,), resolved_speed)
        if error:
            return MoveResult(
                PlanResult(PlanStatus.INVALID_TARGET, error), None, delta, check_collision
            )
        relative = Transform(
            translation=Vector3(*delta),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        plan = self.generate_cartesian_plan(
            {group.id: (Transform.identity(), relative)},
            CartesianPathConfig(),
            speed_scale=resolved_speed,
            check_collision=check_collision,
        )
        if plan is None:
            plan_result = self._planning_failure()
            return MoveResult(plan_result, None, delta, check_collision)
        plan_result = PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)
        execution = self.execute(blocking=blocking, timeout=timeout, plan_id=plan.plan_id)
        return MoveResult(plan_result, execution, delta, check_collision)

    @rpc
    def preview_plan(
        self,
        plan: GeneratedPlan | None = None,
        duration: float | None = None,
    ) -> CommandResult:
        """Preview a complete generated plan in the visualizer."""
        plan = plan or self._pending_plan
        if plan is None or not plan.path:
            return CommandResult(CommandStatus.REJECTED, "No generated plan to preview")
        if self._world_monitor is None:
            return CommandResult(CommandStatus.FAILED, "Planning not initialized")
        self._world_monitor.animate_trajectory(plan.trajectory, duration)
        return CommandResult(CommandStatus.SUCCEEDED, "Preview requested")

    def has_planned_path(self) -> bool:
        """Check if there's a planned path ready.

        Returns:
            True if a path is planned and ready
        """
        with self._lock:
            return self._state is OperationStatus.HAS_PLAN

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
    def clear_planned_path(self) -> CommandResult:
        """Discard the pending plan and its preview without cancelling active execution."""
        with self._command_lock:
            self._invalidate(planning_only=True)
            self._dismiss_preview()
        return CommandResult(CommandStatus.SUCCEEDED, "Pending plan cleared")

    @rpc
    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]:
        """Return public planning-group capabilities."""
        if self._world_monitor is None:
            return ()
        has_gripper = self.config.model.gripper_hardware_id is not None
        return tuple(
            PlanningGroupInfo(
                id=group.id,
                joint_names=group.joint_names,
                base_frame=group.base_link,
                tip_frame=group.tip_link,
                has_gripper=has_gripper,
            )
            for group in self._world_monitor.planning_groups.list()
        )

    def _group_joint_presets(self, group: PlanningGroup) -> dict[str, JointState]:
        config = self.config.model
        presets: dict[str, JointState] = {}

        def selected(state: JointState) -> JointState:
            positions = dict(zip(state.name, state.position, strict=True))
            return JointState(
                name=list(group.joint_names),
                position=[positions[name] for name in group.joint_names],
            )

        if config.home_joints is not None:
            presets["home"] = selected(
                JointState(name=config.joint_names, position=config.home_joints)
            )
        if self._init_joints is not None:
            presets["init"] = selected(self._init_joints)
        return presets

    def _get_group_gripper_position(self) -> float | None:
        hardware_id = self.config.model.gripper_hardware_id
        if hardware_id is None:
            return None
        values = self._control_coordinator.task_invoke(
            f"{hardware_id}_gripper", "get_normalized", {}
        )
        return float(values[0]) if values else None

    def get_current_joint_state(self) -> JointState | None:
        """Return the complete canonical model joint state."""
        if self._world_monitor is None:
            return None
        return self._world_monitor.get_current_joint_state()

    def get_model_info(self) -> ModelInfoPayload:
        """Get information about the configured logical robot model."""
        config = self.config.model
        planning_groups = self.list_planning_groups()
        return {
            "joint_names": config.joint_names,
            "planning_groups": list(planning_groups),
            "base_link": config.base_link,
            "home_joints": config.home_joints,
            "pre_grasp_offset": config.pre_grasp_offset,
            "init_joints": list(self._init_joints.position)
            if self._init_joints is not None
            else None,
        }

    def get_model_config(self) -> RobotModelConfig:
        """Return the configured model for in-process visualization adapters."""
        return self.config.model

    def get_init_joints(self) -> JointState | None:
        """Get the init joint state captured at startup or set manually."""
        return self._init_joints

    def set_init_joints(self, joint_state: JointState) -> bool:
        """Set the init joint state.

        Args:
            joint_state: New init joint state (names + positions)
        """
        self._init_joints = joint_state
        logger.info("Init joints set", positions=joint_state.position)
        return True

    def set_init_joints_to_current(self) -> bool:
        """Set init joints to the current joint positions."""
        if self._world_monitor is None:
            return False
        current = self._world_monitor.get_current_joint_state()
        if current is None:
            logger.error("Cannot capture init joints — no current joint state")
            return False
        self._init_joints = current
        return True

    def _initialize_execution(self) -> None:
        """Initialize coordinator access and planned execution policy."""
        self._execution_manager = PlanExecutionManager(
            joint_names=self.config.model.joint_names,
            coordinator=self._control_coordinator,
            default_timeout=self.config.execution_timeout,
            bindings=self.config.trajectory_tasks,
        )

    @rpc
    def execute(
        self, blocking: bool = True, timeout: float | None = None, *, plan_id: str | None = None
    ) -> ExecutionResult:
        """Consume a pending plan; blocking waits never hold module admission."""
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            return ExecutionResult(ExecutionStatus.REJECTED, "timeout must be finite and >= 0")
        with self._command_lock:
            result, epoch = self._dispatch_plan(self._pending_plan, plan_id)
        if blocking and result.status is ExecutionStatus.ACCEPTED:
            return self._wait_for_execution(epoch, timeout)
        return result

    def _dispatch_plan(
        self,
        target_plan: GeneratedPlan | None,
        plan_id: str | None,
    ) -> tuple[ExecutionResult, int | None]:
        """Admit and dispatch under _command_lock, shared by RPC and visualization."""
        self._refresh_execution_status()
        with self._lock:
            if target_plan is None:
                return ExecutionResult(ExecutionStatus.NO_PLAN, "No pending plan"), None
            if (
                self._pending_plan is None
                or self._pending_plan.plan_id != target_plan.plan_id
                or (plan_id is not None and target_plan.plan_id != plan_id)
            ):
                return ExecutionResult(ExecutionStatus.REJECTED, "Pending plan was replaced"), None
            bound = {
                joint for joints in self.config.trajectory_tasks.values() for joint in joints
            } or set(self.config.model.joint_names)
            unowned = sorted(set(target_plan.trajectory.joint_names) - bound)
            if unowned:
                return ExecutionResult(
                    ExecutionStatus.REJECTED,
                    f"No trajectory task is bound to {unowned}; those joints support planning "
                    "and preview only",
                ), None
        epoch = self._begin(OperationStatus.EXECUTING)
        if epoch is None:
            return ExecutionResult(
                ExecutionStatus.REJECTED, "Manipulation is busy or stopped"
            ), None
        try:
            result = self._execution_manager.execute(target_plan, blocking=False)
        except Exception as exc:
            logger.exception("Failed to dispatch generated plan")
            result = ExecutionResult(ExecutionStatus.UNCERTAIN, str(exc))
        self._apply_execution_result(result, epoch)
        return result, epoch

    @rpc
    def wait_for_execution(self, timeout: float | None = None) -> ExecutionResult:
        """Wait without letting a historical result overwrite another workflow."""
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            return ExecutionResult(ExecutionStatus.REJECTED, "timeout must be finite and >= 0")
        with self._command_lock, self._lock:
            epoch = self._state_epoch if self._state is OperationStatus.EXECUTING else None
        return self._wait_for_execution(epoch, timeout)

    def _wait_for_execution(self, epoch: int | None, timeout: float | None) -> ExecutionResult:
        try:
            result = self._execution_manager.wait(timeout)
        except Exception as exc:
            logger.exception("Failed to wait for execution")
            result = ExecutionResult(ExecutionStatus.UNCERTAIN, str(exc))
        if result.status is ExecutionStatus.TIMED_OUT:
            result = replace(
                result,
                message=f"{result.message}. Motion may still be in progress; timeout does not stop execution.",
            )
        if epoch is not None:
            self._apply_execution_result(result, epoch)
        return result

    def _apply_execution_result(
        self,
        result: ExecutionResult,
        epoch: int,
        *,
        expected: OperationStatus | None = OperationStatus.EXECUTING,
    ) -> None:
        self._finish(
            epoch,
            result.message,
            expected=expected,
            active=result.status
            in {
                ExecutionStatus.ACCEPTED,
                ExecutionStatus.EXECUTING,
                ExecutionStatus.TIMED_OUT,
                ExecutionStatus.UNCERTAIN,
                ExecutionStatus.FAULT,
            },
        )

    def _execute_generated_plan(self, plan: GeneratedPlan) -> bool:
        """Use the same admission rules for the visualization operator."""
        with self._command_lock:
            result, _ = self._dispatch_plan(plan, plan.plan_id)
        if result.status is not ExecutionStatus.ACCEPTED:
            logger.warning(
                "Viser plan execution rejected",
                plan_id=plan.plan_id,
                status=result.status.name,
                reason=result.message,
            )
        return result.status is ExecutionStatus.ACCEPTED

    @property
    def world_monitor(self) -> WorldMonitor | None:
        """Access the world monitor for advanced obstacle/world operations."""
        return self._world_monitor

    async def handle_objects(self, objects: list[DetObject]) -> None:
        """Cache the latest perception objects for an explicit obstacle refresh."""
        if self._world_monitor is not None:
            self._world_monitor.on_objects(objects)

    @rpc
    def refresh_obstacles(self, min_duration: float = 0.0) -> int:
        """Sync cached perception objects into the planning world."""
        if self._world_monitor is None:
            return 0
        return len(self._world_monitor.refresh_obstacles(min_duration))

    @rpc
    def get_obstacles(self) -> dict[str, PoseStamped]:
        """Return planning-world obstacle poses keyed by obstacle name."""
        if self._world_monitor is None:
            return {}
        return {
            obstacle.name: obstacle.pose for obstacle in self._world_monitor.world.get_obstacles()
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
        if not self._world_monitor:
            return ""

        obstacle_type = _SHAPE_TO_OBSTACLE_TYPE.get(shape)
        if obstacle_type is None:
            logger.warning("Unknown obstacle shape", shape=shape)
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

    async def handle_voxel_map(self, cloud: PointCloud2) -> None:
        """Replace the mapped workspace, held as one octree obstacle.

        Building the octree and handing it to the planning world takes the world
        lock, so it runs off the transport thread. The port dispatcher keeps only
        the newest map, which is what we want: each message is a complete map,
        and a stale one has nothing to contribute.
        """
        await asyncio.to_thread(self._apply_voxel_map, cloud)

    def _apply_voxel_map(self, cloud: PointCloud2) -> None:
        if self._world_monitor is None:
            return
        frame = self.config.world_frame
        if cloud.frame_id != frame:
            # The points are metric positions in the planning frame. Registering
            # them through a guessed transform would invent geometry.
            logger.warning(
                "Voxel map is in frame '%s', not the planning frame '%s'; dropped a map.",
                cloud.frame_id,
                frame,
            )
            return

        points = cloud.points_f32()
        if not len(points):
            # An empty map is how a mapper says the space it owns is now clear.
            self._world_monitor.remove_obstacle(VOXEL_MAP_OBSTACLE_ID)
            return
        if not np.isfinite(points).all():
            logger.warning("Voxel map contains non-finite points; dropped a map.")
            return

        obstacle = Obstacle(
            name=VOXEL_MAP_OBSTACLE_ID,
            obstacle_type=ObstacleType.OCTREE,
            pose=PoseStamped(frame_id=frame),
            points=points,
            octree_resolution=self.config.voxel_map_resolution,
        )
        try:
            installed = self._world_monitor.update_obstacle(obstacle) or bool(
                self._world_monitor.add_obstacle(obstacle)
            )
        except ValueError:
            logger.exception("Rejected voxel map; planning continues on the previous one")
            return
        if not installed:
            # Silence here would leave the planner checking against a map that
            # no longer describes the workspace, with nothing to say so.
            logger.warning(
                "Planning world refused the voxel map; planning continues on the previous one"
            )

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
        hardware_id = self.config.model.gripper_hardware_id
        assert hardware_id is not None
        if self._control_coordinator.task_invoke(
            f"{hardware_id}_gripper",
            "set_normalized",
            {"values": [float(position)]},
        ):
            return CommandResult(CommandStatus.SUCCEEDED, f"Gripper set to {position:.0%} open")
        return CommandResult(CommandStatus.FAILED, "Failed to set gripper position")

    def _resolve_pose_group(
        self, planning_group: PlanningGroupID | None
    ) -> PlanningGroup | CommandResult:
        return self._resolve_group_with_capability(planning_group, "pose")

    def _resolve_gripper_group(
        self, planning_group: PlanningGroupID | None
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
            return self.config.model.gripper_hardware_id is not None

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

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        with self._command_lock:
            with self._lock:
                self._started = False
            execution_manager = getattr(self, "_execution_manager", None)
            if execution_manager is not None:
                cancellation = self.cancel()
                if cancellation.status in {
                    ExecutionStatus.UNCERTAIN,
                    ExecutionStatus.FAULT,
                    ExecutionStatus.TIMED_OUT,
                    ExecutionStatus.EXECUTING,
                }:
                    logger.error(
                        "Shutdown could not confirm coordinator trajectory safety: %s",
                        cancellation.message,
                    )
                execution_manager.close()
            else:
                epoch, _ = self._invalidate()
                self._finish(epoch, self.get_error(), expected=None)

        # Stop TF thread
        if self._tf_thread is not None:
            self._tf_stop_event.set()
            self._tf_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._tf_thread = None

        # Stop world monitor (includes visualization thread)
        if self._world_monitor is not None:
            self._world_monitor.stop_all_monitors()

        super().stop()
        self._started = False
