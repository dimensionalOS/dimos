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

"""RoboPlan-backed manipulation world implementation.

This adapter imports RoboPlan at module load time. The factory imports this module
only when the RoboPlan backend is requested, so default planning paths do not need
the optional dependency installed.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, field, replace
import importlib
from itertools import combinations, pairwise
from math import atan2, ceil, sqrt
from pathlib import Path
import tempfile
import time
from typing import TYPE_CHECKING, Any, cast
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

try:
    import roboplan.core as roboplan_core
    import roboplan.rrt as roboplan_rrt
except ImportError as exc:
    raise ImportError(
        "RoboPlanWorld requires the optional roboplan dependency. "
        "Install the manipulation extra before selecting the roboplan backend."
    ) from exc

try:
    roboplan_simple_ik: Any
    import roboplan.simple_ik as roboplan_simple_ik
except ImportError:
    roboplan_simple_ik = None

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.groups.utils import joint_target_to_global_names
from dimos.manipulation.planning.kinematics.config import RoboPlanKinematicsConfig
from dimos.manipulation.planning.spec.config import (
    RobotModelConfig,
    TrajectoryParametrizationConfig,
)
from dimos.manipulation.planning.spec.enums import (
    IKStatus,
    ObstacleType,
    ParametrizationStatus,
    PlanningStatus,
)
from dimos.manipulation.planning.spec.models import (
    CartesianDelta,
    CartesianPathMode,
    GeneratedPlan,
    GeneratedTrajectory,
    IKResult,
    LinearTcpPathConstraint,
    Obstacle,
    PlanningGroupID,
    PlanningResult,
    WorldRobotID,
)
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName
    from dimos.manipulation.planning.spec.protocols import WorldSpec

logger = setup_logger()

_NATIVE_NAME_SEPARATOR = "__"
_COMPOSITE_GROUP_PREFIX = "_dimos_composite"
_CARTESIAN_POSITION_TOLERANCE = 1e-3
_CARTESIAN_ORIENTATION_TOLERANCE = 1e-2
_LINEAR_CARTESIAN_TRANSLATION_STEP = 0.01
_LINEAR_CARTESIAN_ROTATION_STEP = 0.05


@dataclass
class _RoboPlanRobotData:
    robot_id: WorldRobotID
    config: RobotModelConfig
    lower_limits: NDArray[np.float64] | None
    upper_limits: NDArray[np.float64] | None
    model_handle: Any = None


@dataclass(frozen=True)
class _NativeNameMap:
    robot_name: RobotName
    joint_names: dict[str, str]
    link_names: dict[str, str]

    def joint(self, local_name: str) -> str:
        try:
            return self.joint_names[local_name]
        except KeyError as exc:
            raise KeyError(
                f"No RoboPlan-native joint mapping for '{self.robot_name}/{local_name}'"
            ) from exc

    def link(self, local_name: str) -> str:
        try:
            return self.link_names[local_name]
        except KeyError as exc:
            raise KeyError(
                f"No RoboPlan-native link mapping for '{self.robot_name}/{local_name}'"
            ) from exc


@dataclass(frozen=True)
class _RoboPlanGroupData:
    group_ids: tuple[PlanningGroupID, ...]
    group_name: str
    native_joint_names: tuple[str, ...]
    native_to_global_joint_name: dict[str, str]


@dataclass
class RoboPlanContext:
    """DimOS context wrapper for RoboPlan world state."""

    q_by_robot: dict[WorldRobotID, NDArray[np.float64]] = field(default_factory=dict)


class RoboPlanWorld:
    """WorldSpec implementation backed by RoboPlan scene and collision queries."""

    def __init__(
        self,
        enable_viz: bool = False,
        max_generated_composite_groups: int = 128,
        selected_start_tolerance: float = 1e-6,
        **_: Any,
    ) -> None:
        if max_generated_composite_groups < 0:
            raise ValueError("max_generated_composite_groups must be non-negative")
        if selected_start_tolerance < 0.0:
            raise ValueError("selected_start_tolerance must be non-negative")
        self._scene: Any | None = None
        self._enable_viz = enable_viz
        self._max_generated_composite_groups = max_generated_composite_groups
        self._selected_start_tolerance = selected_start_tolerance
        self._trajectory_parametrization_config = TrajectoryParametrizationConfig(
            backend="roboplan"
        )
        if enable_viz:
            logger.warning("RoboPlanWorld does not currently provide manipulation visualization")

        self._robots: dict[WorldRobotID, _RoboPlanRobotData] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._obstacle_handles: dict[str, Any] = {}
        self._robot_counter = 0
        self._finalized = False
        self._live_context = RoboPlanContext()
        self._srdf_tempdirs: list[tempfile.TemporaryDirectory[str]] = []
        self._planning_groups = PlanningGroupRegistry()
        self._robot_ids_by_name: dict[RobotName, WorldRobotID] = {}
        self._roboplan_native_joint_names: dict[PlanningGroupID, tuple[str, ...]] = {}
        self._native_names_by_robot: dict[RobotName, _NativeNameMap] = {}
        self._movable_native_joint_names_by_robot: dict[RobotName, tuple[str, ...]] = {}
        self._roboplan_groups_by_selection: dict[
            frozenset[PlanningGroupID], _RoboPlanGroupData
        ] = {}
        self._full_native_joint_names: tuple[str, ...] = ()
        self._uses_composite_model = False
        self._kinematics_config = RoboPlanKinematicsConfig()

    # Robot Management

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Register a supported robot model for later RoboPlan scene construction."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")
        if config.name in self._robot_ids_by_name:
            raise ValueError(f"Robot name '{config.name}' is already registered")
        if not Path(config.model_path).exists():
            raise FileNotFoundError(f"Robot model not found: {Path(config.model_path).resolve()}")

        self._validate_robot_config(config)
        self._robot_counter += 1
        robot_id = f"robot_{self._robot_counter}"
        model_handle = config.name
        limits = self._configured_joint_limits(config)
        lower, upper = limits if limits is not None else (None, None)
        self._robots[robot_id] = _RoboPlanRobotData(
            robot_id=robot_id,
            config=config,
            lower_limits=lower,
            upper_limits=upper,
            model_handle=model_handle,
        )
        self._live_context.q_by_robot[robot_id] = np.zeros(
            len(config.joint_names), dtype=np.float64
        )
        self._planning_groups.add_robot(config)
        self._robot_ids_by_name[config.name] = robot_id
        logger.info(f"Registered RoboPlan robot '{robot_id}' ({config.name})")
        return robot_id

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs in the world."""
        return list(self._robots.keys())

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get robot configuration by ID."""
        return self._get_robot(robot_id).config

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits in DimOS joint order."""
        robot = self._get_robot(robot_id)
        if robot.lower_limits is None or robot.upper_limits is None:
            raise RuntimeError("RoboPlan joint limits are unavailable until world is finalized")
        return robot.lower_limits.copy(), robot.upper_limits.copy()

    # Obstacle Management

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add a supported obstacle to the RoboPlan scene."""
        self._require_finalized()
        obstacle_id = obstacle.name
        if obstacle_id in self._obstacles:
            return obstacle_id
        handle = self._add_obstacle_to_scene(obstacle, obstacle_id)
        self._obstacles[obstacle_id] = obstacle
        self._obstacle_handles[obstacle_id] = handle
        return obstacle_id

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the RoboPlan scene."""
        self._require_finalized()
        if obstacle_id not in self._obstacles:
            return False
        handle = self._obstacle_handles.get(obstacle_id, obstacle_id)
        scene = self._require_scene()
        scene.removeGeometry(handle)
        del self._obstacles[obstacle_id]
        self._obstacle_handles.pop(obstacle_id, None)
        return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update an obstacle pose and invalidate collision scratch."""
        self._require_finalized()
        if obstacle_id not in self._obstacles:
            return False
        handle = self._obstacle_handles.get(obstacle_id, obstacle_id)
        scene = self._require_scene()
        scene.updateGeometryPlacement(handle, pose_to_matrix(pose))
        self._obstacles[obstacle_id] = replace(self._obstacles[obstacle_id], pose=pose)
        return True

    def clear_obstacles(self) -> None:
        """Remove all tracked obstacles."""
        for obstacle_id in list(self._obstacles.keys()):
            self.remove_obstacle(obstacle_id)

    def get_obstacles(self) -> list[Obstacle]:
        """Get all obstacles currently tracked by DimOS."""
        return list(self._obstacles.values())

    # Lifecycle

    def finalize(self) -> None:
        """Construct the RoboPlan scene and mark it ready for planning queries.

        RoboPlan Python bindings construct a query-ready Scene directly; v0.4.0
        exposes no Scene.finalize() lifecycle method.
        """
        if self._finalized:
            return
        if not self._robots:
            raise RuntimeError("Cannot finalize RoboPlanWorld before adding a robot")
        self._uses_composite_model = len(self._robots) > 1
        self._scene = self._create_scene()
        native_joint_names = self._validate_planning_group_metadata(self._scene)
        for robot_id, robot in tuple(self._robots.items()):
            lower, upper = self._extract_joint_limits(robot.config, robot.model_handle)
            self._robots[robot_id] = replace(
                robot,
                lower_limits=lower,
                upper_limits=upper,
            )
        self._roboplan_native_joint_names.update(native_joint_names)
        self._finalized = True

    @property
    def is_finalized(self) -> bool:
        """Check whether the scene is finalized."""
        return self._finalized

    # Context Management

    def get_live_context(self) -> RoboPlanContext:
        """Get the live context that mirrors robot state."""
        self._require_finalized()
        return self._live_context

    @contextmanager
    def scratch_context(self) -> Generator[RoboPlanContext, None, None]:
        """Create a per-consumer context with independent collision scratch."""
        self._require_finalized()
        ctx = RoboPlanContext(
            q_by_robot={robot_id: q.copy() for robot_id, q in self._live_context.q_by_robot.items()}
        )
        yield ctx

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live context from a driver joint-state message."""
        if not self._finalized:
            return
        self.set_joint_state(self._live_context, robot_id, joint_state)

    # State Operations

    def set_joint_state(
        self, ctx: RoboPlanContext, robot_id: WorldRobotID, joint_state: JointState
    ) -> None:
        """Set robot joint state in a context."""
        self._require_finalized()
        ctx.q_by_robot[robot_id] = self._joint_state_to_q(robot_id, joint_state)

    def get_joint_state(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> JointState:
        """Get robot joint state from a context."""
        robot = self._get_robot(robot_id)
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            q = np.zeros(len(robot.config.joint_names), dtype=np.float64)
        return JointState(name=robot.config.joint_names, position=q.astype(float).tolist())

    # Collision Checking

    def is_collision_free(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> bool:
        """Check if the robot configuration in a context is collision-free."""
        self._require_finalized()
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        return not self._has_collisions(robot_id, q, ctx)

    def get_min_distance(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> float:
        """Get minimum signed distance.

        RoboPlan signed-distance semantics are not verified yet, so do not return
        a misleading approximation.
        """
        raise NotImplementedError("RoboPlanWorld.get_min_distance is not implemented")

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check a joint state using a scratch collision context."""
        with self.scratch_context() as ctx:
            self.set_joint_state(ctx, robot_id, joint_state)
            return self.is_collision_free(ctx, robot_id)

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check if an interpolated edge is collision-free."""
        q_start = self._joint_state_to_q(robot_id, start)
        q_end = self._joint_state_to_q(robot_id, end)
        with self.scratch_context() as ctx:
            return not self._call_path_collision_checker(ctx, robot_id, q_start, q_end, step_size)

    # Forward Kinematics

    def get_group_ee_pose(self, ctx: RoboPlanContext, group_id: PlanningGroupID) -> PoseStamped:
        """Get pose for a planning group's target frame."""
        self._require_finalized()
        group, robot_id, robot, _native_joint_names = self._resolve_group(group_id)
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group_id}' has no pose target frame")
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        scene = self._require_scene()
        result = scene.forwardKinematics(
            self._full_scene_q(ctx), self._native_link_name(robot.config, group.tip_link), ""
        )
        mat = np.asarray(result, dtype=np.float64)
        pose = matrix_to_pose(mat)
        return PoseStamped(
            frame_id="world",
            position=[pose.position.x, pose.position.y, pose.position.z],
            orientation=[
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        )

    def get_link_pose(
        self, ctx: RoboPlanContext, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        """Get link pose as a 4x4 homogeneous transform."""
        self._require_finalized()
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        scene = self._require_scene()
        robot = self._get_robot(robot_id)
        result = scene.forwardKinematics(
            self._full_scene_q(ctx), self._native_link_name(robot.config, link_name), ""
        )
        return np.asarray(result, dtype=np.float64)

    def get_group_jacobian(
        self, ctx: RoboPlanContext, group_id: PlanningGroupID
    ) -> NDArray[np.float64]:
        """Get target-frame Jacobian with columns in public group local joint order."""
        self._require_finalized()
        group, robot_id, robot, native_joint_names = self._resolve_group(group_id)
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group_id}' has no pose target frame")
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        scene = self._require_scene()
        result = scene.computeFrameJacobian(
            self._full_scene_q(ctx), self._native_link_name(robot.config, group.tip_link), True
        )
        arr = np.asarray(result, dtype=np.float64)
        if arr.shape[0] != 6:
            raise ValueError(f"Unexpected RoboPlan Jacobian shape: {arr.shape}; expected 6 x n")
        return self._reorder_jacobian_columns(
            group, robot.config.joint_names, native_joint_names, arr
        )

    # KinematicsSpec for RoboPlan Oink IK

    def configure_kinematics(
        self, config: RoboPlanKinematicsConfig | None = None, **overrides: Any
    ) -> None:
        """Configure RoboPlan Oink IK behavior for this world instance."""
        config_values = (config or RoboPlanKinematicsConfig()).model_dump()
        config_values.update(overrides)
        self._kinematics_config = RoboPlanKinematicsConfig(**config_values)

    def solve(
        self,
        world: Any,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve a single pose target with RoboPlan Oink IK."""
        if world is not self:
            return self._ik_failure(
                IKStatus.NO_SOLUTION,
                "RoboPlan IK requires its RoboPlanWorld instance",
            )
        try:
            robot = self._get_robot(robot_id)
            group_id = self._planning_groups.primary_pose_group_id_for_robot(robot.config.name)
            if group_id is None:
                return self._ik_failure(
                    IKStatus.NO_SOLUTION,
                    f"Robot '{robot.config.name}' has no pose-targetable planning group",
                )
            group = self._planning_groups.get(group_id)
        except (KeyError, ValueError) as exc:
            return self._ik_failure(IKStatus.NO_SOLUTION, str(exc))
        return self.solve_pose_targets(
            world=self,
            pose_targets={group: target_pose},
            seed=seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            max_attempts=max_attempts,
        )

    def solve_pose_targets(
        self,
        world: Any,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        auxiliary_groups: Sequence[PlanningGroup] = (),
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve one or more planning-group pose targets with RoboPlan Oink IK."""
        if world is not self:
            return self._ik_failure(
                IKStatus.NO_SOLUTION,
                "RoboPlan IK requires its RoboPlanWorld instance",
            )
        try:
            self._require_finalized()
        except RuntimeError as exc:
            return self._ik_failure(IKStatus.NO_SOLUTION, str(exc))
        if not pose_targets:
            return self._ik_failure(IKStatus.NO_SOLUTION, "At least one pose target is required")

        pose_groups = tuple(pose_targets.keys())
        for group in pose_groups:
            if not group.has_pose_target or group.tip_link is None:
                return self._ik_failure(
                    IKStatus.NO_SOLUTION,
                    f"Planning group '{group.id}' has no pose target frame",
                )

        try:
            selection = PlanningGroupSelection.from_groups(pose_groups + tuple(auxiliary_groups))
        except ValueError as exc:
            return self._ik_failure(IKStatus.NO_SOLUTION, str(exc))
        unsupported = self._validate_supported_selection(selection)
        if unsupported is not None:
            return self._ik_failure(IKStatus.NO_SOLUTION, unsupported.message)

        try:
            optimal_ik = self._load_optimal_ik()
            group_data = self._group_data_for_selection_ids(selection.group_ids)
            scene = self._require_scene()
            oink = optimal_ik.Oink(scene, group_name=group_data.group_name)
            tasks = self._make_oink_frame_tasks(
                optimal_ik,
                oink,
                scene,
                pose_targets,
                position_tolerance,
                orientation_tolerance,
            )
            constraints = self._make_oink_constraints(optimal_ik, oink, group_data)
            candidate_native_q = self._seed_native_selection_q(selection, group_data, seed)
            lower_limits, upper_limits = self._selection_native_limits(selection, group_data)
        except (ImportError, KeyError, ValueError, AttributeError) as exc:
            return self._ik_failure(IKStatus.NO_SOLUTION, f"RoboPlan Oink IK setup failed: {exc}")

        final_position_error = float("inf")
        final_orientation_error = float("inf")
        iteration_limit = max(1, self._kinematics_config.max_iterations)
        if max_attempts <= 0:
            return self._ik_failure(IKStatus.NO_SOLUTION, "max_attempts must be positive")

        try:
            with self.scratch_context() as ctx:
                for iteration in range(iteration_limit):
                    result = self._maybe_return_converged_oink_result(
                        ctx,
                        selection,
                        group_data,
                        candidate_native_q,
                        pose_targets,
                        position_tolerance,
                        orientation_tolerance,
                        iteration,
                    )
                    final_position_error = result.position_error
                    final_orientation_error = result.orientation_error
                    if result.is_success():
                        return result
                    if result.status == IKStatus.COLLISION:
                        return result

                    full_q = self._full_scene_q_with_native_selection_q(
                        ctx, group_data, candidate_native_q
                    )
                    self._set_scene_joint_positions(scene, full_q)
                    delta_q = self._solve_oink_delta(oink, group_data, scene, tasks, constraints)
                    delta_full_q = self._scatter_oink_delta(oink, group_data, delta_q)
                    next_full_q = self._integrate_scene_q(scene, full_q, delta_full_q)
                    self._set_scene_joint_positions(scene, next_full_q)
                    candidate_native_q = self._native_selection_q_from_full_q(
                        group_data, next_full_q
                    )
                    candidate_native_q = np.clip(candidate_native_q, lower_limits, upper_limits)

                result = self._maybe_return_converged_oink_result(
                    ctx,
                    selection,
                    group_data,
                    candidate_native_q,
                    pose_targets,
                    position_tolerance,
                    orientation_tolerance,
                    iteration_limit,
                )
                final_position_error = result.position_error
                final_orientation_error = result.orientation_error
                if result.is_success():
                    return result
                if result.status == IKStatus.COLLISION:
                    return result
        except ValueError as exc:
            return self._ik_failure(IKStatus.NO_SOLUTION, f"RoboPlan Oink IK mapping failed: {exc}")
        except Exception as exc:
            return self._ik_failure(IKStatus.NO_SOLUTION, f"RoboPlan Oink IK failed: {exc}")

        return IKResult(
            status=IKStatus.NO_SOLUTION,
            joint_state=None,
            position_error=final_position_error,
            orientation_error=final_orientation_error,
            iterations=iteration_limit,
            message="RoboPlan Oink IK did not converge within the iteration budget",
        )

    # PlannerSpec for native RoboPlan planning

    def _is_this_world(self, world: object) -> bool:
        return world is self

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a robot-scoped joint path using the robot's full planning group."""
        if not self._is_this_world(world):
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        self._require_finalized()

        try:
            robot = self._get_robot(robot_id)
        except KeyError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))

        group = self._full_robot_planning_group(robot.config.name)
        if group is None:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message=(
                    "RoboPlan-native joint planning requires a planning group that covers "
                    "the robot controllable joint set exactly"
                ),
            )

        selection = self._planning_groups.select((group.id,))
        result = self.plan_selected_joint_path(
            self,
            selection,
            start,
            goal,
            timeout=timeout,
        )
        if not result.is_success():
            return result

        path = self._selection_path_to_robot_joint_path(selection, group, robot, result.path)
        return PlanningResult(
            status=result.status,
            path=path,
            planning_time=result.planning_time,
            path_length=compute_path_length(path),
            iterations=result.iterations,
            message=result.message,
            timestamps=result.timestamps,
        )

    def plan_selected_joint_path(
        self,
        world: Any,
        selection: PlanningGroupSelection,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a path using RoboPlan-native RRT for one selected planning group."""
        if world is not self:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        self._require_finalized()
        unsupported = self._validate_supported_selection(selection)
        if unsupported is not None:
            return unsupported

        start_time = time.time()
        group_data = self._group_data_for_selection_ids(selection.group_ids)
        try:
            invalid_start = self._validate_selected_start_matches_belief(selection, start)
            if invalid_start is not None:
                return invalid_start
            q_start = self._joint_state_to_native_selection_q(selection, group_data, start)
            q_goal = self._joint_state_to_native_selection_q(selection, group_data, goal)
            self._set_scene_current_q(self._live_context)
            result = self._run_native_rrt(
                group_data.group_name, group_data.native_joint_names, q_start, q_goal, timeout
            )
            result_names, path_arrays = self._extract_native_path(result)
            path = self._native_path_to_public_selection_path(
                selection, group_data, path_arrays, result_names
            )
            if path and not self._selection_path_collision_free(selection, path):
                return PlanningResult(
                    status=PlanningStatus.NO_SOLUTION,
                    planning_time=time.time() - start_time,
                    message="RoboPlan-native planning failed: returned path is in collision",
                )
        except ValueError as exc:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                planning_time=time.time() - start_time,
                message=str(exc),
            )
        except Exception as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan-native planning failed: {exc}",
            )
        if not path:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan-native planning failed: returned an empty path",
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - start_time,
            path_length=compute_path_length(path),
            message="RoboPlan path found",
        )

    def plan_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        pose_targets: Mapping[PlanningGroupID, PoseStamped],
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        path_mode: CartesianPathMode = "free",
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a RoboPlan-backed free path to absolute Cartesian TCP targets."""
        if not self._is_this_world(world):
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native Cartesian planner requires its RoboPlanWorld instance",
            )
        self._require_finalized()

        start_time = time.time()
        invalid = self._validate_cartesian_request(
            selection,
            start,
            pose_targets.keys(),
            auxiliary_groups,
            path_mode,
            timeout,
        )
        if invalid is not None:
            invalid.planning_time = time.time() - start_time
            return invalid
        invalid = self._validate_absolute_cartesian_targets(pose_targets)
        if invalid is not None:
            invalid.planning_time = time.time() - start_time
            return invalid
        if path_mode == "linear":
            return self._plan_linear_cartesian_path(
                selection,
                start,
                pose_targets,
                timeout,
                start_time,
            )
        if len(pose_targets) == 1 and roboplan_simple_ik is None:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                planning_time=time.time() - start_time,
                message="RoboPlan Cartesian planning requires roboplan.simple_ik",
            )

        try:
            target_groups = self._target_groups_for_ids(pose_targets.keys())
            auxiliary_group_objects = self._target_groups_for_ids(auxiliary_groups)
            group_data = self._group_data_for_selection_ids(selection.group_ids)
            with self.scratch_context() as ctx:
                self._set_selection_state(ctx, selection, start)
                self._set_scene_current_q(ctx)
                q_start = self._joint_state_to_native_selection_q(selection, group_data, start)
                q_goal_state = self._solve_cartesian_goal_state(
                    selection,
                    group_data,
                    q_start,
                    start,
                    {
                        target_groups[group_id]: PoseStamped(
                            frame_id="world",
                            position=target.position,
                            orientation=target.orientation,
                        )
                        for group_id, target in pose_targets.items()
                    },
                    tuple(auxiliary_group_objects.values()),
                    timeout,
                )
                q_goal = self._joint_state_to_native_selection_q(
                    selection, group_data, q_goal_state
                )
            result = self._run_native_rrt(
                group_data.group_name,
                group_data.native_joint_names,
                q_start,
                q_goal,
                timeout,
            )
            result_names, path_arrays = self._extract_native_path(result)
            path = self._native_path_to_public_selection_path(
                selection, group_data, path_arrays, result_names
            )
            if path and not self._selection_path_collision_free(selection, path):
                return PlanningResult(
                    status=PlanningStatus.NO_SOLUTION,
                    planning_time=time.time() - start_time,
                    message="RoboPlan Cartesian planning failed: returned path is in collision",
                )
            if path and not self._cartesian_final_poses_match(selection, path[-1], pose_targets):
                return PlanningResult(
                    status=PlanningStatus.NO_SOLUTION,
                    planning_time=time.time() - start_time,
                    message="RoboPlan Cartesian planning failed: final TCP pose missed target",
                )
        except ValueError as exc:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                planning_time=time.time() - start_time,
                message=str(exc),
            )
        except Exception as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan Cartesian planning failed: {exc}",
            )
        if not path:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan Cartesian planning failed: returned an empty path",
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - start_time,
            path_length=compute_path_length(path),
            message="RoboPlan Cartesian path found",
        )

    def plan_relative_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        delta_targets: Mapping[PlanningGroupID, CartesianDelta],
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        path_mode: CartesianPathMode = "free",
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a RoboPlan-backed free path to relative Cartesian TCP deltas."""
        if not self._is_this_world(world):
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native Cartesian planner requires its RoboPlanWorld instance",
            )
        self._require_finalized()

        start_time = time.time()
        invalid = self._validate_cartesian_request(
            selection,
            start,
            delta_targets.keys(),
            auxiliary_groups,
            path_mode,
            timeout,
        )
        if invalid is not None:
            invalid.planning_time = time.time() - start_time
            return invalid
        invalid = self._validate_relative_cartesian_targets(delta_targets)
        if invalid is not None:
            invalid.planning_time = time.time() - start_time
            return invalid

        with self.scratch_context() as ctx:
            self._set_selection_state(ctx, selection, start)
            pose_targets = self._resolve_relative_cartesian_targets(ctx, delta_targets)
        result = self.plan_cartesian_path(
            world,
            selection,
            start,
            pose_targets,
            auxiliary_groups=auxiliary_groups,
            path_mode=path_mode,
            timeout=timeout,
        )
        result.planning_time = time.time() - start_time
        return result

    def get_name(self) -> str:
        """Get backend name."""
        return "roboplan"

    def configure_trajectory_parametrization(self, config: TrajectoryParametrizationConfig) -> None:
        """Configure RoboPlan-backed trajectory parametrization."""
        if config.backend != "roboplan":
            raise ValueError("RoboPlanWorld only supports backend='roboplan'")
        self._trajectory_parametrization_config = config

    def parametrize(
        self,
        plan: GeneratedPlan,
        *,
        speed_scale: float = 1.0,
    ) -> GeneratedTrajectory:
        """Time-parametrize a geometric generated plan using RoboPlan TOPP-RA."""
        if not plan.is_success():
            return self._trajectory_parametrization_failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                "GeneratedPlan is not successful",
                speed_scale=speed_scale,
            )
        if speed_scale <= 0.0:
            return self._trajectory_parametrization_failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                "speed_scale must be positive",
                speed_scale=speed_scale,
            )
        if len(plan.path) < 2:
            return self._trajectory_parametrization_failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                "GeneratedPlan must contain at least two waypoints",
                speed_scale=speed_scale,
            )
        try:
            self._require_finalized()
            roboplan_toppra = importlib.import_module("roboplan.toppra")
            selection = self._planning_groups.select(plan.group_ids)
            group_data = self._group_data_for_selection_ids(plan.group_ids)
            options = self._roboplan_toppra_options(roboplan_toppra, speed_scale=speed_scale)
            self._set_scene_current_q(self._live_context)
            smoothed = self._maybe_parametrize_smoothed_path(
                plan,
                selection,
                group_data,
                roboplan_toppra,
                options,
                speed_scale=speed_scale,
            )
            if smoothed is not None:
                return smoothed
            return self._parametrize_roboplan_path(
                plan,
                plan.path,
                selection,
                group_data,
                roboplan_toppra,
                options,
                speed_scale=speed_scale,
            )
        except ImportError:
            return self._trajectory_parametrization_failure(
                plan,
                ParametrizationStatus.BACKEND_UNAVAILABLE,
                "RoboPlan trajectory parametrization requires roboplan.toppra. "
                "Install the manipulation extra before selecting backend='roboplan'.",
                speed_scale=speed_scale,
            )
        except ValueError as exc:
            return self._trajectory_parametrization_failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                str(exc),
                speed_scale=speed_scale,
            )
        except Exception as exc:
            return self._trajectory_parametrization_failure(
                plan,
                ParametrizationStatus.FAILED,
                f"RoboPlan trajectory parametrization failed: {exc}",
                speed_scale=speed_scale,
            )

    # Internals

    def _trajectory_parametrization_failure(
        self,
        plan: GeneratedPlan,
        status: ParametrizationStatus,
        message: str,
        *,
        speed_scale: float = 1.0,
    ) -> GeneratedTrajectory:
        return GeneratedTrajectory(
            status=status,
            message=message,
            speed_scale=speed_scale,
            source_group_ids=plan.group_ids,
            source_plan_status=plan.status,
            source_plan_message=plan.message,
        )

    def _generated_plan_to_roboplan_joint_path(
        self,
        plan: GeneratedPlan,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
    ) -> Any:
        positions: list[list[float]] = []
        for waypoint in plan.path:
            positions_by_global = self._strict_selection_positions_by_global(selection, waypoint)
            position = [
                positions_by_global[group_data.native_to_global_joint_name[native_name]]
                for native_name in group_data.native_joint_names
            ]
            if not np.all(np.isfinite(position)):
                raise ValueError("Generated-plan waypoint positions must be finite")
            positions.append(position)
        joint_path = cast("Any", roboplan_core.JointPath())
        joint_path.joint_names = list(group_data.native_joint_names)
        joint_path.positions = np.asarray(positions, dtype=np.float64)
        return joint_path

    def _parametrize_roboplan_path(
        self,
        source_plan: GeneratedPlan,
        path: list[JointState],
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        roboplan_toppra: Any,
        options: Any,
        *,
        speed_scale: float,
    ) -> GeneratedTrajectory:
        path_plan = replace(source_plan, path=path)
        joint_path = self._generated_plan_to_roboplan_joint_path(path_plan, selection, group_data)
        native_trajectory = roboplan_toppra.PathParameterizerTOPPRA(
            self._require_scene(), group_data.group_name
        ).generate(joint_path, options)
        return self._roboplan_trajectory_to_generated(
            source_plan,
            selection,
            group_data,
            native_trajectory,
            speed_scale=speed_scale,
        )

    def _maybe_parametrize_smoothed_path(
        self,
        plan: GeneratedPlan,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        roboplan_toppra: Any,
        options: Any,
        *,
        speed_scale: float,
    ) -> GeneratedTrajectory | None:
        config = self._trajectory_parametrization_config
        if not config.roboplan_smoothing_enabled:
            return None
        if len(plan.path) < config.roboplan_smoothing_min_waypoints:
            return None
        for candidate_path in self._adaptive_smoothing_candidates(plan.path):
            if len(candidate_path) >= len(plan.path):
                continue
            try:
                rejection = self._validate_smoothed_path(
                    plan, selection, group_data, candidate_path
                )
            except Exception as exc:
                rejection = f"smoothing candidate validation raised {type(exc).__name__}: {exc}"
            if rejection is not None:
                logger.info(
                    "Rejected RoboPlan smoothed path (%d -> %d waypoints): %s",
                    len(plan.path),
                    len(candidate_path),
                    rejection,
                )
                continue
            try:
                trajectory = self._parametrize_roboplan_path(
                    plan,
                    candidate_path,
                    selection,
                    group_data,
                    roboplan_toppra,
                    options,
                    speed_scale=speed_scale,
                )
            except Exception as exc:
                logger.info(
                    "RoboPlan TOPPRA rejected smoothed path (%d -> %d waypoints): %s",
                    len(plan.path),
                    len(candidate_path),
                    exc,
                )
                continue
            try:
                rejection = self._validate_smoothed_trajectory_output(
                    plan, selection, group_data, candidate_path, trajectory
                )
            except Exception as exc:
                rejection = f"smoothed trajectory validation raised {type(exc).__name__}: {exc}"
            if rejection is not None:
                logger.info(
                    "Rejected RoboPlan smoothed trajectory (%d -> %d waypoints): %s",
                    len(plan.path),
                    len(candidate_path),
                    rejection,
                )
                continue
            trajectory.message = (
                f"{trajectory.message}; smoothed RoboPlan path "
                f"{len(plan.path)} -> {len(candidate_path)} waypoints"
            )
            return trajectory
        return None

    def _adaptive_smoothing_candidates(self, path: list[JointState]) -> list[list[JointState]]:
        config = self._trajectory_parametrization_config
        waypoint_count = len(path)
        min_keep = max(2, ceil(waypoint_count * config.roboplan_smoothing_min_keep_fraction))
        max_keep = max(min_keep, waypoint_count - 1)
        if min_keep >= waypoint_count:
            return []
        if config.roboplan_smoothing_max_attempts == 1:
            keep_counts = [min_keep]
        else:
            keep_counts = [
                round(value)
                for value in np.linspace(
                    min_keep,
                    max_keep,
                    config.roboplan_smoothing_max_attempts,
                )
            ]
        candidates: list[list[JointState]] = []
        seen_counts: set[int] = set()
        for keep_count in keep_counts:
            keep_count = min(max(2, keep_count), waypoint_count - 1)
            if keep_count in seen_counts:
                continue
            seen_counts.add(keep_count)
            indices = sorted(
                {round(index) for index in np.linspace(0, waypoint_count - 1, keep_count)}
                | {0, waypoint_count - 1}
            )
            candidates.append([path[index] for index in indices])
        return candidates

    def _validate_smoothed_trajectory_output(
        self,
        source_plan: GeneratedPlan,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        candidate_path: list[JointState],
        trajectory: GeneratedTrajectory,
    ) -> str | None:
        if not trajectory.is_success():
            return trajectory.message or "smoothed trajectory parametrization failed"
        path = [
            JointState(
                name=list(trajectory.joint_names),
                position=[float(value) for value in point.positions],
            )
            for point in trajectory.points
        ]
        if len(path) < 2:
            return "smoothed trajectory output has fewer than two points"
        config = self._trajectory_parametrization_config
        try:
            output_positions = self._strict_selection_positions_matrix(selection, path)
            candidate_positions = self._strict_selection_positions_matrix(selection, candidate_path)
        except ValueError as exc:
            return str(exc)
        output_deviation = self._max_joint_deviation_from_polyline(
            output_positions, candidate_positions
        )
        if output_deviation > config.roboplan_smoothing_max_joint_deviation:
            return (
                "smoothed trajectory output deviates from accepted candidate by "
                f"{output_deviation:.6f}, exceeding "
                f"{config.roboplan_smoothing_max_joint_deviation:.6f}"
            )
        return self._validate_smoothed_path(source_plan, selection, group_data, path)

    def _validate_smoothed_path(
        self,
        source_plan: GeneratedPlan,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        candidate_path: list[JointState],
    ) -> str | None:
        config = self._trajectory_parametrization_config
        try:
            original_positions = self._strict_selection_positions_matrix(
                selection, source_plan.path
            )
            candidate_positions = self._strict_selection_positions_matrix(selection, candidate_path)
            self._strict_selection_native_positions_matrix(selection, group_data, candidate_path)
        except ValueError as exc:
            return str(exc)
        if len(candidate_path) < 2:
            return "candidate path must contain at least two waypoints"
        if not np.allclose(candidate_positions[0], original_positions[0], atol=1e-9):
            return "candidate path does not preserve the original start state"
        if not np.allclose(candidate_positions[-1], original_positions[-1], atol=1e-9):
            return "candidate path does not preserve the original goal state"
        limits_error = self._validate_selection_path_within_limits(
            selection, group_data, candidate_path
        )
        if limits_error is not None:
            return limits_error
        deviation = self._max_joint_deviation_from_polyline(original_positions, candidate_positions)
        if deviation > config.roboplan_smoothing_max_joint_deviation:
            return (
                "candidate path deviates from original joint path by "
                f"{deviation:.6f}, exceeding "
                f"{config.roboplan_smoothing_max_joint_deviation:.6f}"
            )
        if not self._selection_path_collision_free(
            selection, candidate_path, step_size=config.roboplan_smoothing_edge_step_size
        ):
            return "candidate path is in collision"
        path_constraints = source_plan.path_constraints
        if isinstance(path_constraints, LinearTcpPathConstraint):
            return self._validate_linear_tcp_constraint(selection, candidate_path, path_constraints)
        return None

    def _strict_selection_positions_matrix(
        self, selection: PlanningGroupSelection, path: list[JointState]
    ) -> NDArray[np.float64]:
        expected_names = tuple(selection.joint_names)
        positions: list[list[float]] = []
        for waypoint in path:
            positions_by_global = self._strict_selection_positions_by_global(selection, waypoint)
            positions.append([positions_by_global[name] for name in expected_names])
        return np.asarray(positions, dtype=np.float64)

    def _strict_selection_positions_by_global(
        self, selection: PlanningGroupSelection, waypoint: JointState
    ) -> dict[str, float]:
        expected_names = tuple(selection.joint_names)
        if tuple(waypoint.name) != expected_names:
            raise ValueError(
                "GeneratedPlan waypoint names must exactly match selected global joints"
            )
        if len(waypoint.position) != len(expected_names):
            raise ValueError("GeneratedPlan waypoint position count must match selected joints")
        values = [float(value) for value in waypoint.position]
        if not np.all(np.isfinite(values)):
            raise ValueError("GeneratedPlan waypoint positions must be finite")
        return dict(zip(expected_names, values, strict=True))

    def _strict_selection_native_positions_matrix(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        path: list[JointState],
    ) -> NDArray[np.float64]:
        rows: list[list[float]] = []
        for waypoint in path:
            positions_by_global = self._selection_positions_by_global(selection, waypoint)
            rows.append(
                [
                    positions_by_global[group_data.native_to_global_joint_name[native_name]]
                    for native_name in group_data.native_joint_names
                ]
            )
        return np.asarray(rows, dtype=np.float64)

    def _validate_selection_path_within_limits(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        path: list[JointState],
    ) -> str | None:
        lower_limits, upper_limits = self._selection_native_limits(selection, group_data)
        native_positions = self._strict_selection_native_positions_matrix(
            selection, group_data, path
        )
        if np.any(native_positions < lower_limits - 1e-9) or np.any(
            native_positions > upper_limits + 1e-9
        ):
            return "candidate path violates selected joint limits"
        return None

    def _max_joint_deviation_from_polyline(
        self,
        original_positions: NDArray[np.float64],
        candidate_positions: NDArray[np.float64],
    ) -> float:
        max_deviation = 0.0
        for point in original_positions:
            min_distance = min(
                self._point_to_segment_distance(point, start, end)
                for start, end in pairwise(candidate_positions)
            )
            max_deviation = max(max_deviation, min_distance)
        return max_deviation

    def _point_to_segment_distance(
        self,
        point: NDArray[np.float64],
        start: NDArray[np.float64],
        end: NDArray[np.float64],
    ) -> float:
        segment = end - start
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1e-16:
            return float(np.linalg.norm(point - start))
        alpha = float(np.clip(np.dot(point - start, segment) / length_squared, 0.0, 1.0))
        projection = start + alpha * segment
        return float(np.linalg.norm(point - projection))

    def _validate_linear_tcp_constraint(
        self,
        selection: PlanningGroupSelection,
        path: list[JointState],
        constraint: LinearTcpPathConstraint,
    ) -> str | None:
        if constraint.group_id not in selection.group_ids:
            return f"linear TCP constraint group '{constraint.group_id}' is not selected"
        if constraint.start_pose is None or constraint.target_pose is None:
            return "linear TCP constraint requires start_pose and target_pose"
        metadata_error = self._validate_linear_tcp_constraint_metadata(selection, constraint)
        if metadata_error is not None:
            return metadata_error
        config = self._trajectory_parametrization_config
        with self.scratch_context() as ctx:
            for joint_state in self._sample_selection_path(
                selection, path, config.roboplan_smoothing_edge_step_size
            ):
                self._set_selection_state(ctx, selection, joint_state)
                pose = self.get_group_ee_pose(ctx, constraint.group_id)
                position_error, orientation_error = self._linear_tcp_pose_errors(pose, constraint)
                if position_error > constraint.max_translational_deviation:
                    return (
                        "linear TCP candidate deviates from line by "
                        f"{position_error:.6f}, exceeding "
                        f"{constraint.max_translational_deviation:.6f}"
                    )
                if orientation_error > constraint.max_rotational_deviation:
                    return (
                        "linear TCP candidate orientation deviates by "
                        f"{orientation_error:.6f}, exceeding "
                        f"{constraint.max_rotational_deviation:.6f}"
                    )
        return None

    def _validate_linear_tcp_constraint_metadata(
        self,
        selection: PlanningGroupSelection,
        constraint: LinearTcpPathConstraint,
    ) -> str | None:
        group = next(
            (
                selected_group
                for selected_group in selection.groups
                if selected_group.id == constraint.group_id
            ),
            None,
        )
        if group is None:
            return f"linear TCP constraint group '{constraint.group_id}' is not selected"
        expected_tcp_frame = group.tip_link or ""
        if constraint.tcp_frame != expected_tcp_frame:
            return (
                "linear TCP constraint tcp_frame must match selected group tip; "
                f"expected '{expected_tcp_frame}', got '{constraint.tcp_frame}'"
            )
        assert constraint.start_pose is not None
        assert constraint.target_pose is not None
        if constraint.start_pose.frame_id != "world" or constraint.target_pose.frame_id != "world":
            return "linear TCP constraint start_pose and target_pose must be in world frame"
        tolerances = (
            constraint.max_translational_deviation,
            constraint.max_rotational_deviation,
        )
        if any(not np.isfinite(tolerance) or tolerance < 0.0 for tolerance in tolerances):
            return "linear TCP constraint tolerances must be finite and nonnegative"
        return None

    def _sample_selection_path(
        self,
        selection: PlanningGroupSelection,
        path: list[JointState],
        step_size: float,
    ) -> list[JointState]:
        if len(path) < 2:
            return path
        samples: list[JointState] = []
        for start, end in pairwise(path):
            start_by_global = self._selection_positions_by_global(selection, start)
            end_by_global = self._selection_positions_by_global(selection, end)
            q_start = np.asarray(
                [start_by_global[name] for name in selection.joint_names], dtype=np.float64
            )
            q_end = np.asarray(
                [end_by_global[name] for name in selection.joint_names], dtype=np.float64
            )
            distance = float(np.linalg.norm(q_end - q_start))
            n_steps = max(2, ceil(distance / step_size) + 1)
            for index in range(n_steps):
                if samples and index == 0:
                    continue
                alpha = index / (n_steps - 1)
                q = q_start + alpha * (q_end - q_start)
                samples.append(
                    JointState(
                        name=list(selection.joint_names),
                        position=[float(value) for value in q],
                    )
                )
        return samples

    def _linear_tcp_pose_errors(
        self,
        pose: PoseStamped,
        constraint: LinearTcpPathConstraint,
    ) -> tuple[float, float]:
        assert constraint.start_pose is not None
        assert constraint.target_pose is not None
        current_matrix = pose_to_matrix(pose)
        start_matrix = pose_to_matrix(constraint.start_pose)
        target_matrix = pose_to_matrix(constraint.target_pose)
        alpha = self._linear_tcp_alpha(current_matrix, start_matrix, target_matrix)
        expected_matrix = np.eye(4, dtype=np.float64)
        expected_matrix[:3, 3] = start_matrix[:3, 3] + alpha * (
            target_matrix[:3, 3] - start_matrix[:3, 3]
        )
        slerp = Slerp([0.0, 1.0], R.from_matrix([start_matrix[:3, :3], target_matrix[:3, :3]]))
        expected_matrix[:3, :3] = slerp([alpha]).as_matrix()[0]
        return compute_pose_error(current_matrix, expected_matrix)

    def _linear_tcp_alpha(
        self,
        current_matrix: NDArray[np.float64],
        start_matrix: NDArray[np.float64],
        target_matrix: NDArray[np.float64],
    ) -> float:
        translation_delta = target_matrix[:3, 3] - start_matrix[:3, 3]
        translation_distance_squared = float(np.dot(translation_delta, translation_delta))
        if translation_distance_squared > 1e-16:
            return float(
                np.clip(
                    np.dot(current_matrix[:3, 3] - start_matrix[:3, 3], translation_delta)
                    / translation_distance_squared,
                    0.0,
                    1.0,
                )
            )
        start_rotation = R.from_matrix(start_matrix[:3, :3])
        target_rotation = R.from_matrix(target_matrix[:3, :3])
        current_rotation = R.from_matrix(current_matrix[:3, :3])
        total_rotvec = (target_rotation * start_rotation.inv()).as_rotvec()
        total_distance_squared = float(np.dot(total_rotvec, total_rotvec))
        if total_distance_squared <= 1e-16:
            return 0.0
        current_rotvec = (current_rotation * start_rotation.inv()).as_rotvec()
        return float(
            np.clip(np.dot(current_rotvec, total_rotvec) / total_distance_squared, 0.0, 1.0)
        )

    def _roboplan_toppra_options(self, roboplan_toppra: Any, *, speed_scale: float) -> Any:
        config = self._trajectory_parametrization_config
        options = roboplan_toppra.TOPPRAOptions()
        mode_name_by_config = {
            "hermite": "Hermite",
            "cubic": "Cubic",
            "adaptive": "Adaptive",
            "linear_blend": "LinearBlend",
        }
        options.dt = config.roboplan_dt
        options.mode = getattr(
            roboplan_toppra.SplineFittingMode,
            mode_name_by_config[config.roboplan_spline_mode],
        )
        options.velocity_scale = config.velocity_scale * speed_scale
        options.acceleration_scale = config.acceleration_scale * speed_scale
        options.max_adaptive_iterations = config.roboplan_max_adaptive_iterations
        options.max_adaptive_step_size = config.roboplan_max_adaptive_step_size
        options.max_blend_deviation = config.roboplan_max_blend_deviation
        return options

    def _roboplan_trajectory_to_generated(
        self,
        plan: GeneratedPlan,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        native_trajectory: Any,
        *,
        speed_scale: float,
    ) -> GeneratedTrajectory:
        native_joint_names = tuple(
            getattr(native_trajectory, "joint_names", None) or group_data.native_joint_names
        )
        try:
            global_joint_names = [
                group_data.native_to_global_joint_name[native_name]
                for native_name in native_joint_names
            ]
        except KeyError as exc:
            raise ValueError(
                f"RoboPlan trajectory returned unknown native joint '{exc.args[0]}'"
            ) from exc

        times = np.asarray(getattr(native_trajectory, "times", []), dtype=np.float64)
        positions = np.asarray(getattr(native_trajectory, "positions", []), dtype=np.float64)
        velocities_raw = getattr(native_trajectory, "velocities", None)
        velocities = (
            np.zeros_like(positions)
            if velocities_raw is None or len(velocities_raw) == 0
            else np.asarray(velocities_raw, dtype=np.float64)
        )
        if positions.ndim != 2:
            raise ValueError("RoboPlan trajectory positions must be a 2D array")
        if len(times) != positions.shape[0]:
            raise ValueError("RoboPlan trajectory times must align with positions")
        if positions.shape[1] != len(global_joint_names):
            raise ValueError("RoboPlan trajectory joint count does not match joint_names")
        if velocities.shape != positions.shape:
            raise ValueError("RoboPlan trajectory velocities must align with positions")
        if len(times) == 0 or np.any(~np.isfinite(times)) or np.any(np.diff(times) < 0.0):
            raise ValueError("RoboPlan trajectory times must be finite and monotonic")
        if np.any(~np.isfinite(positions)) or np.any(~np.isfinite(velocities)):
            raise ValueError("RoboPlan trajectory contains non-finite values")

        native_indices_by_global = {
            global_name: index for index, global_name in enumerate(global_joint_names)
        }
        selected_names = tuple(selection.joint_names)
        selected_name_set = set(selected_names)
        missing_selected = [
            global_name
            for global_name in selected_names
            if global_name not in native_indices_by_global
        ]
        extra_returned = [
            global_name
            for global_name in global_joint_names
            if global_name not in selected_name_set
        ]
        if missing_selected or extra_returned:
            raise ValueError(
                "RoboPlan trajectory joint_names must match selected global joints; "
                f"missing={missing_selected}, extra={extra_returned}"
            )
        selected_indices = [native_indices_by_global[name] for name in selected_names]
        positions = positions[:, selected_indices]
        velocities = velocities[:, selected_indices]

        points = [
            TrajectoryPoint(
                time_from_start=float(time_from_start),
                positions=[float(value) for value in position],
                velocities=[float(value) for value in velocity],
            )
            for time_from_start, position, velocity in zip(
                times, positions, velocities, strict=True
            )
        ]
        return GeneratedTrajectory(
            joint_names=list(selected_names),
            points=points,
            duration=float(times[-1]),
            speed_scale=speed_scale,
            status=ParametrizationStatus.SUCCESS,
            message="Trajectory parametrized with roboplan",
            source_group_ids=plan.group_ids,
            source_plan_status=plan.status,
            source_plan_message=plan.message,
        )

    def _validate_cartesian_request(
        self,
        selection: PlanningGroupSelection,
        start: JointState,
        target_ids_iterable: Iterable[PlanningGroupID],
        auxiliary_groups: Sequence[PlanningGroupID],
        path_mode: CartesianPathMode,
        timeout: float,
    ) -> PlanningResult | None:
        target_ids = set(target_ids_iterable)
        auxiliary_ids = set(auxiliary_groups)
        if path_mode not in ("free", "linear"):
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message=f"Unsupported Cartesian path_mode: {path_mode}",
            )
        if timeout <= 0.0:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL, message="timeout must be positive"
            )
        if not target_ids:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="At least one Cartesian target group is required",
            )
        if len(auxiliary_ids) != len(auxiliary_groups):
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="auxiliary_groups must not contain duplicate group IDs",
            )
        overlap = target_ids & auxiliary_ids
        if overlap:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message=f"Cartesian target groups overlap auxiliary groups: {sorted(overlap)}",
            )
        selected_ids = set(selection.group_ids)
        coverage = target_ids | auxiliary_ids
        if coverage != selected_ids:
            missing = sorted(selected_ids - coverage)
            extra = sorted(coverage - selected_ids)
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message=(
                    "Cartesian target groups plus auxiliary_groups must exactly cover "
                    f"selection.group_ids; missing={missing}, extra={extra}"
                ),
            )

        try:
            groups_by_id = self._target_groups_for_ids(target_ids | auxiliary_ids)
        except KeyError as exc:
            return PlanningResult(status=PlanningStatus.UNSUPPORTED, message=str(exc))
        for group_id in target_ids:
            if groups_by_id[group_id].tip_link is None:
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=f"Planning group '{group_id}' has no pose target frame",
                )
        unsupported = self._validate_supported_selection(selection)
        if unsupported is not None:
            return unsupported
        try:
            self._selection_positions_by_global(selection, start)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        if path_mode == "linear" and len(target_ids) > 1:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="Linear Cartesian planning currently supports at most one target group",
            )
        return None

    def _validate_absolute_cartesian_targets(
        self, pose_targets: Mapping[PlanningGroupID, PoseStamped]
    ) -> PlanningResult | None:
        for group_id, target in pose_targets.items():
            if not isinstance(target, PoseStamped):
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=f"Absolute Cartesian target for '{group_id}' must be a PoseStamped",
                )
            if target.frame_id not in ("", "world"):
                return PlanningResult(
                    status=PlanningStatus.UNSUPPORTED,
                    message="RoboPlan Cartesian planning currently supports only world-frame poses",
                )
        return None

    def _validate_relative_cartesian_targets(
        self, delta_targets: Mapping[PlanningGroupID, CartesianDelta]
    ) -> PlanningResult | None:
        for group_id, target in delta_targets.items():
            if not isinstance(target, CartesianDelta):
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=f"Relative Cartesian target for '{group_id}' must be a CartesianDelta",
                )
            if target.frame_id != "world":
                return PlanningResult(
                    status=PlanningStatus.UNSUPPORTED,
                    message="RoboPlan Cartesian planning currently supports only world-frame deltas",
                )
        return None

    def _target_groups_for_ids(
        self, group_ids: Iterable[PlanningGroupID]
    ) -> dict[PlanningGroupID, PlanningGroup]:
        return {group_id: self._planning_groups.get(group_id) for group_id in group_ids}

    def _resolve_relative_cartesian_targets(
        self,
        ctx: RoboPlanContext,
        delta_targets: Mapping[PlanningGroupID, CartesianDelta],
    ) -> dict[PlanningGroupID, PoseStamped]:
        return {
            group_id: self._resolve_relative_cartesian_target(ctx, group_id, delta)
            for group_id, delta in delta_targets.items()
        }

    def _resolve_relative_cartesian_target(
        self, ctx: RoboPlanContext, group_id: PlanningGroupID, delta: CartesianDelta
    ) -> PoseStamped:
        start_pose = self.get_group_ee_pose(ctx, group_id)
        start_matrix = pose_to_matrix(start_pose)
        target_matrix = start_matrix.copy()
        target_matrix[:3, 3] = start_matrix[:3, 3] + np.asarray(delta.translation, dtype=np.float64)
        delta_rotation = R.from_euler("xyz", delta.rotation_rpy).as_matrix()
        target_matrix[:3, :3] = delta_rotation @ start_matrix[:3, :3]
        pose = matrix_to_pose(target_matrix)
        return PoseStamped(frame_id="world", position=pose.position, orientation=pose.orientation)

    def _plan_linear_cartesian_path(
        self,
        selection: PlanningGroupSelection,
        start: JointState,
        pose_targets: Mapping[PlanningGroupID, PoseStamped],
        timeout: float,
        start_time: float,
    ) -> PlanningResult:
        if len(pose_targets) != 1:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                planning_time=time.time() - start_time,
                message="Linear Cartesian planning currently supports at most one target group",
            )
        try:
            optimal_ik = self._load_optimal_ik()
        except ImportError as exc:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                planning_time=time.time() - start_time,
                message=f"RoboPlan linear Cartesian planning requires Oink IK: {exc}",
            )

        group_id, target_pose = next(iter(pose_targets.items()))
        try:
            group = self._target_groups_for_ids((group_id,))[group_id]
            group_data = self._group_data_for_selection_ids(selection.group_ids)
            with self.scratch_context() as ctx:
                self._set_selection_state(ctx, selection, start)
                start_pose = self.get_group_ee_pose(ctx, group_id)
                waypoints = self._linear_cartesian_waypoints(start_pose, target_pose)
                q_current = self._joint_state_to_native_selection_q(selection, group_data, start)
                path_arrays = [q_current.copy()]
                for waypoint in waypoints[1:]:
                    q_current = self._solve_linear_oink_waypoint(
                        optimal_ik,
                        selection,
                        group,
                        group_data,
                        q_current,
                        waypoint,
                    )
                    path_arrays.append(q_current.copy())
                path = self._native_path_to_public_selection_path(
                    selection, group_data, path_arrays, None
                )
        except ValueError as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan linear Cartesian planning failed: {exc}",
            )
        except Exception as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan linear Cartesian planning failed: {exc}",
            )
        if not path:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan linear Cartesian planning failed: returned an empty path",
            )
        if not self._selection_path_collision_free(selection, path):
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan linear Cartesian planning failed: returned path is in collision",
            )
        if not self._cartesian_final_poses_match(selection, path[-1], pose_targets):
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan linear Cartesian planning failed: final TCP pose missed target",
            )
        target_pose_world = PoseStamped(
            frame_id="world",
            position=target_pose.position,
            orientation=target_pose.orientation,
        )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - start_time,
            path_length=compute_path_length(path),
            message="RoboPlan linear Cartesian path found",
            path_constraints=LinearTcpPathConstraint(
                group_id=group_id,
                tcp_frame=group.tip_link or "",
                start_pose=start_pose,
                target_pose=target_pose_world,
                max_translational_deviation=_CARTESIAN_POSITION_TOLERANCE,
                max_rotational_deviation=_CARTESIAN_ORIENTATION_TOLERANCE,
            ),
        )

    def _solve_linear_oink_waypoint(
        self,
        optimal_ik: Any,
        selection: PlanningGroupSelection,
        group: PlanningGroup,
        group_data: _RoboPlanGroupData,
        q_seed: NDArray[np.float64],
        target_pose: PoseStamped,
    ) -> NDArray[np.float64]:
        scene = self._require_scene()
        oink = optimal_ik.Oink(scene, group_name=group_data.group_name)
        tasks = self._make_oink_frame_tasks(
            optimal_ik,
            oink,
            scene,
            {group: target_pose},
            _CARTESIAN_POSITION_TOLERANCE,
            _CARTESIAN_ORIENTATION_TOLERANCE,
        )
        constraints = self._make_oink_constraints(optimal_ik, oink, group_data)
        lower_limits, upper_limits = self._selection_native_limits(selection, group_data)
        q_candidate = np.asarray(q_seed, dtype=np.float64).copy()
        iteration_limit = max(1, self._kinematics_config.max_iterations)
        with self.scratch_context() as ctx:
            for iteration in range(iteration_limit):
                result = self._maybe_return_converged_oink_result(
                    ctx,
                    selection,
                    group_data,
                    q_candidate,
                    {group: target_pose},
                    _CARTESIAN_POSITION_TOLERANCE,
                    _CARTESIAN_ORIENTATION_TOLERANCE,
                    iteration,
                )
                if result.is_success():
                    return q_candidate
                if result.status == IKStatus.COLLISION:
                    raise ValueError(result.message)

                self._set_scene_group_joint_positions(scene, group_data.group_name, q_candidate)
                delta_q = self._solve_oink_delta(oink, group_data, scene, tasks, constraints)
                q_candidate = self._integrate_native_selection_q(
                    oink, group_data, q_candidate, delta_q
                )
                q_candidate = np.clip(q_candidate, lower_limits, upper_limits)

            result = self._maybe_return_converged_oink_result(
                ctx,
                selection,
                group_data,
                q_candidate,
                {group: target_pose},
                _CARTESIAN_POSITION_TOLERANCE,
                _CARTESIAN_ORIENTATION_TOLERANCE,
                iteration_limit,
            )
            if result.is_success():
                return q_candidate
            raise ValueError(result.message)

    def _linear_cartesian_waypoints(
        self, start_pose: PoseStamped, target_pose: PoseStamped
    ) -> list[PoseStamped]:
        start_matrix = pose_to_matrix(start_pose)
        target_matrix = pose_to_matrix(target_pose)
        translation_delta = target_matrix[:3, 3] - start_matrix[:3, 3]
        translation_distance = float(np.linalg.norm(translation_delta))
        start_rotation = R.from_matrix(start_matrix[:3, :3])
        target_rotation = R.from_matrix(target_matrix[:3, :3])
        rotation_distance = float((target_rotation * start_rotation.inv()).magnitude())
        step_count = max(
            1,
            ceil(translation_distance / _LINEAR_CARTESIAN_TRANSLATION_STEP),
            ceil(rotation_distance / _LINEAR_CARTESIAN_ROTATION_STEP),
        )
        slerp = Slerp([0.0, 1.0], R.from_matrix([start_matrix[:3, :3], target_matrix[:3, :3]]))
        waypoints: list[PoseStamped] = []
        for step in range(step_count + 1):
            alpha = step / step_count
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, 3] = start_matrix[:3, 3] + alpha * translation_delta
            matrix[:3, :3] = slerp([alpha]).as_matrix()[0]
            pose = matrix_to_pose(matrix)
            waypoints.append(
                PoseStamped(frame_id="world", position=pose.position, orientation=pose.orientation)
            )
        return waypoints

    def _solve_cartesian_goal_state(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        q_start: NDArray[np.float64],
        start: JointState,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        auxiliary_groups: Sequence[PlanningGroup],
        timeout: float,
    ) -> JointState:
        if len(pose_targets) == 1:
            group, target_pose = next(iter(pose_targets.items()))
            q_goal = self._solve_cartesian_goal(group, group_data, q_start, target_pose, timeout)
            return self._native_selection_q_to_joint_state(selection, group_data, q_goal)

        ordered_pose_targets = {
            group: pose_targets[group] for group in selection.groups if group in pose_targets
        }
        ordered_auxiliary_groups = tuple(
            group for group in selection.groups if group in set(auxiliary_groups)
        )
        ik_result = self.solve_pose_targets(
            self,
            ordered_pose_targets,
            auxiliary_groups=ordered_auxiliary_groups,
            seed=start,
            position_tolerance=_CARTESIAN_POSITION_TOLERANCE,
            orientation_tolerance=_CARTESIAN_ORIENTATION_TOLERANCE,
        )
        if not ik_result.is_success() or ik_result.joint_state is None:
            raise ValueError(f"RoboPlan Cartesian IK failed: {ik_result.message}")
        positions_by_global = self._selection_positions_by_global(selection, ik_result.joint_state)
        return JointState(
            name=list(selection.joint_names),
            position=[positions_by_global[name] for name in selection.joint_names],
        )

    def _solve_cartesian_goal(
        self,
        group: PlanningGroup,
        group_data: _RoboPlanGroupData,
        q_start: NDArray[np.float64],
        target_pose: PoseStamped,
        timeout: float,
    ) -> NDArray[np.float64]:
        if roboplan_simple_ik is None:
            raise ValueError("RoboPlan SimpleIk is unavailable")
        options_cls = getattr(roboplan_simple_ik, "SimpleIkOptions", None)
        solver_cls = getattr(roboplan_simple_ik, "SimpleIk", None)
        if options_cls is None or solver_cls is None:
            raise ValueError("roboplan.simple_ik does not expose SimpleIk and SimpleIkOptions")

        options = options_cls()
        self._configure_simple_ik_options(options, timeout, group_data.group_name)
        solver = solver_cls(self._require_scene(), options)
        goal = self._to_native_cartesian_configuration(group, target_pose)
        start_config = self._to_native_joint_configuration(group_data.native_joint_names, q_start)
        solution_config = self._to_native_joint_configuration(
            group_data.native_joint_names, q_start
        )
        solve_ik = getattr(solver, "solveIk", None)
        if solve_ik is None:
            raise ValueError("RoboPlan SimpleIk solver does not expose solveIk")
        result = solve_ik(goal, start_config, solution_config)
        if isinstance(result, bool):
            if not result:
                raise ValueError("RoboPlan SimpleIk failed to solve Cartesian target")
        elif result is not None:
            solution_config = result
        return self._native_joint_configuration_to_q(group_data.native_joint_names, solution_config)

    def _configure_simple_ik_options(
        self, options: Any, timeout: float, group_name: str | None = None
    ) -> None:
        for name, value in (
            *((("group_name", group_name),) if group_name is not None else ()),
            ("max_time", timeout),
            ("timeout", timeout),
            ("max_solve_time", timeout),
            ("max_iterations", 100),
        ):
            try:
                setattr(options, name, value)
            except AttributeError:
                continue

    def _to_native_cartesian_configuration(
        self, group: PlanningGroup, target_pose: PoseStamped
    ) -> Any:
        config_cls = getattr(roboplan_core, "CartesianConfiguration", None)
        if config_cls is None:
            raise ValueError("roboplan.core does not expose CartesianConfiguration")
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group.id}' has no pose target frame")
        frame_name = self._native_link_name(
            self._get_robot(self._robot_ids_by_name[group.robot_name]).config, group.tip_link
        )
        matrix = pose_to_matrix(target_pose)
        for args in (("", frame_name, matrix), (frame_name, matrix), (matrix, frame_name)):
            try:
                return config_cls(*args)
            except TypeError:
                continue
        config = config_cls()
        for name, value in (
            ("frame_name", frame_name),
            ("frame", frame_name),
            ("tip_frame", frame_name),
            ("base_frame", ""),
            ("matrix", matrix),
            ("pose", matrix),
            ("transform", matrix),
            ("tform", matrix),
        ):
            try:
                setattr(config, name, value)
            except AttributeError:
                continue
        return config

    def _native_joint_configuration_to_q(
        self, native_joint_names: tuple[str, ...], config: Any
    ) -> NDArray[np.float64]:
        positions = np.asarray(getattr(config, "positions", []), dtype=np.float64)
        if len(positions) != len(native_joint_names):
            raise ValueError(
                "RoboPlan SimpleIk solution length does not match selected planning group: "
                f"{len(positions)} != {len(native_joint_names)}"
            )
        result_joint_names = getattr(config, "joint_names", None)
        if not result_joint_names:
            return positions
        names = tuple(result_joint_names)
        if len(set(names)) != len(names):
            raise ValueError("RoboPlan SimpleIk returned duplicate joint names")
        if set(names) != set(native_joint_names):
            raise ValueError(
                "RoboPlan SimpleIk joint names do not match selected planning group: "
                f"RoboPlan={list(names)}, configured={list(native_joint_names)}"
            )
        positions_by_name = dict(zip(names, positions, strict=True))
        return np.asarray([positions_by_name[name] for name in native_joint_names])

    def _cartesian_final_pose_matches(
        self,
        selection: PlanningGroupSelection,
        group_id: PlanningGroupID,
        final_state: JointState,
        target_pose: PoseStamped,
    ) -> bool:
        with self.scratch_context() as ctx:
            self._set_selection_state(ctx, selection, final_state)
            actual_pose = self.get_group_ee_pose(ctx, group_id)
        actual_matrix = pose_to_matrix(actual_pose)
        target_matrix = pose_to_matrix(target_pose)
        position_error = float(np.linalg.norm(actual_matrix[:3, 3] - target_matrix[:3, 3]))
        rotation_error = float(
            R.from_matrix(target_matrix[:3, :3].T @ actual_matrix[:3, :3]).magnitude()
        )
        return (
            position_error <= _CARTESIAN_POSITION_TOLERANCE
            and rotation_error <= _CARTESIAN_ORIENTATION_TOLERANCE
        )

    def _cartesian_final_poses_match(
        self,
        selection: PlanningGroupSelection,
        final_state: JointState,
        target_poses: Mapping[PlanningGroupID, PoseStamped],
    ) -> bool:
        return all(
            self._cartesian_final_pose_matches(selection, group_id, final_state, target_pose)
            for group_id, target_pose in target_poses.items()
        )

    def _ik_failure(
        self,
        status: IKStatus,
        message: str,
        position_error: float = 0.0,
        orientation_error: float = 0.0,
        iterations: int = 0,
    ) -> IKResult:
        return IKResult(
            status=status,
            joint_state=None,
            position_error=position_error,
            orientation_error=orientation_error,
            iterations=iterations,
            message=message,
        )

    def _load_optimal_ik(self) -> Any:
        try:
            return importlib.import_module("roboplan.optimal_ik")
        except ImportError as exc:
            raise ImportError(
                "RoboPlan Oink IK requires roboplan.optimal_ik. "
                "Install a RoboPlan build that includes the optimal_ik Python bindings."
            ) from exc

    def _make_oink_frame_tasks(
        self,
        optimal_ik: Any,
        oink: Any,
        scene: Any,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> list[Any]:
        tasks: list[Any] = []
        for group, target_pose in pose_targets.items():
            if group.tip_link is None:
                raise ValueError(f"Planning group '{group.id}' has no pose target frame")
            native_map = self._native_names_by_robot[group.robot_name]
            target = cast("Any", roboplan_core).CartesianConfiguration()
            # DimOS public Cartesian pose targets are absolute world-frame PoseStamped
            # values. Oink interprets an empty base frame as world, matching the
            # target.tform matrix below even for robot-scoped groups in composite scenes.
            target.base_frame = ""
            target.tip_frame = native_map.link(group.tip_link)
            target.tform = pose_to_matrix(target_pose)
            options = self._make_oink_frame_task_options(
                optimal_ik,
                position_tolerance,
                orientation_tolerance,
            )
            tasks.append(optimal_ik.FrameTask(oink, scene, target, options))
        return tasks

    def _make_oink_frame_task_options(
        self,
        optimal_ik: Any,
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> Any:
        values = {
            "position_cost": self._kinematics_config.position_cost,
            "orientation_cost": self._kinematics_config.orientation_cost,
            "task_gain": self._kinematics_config.task_gain,
            "lm_damping": self._kinematics_config.lm_damping,
            "max_position_error": position_tolerance,
            "max_rotation_error": orientation_tolerance,
        }
        try:
            return optimal_ik.FrameTaskOptions(**values)
        except TypeError:
            options = optimal_ik.FrameTaskOptions()
            for name, value in values.items():
                setattr(options, name, value)
            return options

    def _make_oink_constraints(
        self, optimal_ik: Any, oink: Any, group_data: _RoboPlanGroupData
    ) -> list[Any]:
        constraints = [optimal_ik.PositionLimit(oink, gain=1.0)]
        if self._kinematics_config.velocity_limit is not None:
            v_max = np.full(
                self._oink_variable_count(oink, group_data),
                self._kinematics_config.velocity_limit,
                dtype=np.float64,
            )
            constraints.append(
                optimal_ik.VelocityLimit(oink, self._kinematics_config.dt, v_max=v_max)
            )
        return constraints

    def _oink_variable_count(self, oink: Any, group_data: _RoboPlanGroupData) -> int:
        v_indices = getattr(oink, "v_indices", None)
        if v_indices is None:
            return len(group_data.native_joint_names)
        return len(tuple(v_indices))

    def _seed_native_selection_q(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        seed: JointState | None,
    ) -> NDArray[np.float64]:
        positions_by_global = self._selection_belief_positions(selection)
        if seed is not None:
            self._overlay_seed_positions(selection, positions_by_global, seed)
        return np.asarray(
            [
                positions_by_global[group_data.native_to_global_joint_name[native_name]]
                for native_name in group_data.native_joint_names
            ],
            dtype=np.float64,
        )

    def _overlay_seed_positions(
        self,
        selection: PlanningGroupSelection,
        positions_by_global: dict[str, float],
        seed: JointState,
    ) -> None:
        if not seed.name:
            if len(seed.position) != len(selection.joint_names):
                raise ValueError(
                    f"Seed has {len(seed.position)} positions for selection, "
                    f"expected {len(selection.joint_names)}"
                )
            for global_name, position in zip(selection.joint_names, seed.position, strict=True):
                positions_by_global[global_name] = float(position)
            return
        if len(seed.name) != len(seed.position):
            raise ValueError(f"Seed has {len(seed.name)} names but {len(seed.position)} positions")

        local_to_global: dict[str, str] = {}
        for group in selection.groups:
            for local_name, global_name in zip(
                group.local_joint_names, group.joint_names, strict=True
            ):
                local_to_global[local_name] = global_name
                local_to_global[f"{group.robot_name}/{local_name}"] = global_name

        seen: set[str] = set()
        for seed_name, position in zip(seed.name, seed.position, strict=True):
            if seed_name in selection.joint_names:
                global_name = seed_name
            elif seed_name in local_to_global:
                global_name = local_to_global[seed_name]
            else:
                raise ValueError(f"Seed contains joint outside RoboPlan selection: {seed_name}")
            if global_name in seen:
                raise ValueError(f"Seed contains duplicate selected joint: {global_name}")
            seen.add(global_name)
            positions_by_global[global_name] = float(position)

    def _selection_native_limits(
        self, selection: PlanningGroupSelection, group_data: _RoboPlanGroupData
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        lower_by_global: dict[str, float] = {}
        upper_by_global: dict[str, float] = {}
        for group in selection.groups:
            robot_id = self._robot_ids_by_name[group.robot_name]
            robot = self._get_robot(robot_id)
            if robot.lower_limits is None or robot.upper_limits is None:
                raise ValueError(f"RoboPlan joint limits are unavailable for robot '{robot_id}'")
            index_by_local = {name: index for index, name in enumerate(robot.config.joint_names)}
            for local_name, global_name in zip(
                group.local_joint_names, group.joint_names, strict=True
            ):
                index = index_by_local[local_name]
                lower_by_global[global_name] = float(robot.lower_limits[index])
                upper_by_global[global_name] = float(robot.upper_limits[index])
        return (
            np.asarray(
                [
                    lower_by_global[group_data.native_to_global_joint_name[native_name]]
                    for native_name in group_data.native_joint_names
                ],
                dtype=np.float64,
            ),
            np.asarray(
                [
                    upper_by_global[group_data.native_to_global_joint_name[native_name]]
                    for native_name in group_data.native_joint_names
                ],
                dtype=np.float64,
            ),
        )

    def _maybe_return_converged_oink_result(
        self,
        ctx: RoboPlanContext,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        native_q: NDArray[np.float64],
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        position_tolerance: float,
        orientation_tolerance: float,
        iterations: int,
    ) -> IKResult:
        joint_state = self._native_selection_q_to_joint_state(selection, group_data, native_q)
        self._set_selection_state(ctx, selection, joint_state)
        position_error, orientation_error = self._pose_target_errors(ctx, pose_targets)
        if position_error > position_tolerance or orientation_error > orientation_tolerance:
            return IKResult(
                status=IKStatus.NO_SOLUTION,
                joint_state=None,
                position_error=position_error,
                orientation_error=orientation_error,
                iterations=iterations,
                message="RoboPlan Oink IK candidate has not converged",
            )
        if self._kinematics_config.collision_check and not self._selection_config_collision_free(
            selection, joint_state
        ):
            return IKResult(
                status=IKStatus.COLLISION,
                joint_state=None,
                position_error=position_error,
                orientation_error=orientation_error,
                iterations=iterations,
                message="RoboPlan Oink IK converged to a colliding configuration",
            )
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=joint_state,
            position_error=position_error,
            orientation_error=orientation_error,
            iterations=iterations,
            message="RoboPlan Oink IK solution found",
        )

    def _native_selection_q_to_joint_state(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        native_q: NDArray[np.float64],
    ) -> JointState:
        if len(native_q) != len(group_data.native_joint_names):
            raise ValueError(
                f"RoboPlan Oink returned {len(native_q)} selected positions, "
                f"expected {len(group_data.native_joint_names)}"
            )
        positions_by_global = {
            group_data.native_to_global_joint_name[native_name]: float(position)
            for native_name, position in zip(group_data.native_joint_names, native_q, strict=True)
        }
        return JointState(
            {
                "name": list(selection.joint_names),
                "position": [positions_by_global[name] for name in selection.joint_names],
            }
        )

    def _pose_target_errors(
        self, ctx: RoboPlanContext, pose_targets: Mapping[PlanningGroup, PoseStamped]
    ) -> tuple[float, float]:
        position_errors: list[float] = []
        orientation_errors: list[float] = []
        for group, target_pose in pose_targets.items():
            current_pose = self.get_group_ee_pose(ctx, group.id)
            position_error, orientation_error = compute_pose_error(
                pose_to_matrix(current_pose), pose_to_matrix(target_pose)
            )
            position_errors.append(position_error)
            orientation_errors.append(orientation_error)
        return max(position_errors), max(orientation_errors)

    def _full_scene_q_with_native_selection_q(
        self,
        ctx: RoboPlanContext,
        group_data: _RoboPlanGroupData,
        native_q: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        if len(native_q) != len(group_data.native_joint_names):
            raise ValueError(
                f"Selected q length {len(native_q)} does not match "
                f"{len(group_data.native_joint_names)} native joints"
            )
        full_q = self._full_scene_q(ctx)
        index_by_native = {name: index for index, name in enumerate(self._full_native_joint_names)}
        for native_name, position in zip(group_data.native_joint_names, native_q, strict=True):
            full_q[index_by_native[native_name]] = float(position)
        return full_q

    def _set_scene_joint_positions(self, scene: Any, q: NDArray[np.float64]) -> None:
        setter = getattr(scene, "setJointPositions", None)
        if setter is not None:
            setter(np.asarray(q, dtype=np.float64))

    def _set_scene_group_joint_positions(
        self, scene: Any, group_name: str, q: NDArray[np.float64]
    ) -> None:
        setter = getattr(scene, "setJointPositions", None)
        if setter is None:
            raise ValueError("RoboPlan Scene does not expose setJointPositions")
        q_array = np.asarray(q, dtype=np.float64)
        try:
            setter(group_name, q_array)
            return
        except TypeError:
            pass
        try:
            setter(q_array, group_name)
            return
        except TypeError:
            pass

        to_full = getattr(scene, "toFullJointPositions", None)
        if to_full is None:
            raise ValueError(
                "RoboPlan Scene does not expose a group-aware joint state setter "
                "or toFullJointPositions"
            )
        full_q = np.asarray(to_full(group_name, q_array), dtype=np.float64)
        setter(full_q)

    def _solve_oink_delta(
        self,
        oink: Any,
        group_data: _RoboPlanGroupData,
        scene: Any,
        tasks: Sequence[Any],
        constraints: Sequence[Any],
    ) -> NDArray[np.float64]:
        delta_q = np.zeros(self._oink_variable_count(oink, group_data), dtype=np.float64)
        oink.solveIk(
            scene,
            list(tasks),
            list(constraints),
            [],
            delta_q,
            regularization=self._kinematics_config.regularization,
        )
        return delta_q

    def _scatter_oink_delta(
        self, oink: Any, group_data: _RoboPlanGroupData, delta_q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        delta = np.asarray(delta_q, dtype=np.float64)
        delta_full = np.zeros(len(self._full_native_joint_names), dtype=np.float64)
        v_indices = getattr(oink, "v_indices", None)
        if v_indices is not None:
            indices = np.asarray(tuple(v_indices), dtype=np.int64)
            if len(indices) != len(delta):
                raise ValueError(
                    f"Oink returned delta length {len(delta)} for {len(indices)} velocity indices"
                )
            delta_full[indices] = delta
            return delta_full
        if len(delta) == len(delta_full):
            return delta
        if len(delta) != len(group_data.native_joint_names):
            raise ValueError(
                f"Oink returned delta length {len(delta)}, expected full scene length "
                f"{len(delta_full)} or selected length {len(group_data.native_joint_names)}"
            )
        index_by_native = {name: index for index, name in enumerate(self._full_native_joint_names)}
        for native_name, value in zip(group_data.native_joint_names, delta, strict=True):
            delta_full[index_by_native[native_name]] = float(value)
        return delta_full

    def _integrate_native_selection_q(
        self,
        oink: Any,
        group_data: _RoboPlanGroupData,
        native_q: NDArray[np.float64],
        delta_q: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        delta_full_q = self._scatter_oink_delta(oink, group_data, delta_q)
        delta_native_q = self._native_selection_q_from_full_q(group_data, delta_full_q)
        return np.asarray(native_q, dtype=np.float64) + delta_native_q

    def _integrate_scene_q(
        self, scene: Any, q: NDArray[np.float64], delta_q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        integrator = getattr(scene, "integrate", None)
        if integrator is not None:
            return np.asarray(integrator(q, delta_q), dtype=np.float64)
        return np.asarray(q, dtype=np.float64) + np.asarray(delta_q, dtype=np.float64)

    def _native_selection_q_from_full_q(
        self, group_data: _RoboPlanGroupData, full_q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        index_by_native = {name: index for index, name in enumerate(self._full_native_joint_names)}
        return np.asarray(
            [full_q[index_by_native[native_name]] for native_name in group_data.native_joint_names],
            dtype=np.float64,
        )

    def _create_scene(self) -> Any:
        if self._uses_composite_model:
            urdf_path, srdf_path, package_paths = self._prepare_composite_model()
            scene_name = "dimos_composite"
        else:
            robot = next(iter(self._robots.values()))
            urdf_path = self._prepare_robot_urdf(robot.config)
            scene_name = robot.config.name
            self._register_identity_native_names(robot.config, urdf_path)
            self._register_roboplan_groups()
            srdf_path = self._prepare_single_robot_srdf(robot.config, urdf_path)
            package_paths = [str(path) for path in robot.config.package_paths.values()]
        scene = roboplan_core.Scene(scene_name, str(urdf_path), str(srdf_path), package_paths)
        if self._uses_composite_model:
            self._apply_collision_exclusions_from_srdf(scene, srdf_path)
        else:
            robot = next(iter(self._robots.values()))
            self._apply_collision_exclusions(scene, robot.config, urdf_path)
        return scene

    def _validate_robot_config(self, config: RobotModelConfig) -> None:
        if not config.joint_names:
            raise ValueError("RoboPlanWorld requires explicit joint_names")

    def _prepare_robot_urdf(self, config: RobotModelConfig) -> Path:
        return Path(
            prepare_urdf_for_drake(
                config.model_path,
                package_paths=config.package_paths,
                xacro_args=config.xacro_args,
                convert_meshes=config.auto_convert_meshes,
                strip_world_joint_child_link=config.base_link
                if config.strip_model_world_joint
                else None,
            )
        )

    def _prepare_single_robot_srdf(self, config: RobotModelConfig, urdf_path: Path) -> Path:
        if config.srdf_path is not None:
            srdf_path = Path(config.srdf_path)
            if not srdf_path.exists():
                raise FileNotFoundError(f"Robot SRDF not found: {srdf_path.resolve()}")
            return srdf_path

        srdf = self._generate_srdf(config, urdf_path)
        srdf_tempdir = tempfile.TemporaryDirectory(prefix="dimos_roboplan_srdf_")
        self._srdf_tempdirs.append(srdf_tempdir)
        cache_dir = Path(srdf_tempdir.name)
        srdf_path = cache_dir / f"{config.name}.srdf"
        srdf_path.write_text(srdf)
        return srdf_path

    def _generate_srdf(self, config: RobotModelConfig, urdf_path: Path) -> str:
        lines = [f'<robot name="{escape(config.name)}">']
        if self._roboplan_groups_by_selection:
            group_items = [
                (group_data.group_name, group_data.native_joint_names)
                for group_data in self._roboplan_groups_by_selection.values()
            ]
        else:
            group_items = [
                (group.name, tuple(group.joint_names)) for group in config.planning_groups
            ]
        for group_name, joint_names in group_items:
            lines.append(f'  <group name="{escape(group_name)}">')
            for joint_name in joint_names:
                lines.append(f'    <joint name="{escape(joint_name)}"/>')
            lines.append("  </group>")
        for link1, link2 in self._collision_exclusion_pairs(config, urdf_path):
            lines.append(
                f'  <disable_collisions link1="{escape(link1)}" link2="{escape(link2)}" '
                'reason="DimOS configured"/>'
            )
        lines.append("</robot>")
        return "\n".join(lines) + "\n"

    def _prepare_composite_model(self) -> tuple[Path, Path, list[str]]:
        tempdir = tempfile.TemporaryDirectory(prefix="dimos_roboplan_composite_")
        self._srdf_tempdirs.append(tempdir)
        cache_dir = Path(tempdir.name)
        prepared_urdfs: dict[RobotName, Path] = {}
        root = ET.Element("robot", {"name": "dimos_composite"})
        ET.SubElement(root, "link", {"name": "dimos_world"})

        for robot in self._robots.values():
            prepared_urdf = self._prepare_robot_urdf(robot.config)
            prepared_urdfs[robot.config.name] = prepared_urdf
            native_names = self._rewrite_robot_urdf_into_composite(
                root, robot.config, prepared_urdf
            )
            self._native_names_by_robot[robot.config.name] = native_names

        self._register_roboplan_groups()
        urdf_path = cache_dir / "dimos_composite.urdf"
        ET.ElementTree(root).write(urdf_path, encoding="unicode", xml_declaration=True)
        srdf_path = cache_dir / "dimos_composite.srdf"
        srdf_path.write_text(self._generate_composite_srdf(prepared_urdfs))
        package_paths = self._composite_package_paths()
        return urdf_path, srdf_path, package_paths

    def _rewrite_robot_urdf_into_composite(
        self, composite_root: ET.Element, config: RobotModelConfig, urdf_path: Path
    ) -> _NativeNameMap:
        try:
            robot_root = ET.parse(urdf_path).getroot()
        except ET.ParseError as exc:
            raise ValueError(
                f"Unable to parse prepared URDF for composite model: {urdf_path}"
            ) from exc

        local_links = [name for link in robot_root.findall("link") if (name := link.get("name"))]
        local_joints = [
            name for joint in robot_root.findall("joint") if (name := joint.get("name"))
        ]
        local_movable_joints = [
            name
            for joint in robot_root.findall("joint")
            if (name := joint.get("name")) and self._is_scene_position_joint(joint)
        ]
        local_materials = [
            name for material in robot_root.findall(".//material") if (name := material.get("name"))
        ]
        if len(set(local_links)) != len(local_links):
            raise ValueError(f"Robot '{config.name}' URDF has duplicate link names")
        if len(set(local_joints)) != len(local_joints):
            raise ValueError(f"Robot '{config.name}' URDF has duplicate joint names")

        link_map = {name: self._native_name(config.name, name) for name in local_links}
        joint_map = {name: self._native_name(config.name, name) for name in local_joints}
        material_map = {name: self._native_name(config.name, name) for name in local_materials}
        for joint_name in config.joint_names:
            joint_map.setdefault(joint_name, self._native_name(config.name, joint_name))
            if joint_name not in local_movable_joints:
                local_movable_joints.append(joint_name)
        for link_name in (config.base_link, config.end_effector_link):
            if link_name:
                link_map.setdefault(link_name, self._native_name(config.name, link_name))
        self._movable_native_joint_names_by_robot[config.name] = tuple(
            joint_map[name] for name in local_movable_joints
        )

        for child in list(robot_root):
            rewritten = deepcopy(child)
            self._rewrite_urdf_names(rewritten, link_map, joint_map, material_map)
            composite_root.append(rewritten)

        if config.base_link not in link_map:
            raise ValueError(
                f"Robot '{config.name}' base_link '{config.base_link}' was not found in prepared URDF"
            )
        fixed_joint = ET.SubElement(
            composite_root,
            "joint",
            {"name": self._native_name(config.name, "dimos_world_joint"), "type": "fixed"},
        )
        ET.SubElement(fixed_joint, "parent", {"link": "dimos_world"})
        ET.SubElement(fixed_joint, "child", {"link": link_map[config.base_link]})
        ET.SubElement(fixed_joint, "origin", self._pose_origin_attrs(config.base_pose))
        return _NativeNameMap(config.name, joint_map, link_map)

    def _rewrite_urdf_names(
        self,
        element: ET.Element,
        link_map: dict[str, str],
        joint_map: dict[str, str],
        material_map: dict[str, str],
    ) -> None:
        if element.tag == "link" and (name := element.get("name")) in link_map:
            element.set("name", link_map[name])
        elif element.tag == "joint" and (name := element.get("name")) in joint_map:
            element.set("name", joint_map[name])
        elif element.tag == "material" and (name := element.get("name")) in material_map:
            element.set("name", material_map[name])

        for attr_name, name_map in (("link", link_map), ("joint", joint_map)):
            attr_value = element.get(attr_name)
            if attr_value in name_map:
                element.set(attr_name, name_map[attr_value])
        for child in list(element):
            self._rewrite_urdf_names(child, link_map, joint_map, material_map)

    def _pose_origin_attrs(self, pose: PoseStamped) -> dict[str, str]:
        mat = pose_to_matrix(pose)
        xyz = mat[:3, 3]
        rpy = self._matrix_to_rpy(mat[:3, :3])
        return {
            "xyz": " ".join(f"{float(value):.17g}" for value in xyz),
            "rpy": " ".join(f"{float(value):.17g}" for value in rpy),
        }

    def _matrix_to_rpy(self, rotation: NDArray[np.float64]) -> tuple[float, float, float]:
        sy = sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
        if sy > 1e-9:
            return (
                atan2(rotation[2, 1], rotation[2, 2]),
                atan2(-rotation[2, 0], sy),
                atan2(rotation[1, 0], rotation[0, 0]),
            )
        return (atan2(-rotation[1, 2], rotation[1, 1]), atan2(-rotation[2, 0], sy), 0.0)

    def _register_identity_native_names(self, config: RobotModelConfig, urdf_path: Path) -> None:
        try:
            root = ET.parse(urdf_path).getroot()
        except ET.ParseError as exc:
            raise ValueError(
                f"Unable to parse prepared URDF for RoboPlan model: {urdf_path}"
            ) from exc
        link_names = {name for link in root.findall("link") if (name := link.get("name"))}
        joint_names = {name for joint in root.findall("joint") if (name := joint.get("name"))}
        movable_joint_names = [
            name
            for joint in root.findall("joint")
            if (name := joint.get("name")) and self._is_scene_position_joint(joint)
        ]
        for joint_name in config.joint_names:
            if joint_name not in movable_joint_names:
                movable_joint_names.append(joint_name)
        self._native_names_by_robot[config.name] = _NativeNameMap(
            robot_name=config.name,
            joint_names={name: name for name in sorted(joint_names | set(config.joint_names))},
            link_names={
                name: name
                for name in sorted(link_names | {config.base_link, config.end_effector_link or ""})
                if name
            },
        )
        self._movable_native_joint_names_by_robot[config.name] = tuple(movable_joint_names)

    def _is_scene_position_joint(self, joint: ET.Element) -> bool:
        return joint.get("type") != "fixed" and joint.find("mimic") is None

    def _register_roboplan_groups(self) -> None:
        self._roboplan_groups_by_selection.clear()
        self._roboplan_native_joint_names.clear()
        groups = self._planning_groups.list()
        for group in groups:
            group_data = self._make_roboplan_group_data((group,))
            self._roboplan_groups_by_selection[frozenset(group_data.group_ids)] = group_data
            self._roboplan_native_joint_names[group.id] = group_data.native_joint_names

        composite_count = 0
        for size in range(2, len(groups) + 1):
            for group_tuple in combinations(groups, size):
                if not self._groups_are_non_overlapping(group_tuple):
                    continue
                composite_count += 1
                if composite_count > self._max_generated_composite_groups:
                    raise ValueError(
                        "Generated RoboPlan composite planning-group count exceeds "
                        f"max_generated_composite_groups={self._max_generated_composite_groups}"
                    )
                group_data = self._make_roboplan_group_data(group_tuple)
                self._roboplan_groups_by_selection[frozenset(group_data.group_ids)] = group_data

        self._full_native_joint_names = tuple(
            native_joint_name
            for robot in self._robots.values()
            for native_joint_name in self._movable_native_joint_names_by_robot.get(
                robot.config.name,
                tuple(
                    self._native_names_by_robot[robot.config.name].joint(joint_name)
                    for joint_name in robot.config.joint_names
                ),
            )
        )

    def _make_roboplan_group_data(self, groups: tuple[PlanningGroup, ...]) -> _RoboPlanGroupData:
        native_joint_names: list[str] = []
        native_to_global: dict[str, str] = {}
        for group in groups:
            native_map = self._native_names_by_robot[group.robot_name]
            for local_name, global_name in zip(
                group.local_joint_names, group.joint_names, strict=True
            ):
                native_name = native_map.joint(local_name)
                if native_name in native_to_global:
                    raise ValueError(
                        f"Duplicate RoboPlan-native joint name in planning groups: {native_name}"
                    )
                native_joint_names.append(native_name)
                native_to_global[native_name] = global_name
        group_ids = tuple(group.id for group in groups)
        if len(groups) == 1 and not self._uses_composite_model:
            group_name = groups[0].group_name
        else:
            group_name = self._roboplan_group_name(group_ids)
        return _RoboPlanGroupData(
            group_ids=group_ids,
            group_name=group_name,
            native_joint_names=tuple(native_joint_names),
            native_to_global_joint_name=native_to_global,
        )

    def _groups_are_non_overlapping(self, groups: tuple[PlanningGroup, ...]) -> bool:
        seen: set[str] = set()
        for group in groups:
            overlap = seen.intersection(group.joint_names)
            if overlap:
                return False
            seen.update(group.joint_names)
        return True

    def _generate_composite_srdf(self, prepared_urdfs: dict[RobotName, Path]) -> str:
        lines = ['<robot name="dimos_composite">']
        for group_data in self._roboplan_groups_by_selection.values():
            lines.append(f'  <group name="{escape(group_data.group_name)}">')
            for native_joint_name in group_data.native_joint_names:
                lines.append(f'    <joint name="{escape(native_joint_name)}"/>')
            lines.append("  </group>")
        for link1, link2 in self._composite_collision_exclusion_pairs(prepared_urdfs):
            lines.append(
                f'  <disable_collisions link1="{escape(link1)}" link2="{escape(link2)}" '
                'reason="DimOS configured"/>'
            )
        lines.append("</robot>")
        return "\n".join(lines) + "\n"

    def _composite_collision_exclusion_pairs(
        self, prepared_urdfs: dict[RobotName, Path]
    ) -> list[tuple[str, str]]:
        pairs: set[tuple[str, str]] = set()
        for robot in self._robots.values():
            native_map = self._native_names_by_robot[robot.config.name]
            local_pairs = set(robot.config.collision_exclusion_pairs)
            local_pairs.update(
                self._adjacent_link_pairs_from_urdf(prepared_urdfs[robot.config.name])
            )
            local_pairs.update(self._disable_collision_pairs_from_srdf(robot.config.srdf_path))
            for link1, link2 in local_pairs:
                try:
                    native_pair = tuple(sorted((native_map.link(link1), native_map.link(link2))))
                except KeyError:
                    logger.debug(
                        f"Skipping unmapped RoboPlan collision exclusion pair: {robot.config.name}:"
                        f" {link1} <-> {link2}"
                    )
                    continue
                pairs.add(native_pair)  # type: ignore[arg-type]
        return sorted(pairs)

    def _disable_collision_pairs_from_srdf(self, srdf_path: Path | None) -> list[tuple[str, str]]:
        if srdf_path is None:
            return []
        try:
            root = ET.parse(srdf_path).getroot()
        except (ET.ParseError, FileNotFoundError):
            return []
        pairs: list[tuple[str, str]] = []
        for disable in root.findall("disable_collisions"):
            link1 = disable.get("link1")
            link2 = disable.get("link2")
            if link1 and link2:
                pairs.append((link1, link2))
        return pairs

    def _apply_collision_exclusions_from_srdf(self, scene: Any, srdf_path: Path) -> None:
        for link1, link2 in self._disable_collision_pairs_from_srdf(srdf_path):
            try:
                scene.setCollisions(link1, link2, False)
            except RuntimeError:
                logger.debug(
                    f"RoboPlan did not accept collision exclusion pair: {link1} <-> {link2}"
                )
            except AttributeError:
                return

    def _composite_package_paths(self) -> list[str]:
        paths: list[str] = []
        for robot in self._robots.values():
            for path in robot.config.package_paths.values():
                path_text = str(path)
                if path_text not in paths:
                    paths.append(path_text)
        return paths

    def _native_name(self, robot_name: RobotName, local_name: str) -> str:
        return f"{self._safe_name(robot_name)}{_NATIVE_NAME_SEPARATOR}{self._safe_name(local_name)}"

    def _roboplan_group_name(self, group_ids: Iterable[PlanningGroupID]) -> str:
        return (
            _COMPOSITE_GROUP_PREFIX
            + "__"
            + "__".join(self._safe_name(group_id) for group_id in group_ids)
        )

    def _safe_name(self, value: str) -> str:
        return value.replace("/", "_").replace(":", "_").replace(" ", "_")

    def _validate_planning_group_metadata(
        self,
        scene: Any,
    ) -> dict[PlanningGroupID, tuple[str, ...]]:
        native_joint_names: dict[PlanningGroupID, tuple[str, ...]] = {}
        updated_group_data: dict[frozenset[PlanningGroupID], _RoboPlanGroupData] = {}
        for selection_key, group_data in self._roboplan_groups_by_selection.items():
            try:
                group_info = scene.getJointGroupInfo(group_data.group_name)
            except AttributeError as exc:
                raise ValueError("RoboPlan scene does not expose planning group metadata") from exc
            native_names = tuple(group_info.joint_names)
            if len(set(native_names)) != len(native_names):
                raise ValueError(
                    f"RoboPlan returned duplicate joint names for planning group '{group_data.group_name}'"
                )
            if set(native_names) != set(group_data.native_joint_names):
                raise ValueError(
                    "RoboPlan planning group joint names do not match configured "
                    f"planning group '{group_data.group_name}': RoboPlan={list(native_names)}, "
                    f"configured={list(group_data.native_joint_names)}"
                )
            updated_group_data[selection_key] = replace(group_data, native_joint_names=native_names)
            if len(group_data.group_ids) == 1:
                native_joint_names[group_data.group_ids[0]] = native_names
        self._roboplan_groups_by_selection.update(updated_group_data)
        return native_joint_names

    def _collision_exclusion_pairs(
        self, config: RobotModelConfig, urdf_path: Path
    ) -> list[tuple[str, str]]:
        pairs = set(config.collision_exclusion_pairs)
        pairs.update(self._adjacent_link_pairs_from_urdf(urdf_path))
        return sorted(pairs)

    def _adjacent_link_pairs_from_urdf(self, urdf_path: Path) -> list[tuple[str, str]]:
        try:
            root = ET.parse(urdf_path).getroot()
        except ET.ParseError as exc:
            raise ValueError(
                f"Unable to parse prepared URDF for SRDF generation: {urdf_path}"
            ) from exc

        pairs: list[tuple[str, str]] = []
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            parent_link = parent.get("link") if parent is not None else None
            child_link = child.get("link") if child is not None else None
            if parent_link and child_link:
                pairs.append((parent_link, child_link))
        return pairs

    def _apply_collision_exclusions(
        self, scene: Any, config: RobotModelConfig, urdf_path: Path
    ) -> None:
        for link1, link2 in self._collision_exclusion_pairs(config, urdf_path):
            try:
                scene.setCollisions(link1, link2, False)
            except RuntimeError:
                logger.debug(
                    f"RoboPlan did not accept collision exclusion pair: {link1} <-> {link2}"
                )
            except AttributeError:
                return

    def _extract_joint_limits(
        self, config: RobotModelConfig, model_handle: Any
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        configured_limits = self._configured_joint_limits(config)
        if configured_limits is not None:
            lower, upper = configured_limits
        else:
            limits = self._query_scene_joint_limits(config, model_handle)
            if limits is None:
                raise ValueError(
                    "RoboPlanWorld requires explicit joint_limits_lower/joint_limits_upper "
                    "when limits cannot be read from RoboPlan bindings"
                )
            lower, upper = limits
        if len(lower) != len(config.joint_names) or len(upper) != len(config.joint_names):
            raise ValueError("Joint limit length must match joint_names length")
        if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
            raise ValueError("RoboPlanWorld requires finite joint limits")
        return lower, upper

    def _configured_joint_limits(
        self, config: RobotModelConfig
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        if config.joint_limits_lower is None or config.joint_limits_upper is None:
            return None
        lower = np.asarray(config.joint_limits_lower, dtype=np.float64)
        upper = np.asarray(config.joint_limits_upper, dtype=np.float64)
        if len(lower) != len(config.joint_names) or len(upper) != len(config.joint_names):
            raise ValueError("Joint limit length must match joint_names length")
        if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
            raise ValueError("RoboPlanWorld requires finite joint limits")
        return lower, upper

    def _query_scene_joint_limits(
        self, config: RobotModelConfig, model_handle: Any
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        _ = model_handle
        scene = self._require_scene()
        joint_order = self._query_scene_joint_limit_order(scene, config)
        if joint_order is None:
            return None
        group_data = self._joint_limit_group_data(config)
        lower, upper = scene.getPositionLimitVectors(group_data.group_name, False)
        lower_array = np.asarray(lower, dtype=np.float64)
        upper_array = np.asarray(upper, dtype=np.float64)
        if len(joint_order) != len(lower_array) or len(joint_order) != len(upper_array):
            raise ValueError(
                "RoboPlan joint limit order length does not match returned limit vectors"
            )
        if len(set(joint_order)) != len(joint_order):
            raise ValueError("RoboPlan returned duplicate joint names for joint limits")
        native_map = self._native_names_by_robot[config.name]
        configured_native_names = [
            native_map.joint(joint_name) for joint_name in config.joint_names
        ]
        if set(joint_order) != set(configured_native_names):
            raise ValueError(
                "RoboPlan joint limit names do not match configured joint_names: "
                f"RoboPlan={joint_order}, configured={configured_native_names}"
            )
        order_indices = [joint_order.index(joint_name) for joint_name in configured_native_names]
        return lower_array[order_indices], upper_array[order_indices]

    def _query_scene_joint_limit_order(
        self, scene: Any, config: RobotModelConfig
    ) -> list[str] | None:
        try:
            group_info = scene.getJointGroupInfo(self._joint_limit_group_data(config).group_name)
        except AttributeError:
            return None
        return list(group_info.joint_names)

    def _joint_limit_group_data(self, config: RobotModelConfig) -> _RoboPlanGroupData:
        for group in self._planning_groups.groups_for_robot(config.name):
            if set(group.local_joint_names) == set(config.joint_names):
                return self._group_data_for_selection_ids((group.id,))
        raise ValueError(
            "RoboPlanWorld requires explicit joint_limits_lower/joint_limits_upper when no "
            "configured planning group covers RobotModelConfig.joint_names"
        )

    def _get_robot(self, robot_id: WorldRobotID) -> _RoboPlanRobotData:
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")
        return self._robots[robot_id]

    def _joint_state_to_q(
        self, robot_id: WorldRobotID, joint_state: JointState
    ) -> NDArray[np.float64]:
        robot = self._get_robot(robot_id)
        if len(joint_state.position) != len(robot.config.joint_names):
            raise ValueError("JointState position length must match configured joint count")
        if not joint_state.name:
            return np.asarray(joint_state.position, dtype=np.float64)
        name_to_pos: dict[str, float] = {}
        for name, position in zip(joint_state.name, joint_state.position, strict=True):
            local_name = self._to_robot_local_joint_name(robot.config, name)
            if local_name in name_to_pos:
                raise ValueError(
                    f"JointState contains duplicate joint for RoboPlanWorld: {local_name}"
                )
            name_to_pos[local_name] = float(position)
        missing = [name for name in robot.config.joint_names if name not in name_to_pos]
        if missing:
            raise ValueError(f"JointState missing joints for RoboPlanWorld: {missing}")
        return np.asarray(
            [name_to_pos[name] for name in robot.config.joint_names], dtype=np.float64
        )

    def _to_robot_local_joint_name(self, config: RobotModelConfig, name: str) -> str:
        """Map accepted robot-scoped joint-state names to local model names."""
        if name in config.joint_names:
            return name
        prefix = f"{config.name}/"
        if name.startswith(prefix):
            local_name = name.removeprefix(prefix)
            if local_name in config.joint_names:
                return local_name
        return name

    def _resolve_group(
        self, group_id: PlanningGroupID
    ) -> tuple[PlanningGroup, WorldRobotID, _RoboPlanRobotData, tuple[str, ...]]:
        group = self._planning_groups.get(group_id)
        try:
            robot_id = self._robot_ids_by_name[group.robot_name]
        except KeyError as exc:
            raise KeyError(f"Robot for planning group '{group_id}' is not registered") from exc
        robot = self._get_robot(robot_id)
        try:
            native_joint_names = self._roboplan_native_joint_names[group.id]
        except KeyError as exc:
            raise KeyError(
                f"RoboPlan native order for planning group '{group_id}' is missing"
            ) from exc
        return group, robot_id, robot, native_joint_names

    def _validate_supported_selection(
        self, selection: PlanningGroupSelection
    ) -> PlanningResult | None:
        try:
            canonical_groups = tuple(
                self._planning_groups.get(group.id) for group in selection.groups
            )
        except KeyError as exc:
            return PlanningResult(status=PlanningStatus.UNSUPPORTED, message=str(exc))
        expected_joint_names = tuple(
            joint_name for group in canonical_groups for joint_name in group.joint_names
        )
        if tuple(selection.joint_names) != expected_joint_names:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message=(
                    "RoboPlan-native planning requires the selection joints to exactly match "
                    "the selected planning groups"
                ),
            )
        if frozenset(selection.group_ids) not in self._roboplan_groups_by_selection:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planning has no generated group for this selection",
            )
        return None

    def _group_data_for_selection_ids(
        self, group_ids: Iterable[PlanningGroupID]
    ) -> _RoboPlanGroupData:
        key = frozenset(group_ids)
        try:
            return self._roboplan_groups_by_selection[key]
        except KeyError as exc:
            raise KeyError(f"No RoboPlan group generated for selection {sorted(key)}") from exc

    def _project_robot_q_to_native_group(
        self,
        group: PlanningGroup,
        robot: _RoboPlanRobotData,
        native_joint_names: tuple[str, ...],
        q: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        if len(q) != len(robot.config.joint_names):
            raise ValueError(
                f"Robot state for '{robot.robot_id}' has {len(q)} positions, "
                f"expected {len(robot.config.joint_names)}"
            )
        native_map = self._native_names_by_robot[robot.config.name]
        positions_by_name = {
            native_map.joint(joint_name): position
            for joint_name, position in zip(robot.config.joint_names, q, strict=True)
        }
        missing = [name for name in native_joint_names if name not in positions_by_name]
        if missing:
            raise ValueError(
                f"Planning group '{group.id}' has joints outside robot state: {missing}"
            )
        return np.asarray(
            [positions_by_name[name] for name in native_joint_names], dtype=np.float64
        )

    def _joint_state_to_native_group_q(
        self,
        group: PlanningGroup,
        native_joint_names: tuple[str, ...],
        joint_state: JointState,
    ) -> NDArray[np.float64]:
        global_state = joint_target_to_global_names(group, joint_state)
        native_map = self._native_names_by_robot[group.robot_name]
        positions_by_native_name = {
            native_map.joint(local_name): position
            for local_name, position in zip(
                group.local_joint_names, global_state.position, strict=True
            )
        }
        missing = [name for name in native_joint_names if name not in positions_by_native_name]
        if missing:
            raise ValueError(f"JointState for '{group.id}' is missing joints: {missing}")
        return np.asarray([positions_by_native_name[name] for name in native_joint_names])

    def _joint_state_to_native_selection_q(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        joint_state: JointState,
    ) -> NDArray[np.float64]:
        positions_by_global = self._selection_positions_by_global(selection, joint_state)
        missing = [
            native_name
            for native_name in group_data.native_joint_names
            if group_data.native_to_global_joint_name[native_name] not in positions_by_global
        ]
        if missing:
            raise ValueError(f"JointState for selection is missing native joints: {missing}")
        return np.asarray(
            [
                positions_by_global[group_data.native_to_global_joint_name[native_name]]
                for native_name in group_data.native_joint_names
            ],
            dtype=np.float64,
        )

    def _selection_positions_by_global(
        self, selection: PlanningGroupSelection, joint_state: JointState
    ) -> dict[str, float]:
        if len(joint_state.position) != len(selection.joint_names):
            raise ValueError("JointState position length must match selected joint count")
        if not joint_state.name:
            return {
                global_name: float(position)
                for global_name, position in zip(
                    selection.joint_names, joint_state.position, strict=True
                )
            }
        if len(selection.groups) == 1:
            group_state = joint_target_to_global_names(selection.groups[0], joint_state)
            return dict(zip(group_state.name, group_state.position, strict=True))
        if not all(name in selection.joint_names for name in joint_state.name):
            raise ValueError(
                "Multi-group RoboPlan selections require JointState names in global selection order"
            )
        positions_by_global: dict[str, float] = {}
        for name, position in zip(joint_state.name, joint_state.position, strict=True):
            if name in positions_by_global:
                raise ValueError(f"JointState contains duplicate selected joint: {name}")
            positions_by_global[name] = float(position)
        missing = [name for name in selection.joint_names if name not in positions_by_global]
        if missing:
            raise ValueError(f"JointState missing selected joints: {missing}")
        return positions_by_global

    def _selection_path_to_robot_joint_path(
        self,
        selection: PlanningGroupSelection,
        group: PlanningGroup,
        robot: _RoboPlanRobotData,
        path: list[JointState],
    ) -> list[JointState]:
        result: list[JointState] = []
        for state in path:
            positions_by_global = self._selection_positions_by_global(selection, state)
            result.append(
                JointState(
                    name=list(robot.config.joint_names),
                    position=[
                        positions_by_global[global_name] for global_name in group.joint_names
                    ],
                )
            )
        return result

    def _validate_selected_start_matches_belief(
        self, selection: PlanningGroupSelection, start: JointState
    ) -> PlanningResult | None:
        try:
            start_by_global = self._selection_positions_by_global(selection, start)
            belief_by_global = self._selection_belief_positions(selection)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        for global_name in selection.joint_names:
            if (
                abs(start_by_global[global_name] - belief_by_global[global_name])
                > self._selected_start_tolerance
            ):
                return PlanningResult(
                    status=PlanningStatus.INVALID_START,
                    message=(
                        "Selected start state does not match RoboPlan Planning world belief for "
                        f"{global_name}"
                    ),
                )
        return None

    def _selection_belief_positions(self, selection: PlanningGroupSelection) -> dict[str, float]:
        positions: dict[str, float] = {}
        for group in selection.groups:
            robot_id = self._robot_ids_by_name[group.robot_name]
            robot = self._get_robot(robot_id)
            robot_q = self._live_context.q_by_robot.get(robot_id)
            if robot_q is None:
                robot_q = np.zeros(len(robot.config.joint_names), dtype=np.float64)
            local_positions = dict(zip(robot.config.joint_names, robot_q, strict=True))
            for local_name, global_name in zip(
                group.local_joint_names, group.joint_names, strict=True
            ):
                positions[global_name] = float(local_positions[local_name])
        return positions

    def _selection_path_collision_free(
        self,
        selection: PlanningGroupSelection,
        path: list[JointState],
        step_size: float = 0.02,
    ) -> bool:
        if not path:
            return False
        if len(path) == 1:
            return self._selection_config_collision_free(selection, path[0])
        for start, end in pairwise(path):
            if not self._selection_edge_collision_free(selection, start, end, step_size):
                return False
        return True

    def _selection_config_collision_free(
        self, selection: PlanningGroupSelection, joint_state: JointState
    ) -> bool:
        with self.scratch_context() as ctx:
            self._set_selection_state(ctx, selection, joint_state)
            return self._selected_robots_collision_free(ctx, selection)

    def _selection_edge_collision_free(
        self,
        selection: PlanningGroupSelection,
        start: JointState,
        end: JointState,
        step_size: float,
    ) -> bool:
        start_by_global = self._selection_positions_by_global(selection, start)
        end_by_global = self._selection_positions_by_global(selection, end)
        q_start = np.asarray(
            [start_by_global[name] for name in selection.joint_names], dtype=np.float64
        )
        q_end = np.asarray(
            [end_by_global[name] for name in selection.joint_names], dtype=np.float64
        )
        dist = float(np.linalg.norm(q_end - q_start))
        if dist < 1e-8:
            return self._selection_config_collision_free(selection, start)
        n_steps = max(2, int(np.ceil(dist / step_size)) + 1)
        with self.scratch_context() as ctx:
            for i in range(n_steps):
                t = i / (n_steps - 1)
                q = q_start + t * (q_end - q_start)
                self._set_selection_state(
                    ctx,
                    selection,
                    JointState(name=list(selection.joint_names), position=q.astype(float).tolist()),
                )
                if not self._selected_robots_collision_free(ctx, selection):
                    return False
        return True

    def _set_selection_state(
        self,
        ctx: RoboPlanContext,
        selection: PlanningGroupSelection,
        joint_state: JointState,
    ) -> None:
        positions_by_global = self._selection_positions_by_global(selection, joint_state)
        for group in selection.groups:
            robot_id = self._robot_ids_by_name[group.robot_name]
            robot = self._get_robot(robot_id)
            robot_q = ctx.q_by_robot.get(robot_id)
            if robot_q is None:
                robot_q = np.zeros(len(robot.config.joint_names), dtype=np.float64)
            else:
                robot_q = robot_q.copy()
            index_by_local = {name: index for index, name in enumerate(robot.config.joint_names)}
            for local_name, global_name in zip(
                group.local_joint_names, group.joint_names, strict=True
            ):
                robot_q[index_by_local[local_name]] = positions_by_global[global_name]
            ctx.q_by_robot[robot_id] = robot_q

    def _selected_robots_collision_free(
        self, ctx: RoboPlanContext, selection: PlanningGroupSelection
    ) -> bool:
        robot_ids = {self._robot_ids_by_name[group.robot_name] for group in selection.groups}
        return all(self.is_collision_free(ctx, robot_id) for robot_id in robot_ids)

    def _overlay_native_group_q_on_robot_q(
        self,
        group: PlanningGroup,
        robot: _RoboPlanRobotData,
        native_joint_names: tuple[str, ...],
        robot_q: NDArray[np.float64],
        native_q: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        if len(native_q) != len(native_joint_names):
            raise ValueError(
                f"Planning group '{group.id}' has {len(native_q)} positions, "
                f"expected {len(native_joint_names)}"
            )
        result = np.asarray(robot_q, dtype=np.float64).copy()
        native_map = self._native_names_by_robot[robot.config.name]
        robot_indices = {
            native_map.joint(name): index for index, name in enumerate(robot.config.joint_names)
        }
        for native_name, position in zip(native_joint_names, native_q, strict=True):
            try:
                result[robot_indices[native_name]] = position
            except KeyError as exc:
                raise ValueError(
                    f"Planning group '{group.id}' has joint outside robot state: {native_name}"
                ) from exc
        return result

    def _scene_q_for_group(
        self, group: PlanningGroup, robot_id: WorldRobotID, robot_q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        ctx = RoboPlanContext(q_by_robot={robot_id: np.asarray(robot_q, dtype=np.float64)})
        for other_robot_id, q in self._live_context.q_by_robot.items():
            ctx.q_by_robot.setdefault(other_robot_id, q)
        return self._full_scene_q(ctx)

    def _native_path_to_public_group_path(
        self,
        group: PlanningGroup,
        native_joint_names: tuple[str, ...],
        path_arrays: list[NDArray[np.float64]],
        result_joint_names: tuple[str, ...] | None,
    ) -> list[JointState]:
        names_for_positions = result_joint_names or native_joint_names
        if len(set(names_for_positions)) != len(names_for_positions):
            raise ValueError("RoboPlan path returned duplicate joint names")
        if set(names_for_positions) != set(group.local_joint_names):
            raise ValueError(
                "RoboPlan path joint names do not match selected planning group: "
                f"RoboPlan={list(names_for_positions)}, configured={list(group.local_joint_names)}"
            )
        path: list[JointState] = []
        for q in path_arrays:
            q_array = np.asarray(q, dtype=np.float64)
            if len(q_array) != len(names_for_positions):
                raise ValueError(
                    "RoboPlan path waypoint length does not match returned joint names: "
                    f"{len(q_array)} != {len(names_for_positions)}"
                )
            positions_by_name = dict(zip(names_for_positions, q_array, strict=True))
            path.append(
                JointState(
                    name=list(group.joint_names),
                    position=[float(positions_by_name[name]) for name in group.local_joint_names],
                )
            )
        return path

    def _native_path_to_public_selection_path(
        self,
        selection: PlanningGroupSelection,
        group_data: _RoboPlanGroupData,
        path_arrays: list[NDArray[np.float64]],
        result_joint_names: tuple[str, ...] | None,
    ) -> list[JointState]:
        names_for_positions = result_joint_names or group_data.native_joint_names
        if len(set(names_for_positions)) != len(names_for_positions):
            raise ValueError("RoboPlan path returned duplicate joint names")
        if set(names_for_positions) != set(group_data.native_joint_names):
            raise ValueError(
                "RoboPlan path joint names do not match selected planning group: "
                f"RoboPlan={list(names_for_positions)}, "
                f"configured={list(group_data.native_joint_names)}"
            )
        native_by_global = {
            global_name: native_name
            for native_name, global_name in group_data.native_to_global_joint_name.items()
        }
        path: list[JointState] = []
        for q in path_arrays:
            q_array = np.asarray(q, dtype=np.float64)
            if len(q_array) != len(names_for_positions):
                raise ValueError(
                    "RoboPlan path waypoint length does not match returned joint names: "
                    f"{len(q_array)} != {len(names_for_positions)}"
                )
            positions_by_native = dict(zip(names_for_positions, q_array, strict=True))
            path.append(
                JointState(
                    name=list(selection.joint_names),
                    position=[
                        float(positions_by_native[native_by_global[global_name]])
                        for global_name in selection.joint_names
                    ],
                )
            )
        return path

    def _reorder_jacobian_columns(
        self,
        group: PlanningGroup,
        robot_joint_names: list[str],
        native_joint_names: tuple[str, ...],
        jacobian: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        native_group_order = tuple(
            self._native_names_by_robot[group.robot_name].joint(name)
            for name in group.local_joint_names
        )
        if jacobian.shape[1] == len(native_joint_names):
            column_names = native_joint_names
        elif jacobian.shape[1] == len(self._full_native_joint_names):
            column_names = self._full_native_joint_names
        elif jacobian.shape[1] == len(robot_joint_names):
            native_map = self._native_names_by_robot[group.robot_name]
            column_names = tuple(native_map.joint(name) for name in robot_joint_names)
        else:
            raise ValueError(
                f"Unexpected RoboPlan Jacobian shape: {jacobian.shape}; cannot map columns "
                f"for planning group '{group.id}'"
            )
        column_indices = [column_names.index(name) for name in native_group_order]
        return jacobian[:, column_indices]

    def _native_link_name(self, config: RobotModelConfig, local_link_name: str) -> str:
        return self._native_names_by_robot[config.name].link(local_link_name)

    def _require_finalized(self) -> None:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

    def _require_scene(self) -> Any:
        if self._scene is None:
            raise RuntimeError("RoboPlan scene is not initialized; finalize the world first")
        return self._scene

    def _to_scene_q(
        self,
        robot_id: WorldRobotID,
        q: NDArray[np.float64],
        ctx: RoboPlanContext | None = None,
    ) -> NDArray[np.float64]:
        """Expand DimOS group positions to RoboPlan's full scene vector when available."""
        robot = self._get_robot(robot_id)
        if len(q) != len(robot.config.joint_names):
            return np.asarray(q, dtype=np.float64)
        return self._full_scene_q(
            ctx if ctx is not None else self._live_context, overlay=(robot_id, q)
        )

    def _full_scene_q(
        self,
        ctx: RoboPlanContext,
        overlay: tuple[WorldRobotID, NDArray[np.float64]] | None = None,
    ) -> NDArray[np.float64]:
        positions_by_native: dict[str, float] = {}
        for robot_id, robot in self._robots.items():
            q = ctx.q_by_robot.get(robot_id)
            if q is None:
                q = np.zeros(len(robot.config.joint_names), dtype=np.float64)
            if overlay is not None and overlay[0] == robot_id:
                q = overlay[1]
            if len(q) != len(robot.config.joint_names):
                raise ValueError(
                    f"Robot state for '{robot_id}' has {len(q)} positions, "
                    f"expected {len(robot.config.joint_names)}"
                )
            native_map = self._native_names_by_robot[robot.config.name]
            for joint_name, position in zip(robot.config.joint_names, q, strict=True):
                positions_by_native[native_map.joint(joint_name)] = float(position)
        return np.asarray(
            [positions_by_native.get(name, 0.0) for name in self._full_native_joint_names],
            dtype=np.float64,
        )

    def _set_scene_current_q(self, ctx: RoboPlanContext) -> None:
        scene = self._require_scene()
        setter = getattr(scene, "setJointPositions", None)
        if setter is not None:
            setter(self._full_scene_q(ctx))

    def _full_robot_planning_group(self, robot_name: RobotName) -> PlanningGroup | None:
        try:
            robot_id = self._robot_ids_by_name[robot_name]
        except KeyError:
            return None
        robot = self._get_robot(robot_id)
        robot_joint_set = set(robot.config.joint_names)
        for group in self._planning_groups.groups_for_robot(robot_name):
            if set(group.local_joint_names) == robot_joint_set:
                return group
        return None

    def _has_collisions(
        self, robot_id: WorldRobotID, q: NDArray[np.float64], ctx: RoboPlanContext
    ) -> bool:
        scene = self._require_scene()
        scene_q = self._to_scene_q(robot_id, q, ctx)
        return bool(scene.hasCollisions(scene_q))

    def _call_path_collision_checker(
        self,
        ctx: RoboPlanContext,
        robot_id: WorldRobotID,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
        step_size: float,
    ) -> bool:
        scene = self._require_scene()
        scene_q_start = self._to_scene_q(robot_id, q_start, ctx)
        scene_q_end = self._to_scene_q(robot_id, q_end, ctx)
        return bool(
            roboplan_core.hasCollisionsAlongPath(
                scene,
                scene_q_start,
                scene_q_end,
                step_size,
                False,
                True,
            )
        )

    def _add_obstacle_to_scene(self, obstacle: Obstacle, obstacle_id: str) -> Any:
        scene = self._require_scene()
        matrix = pose_to_matrix(obstacle.pose)
        if obstacle.obstacle_type == ObstacleType.BOX:
            self._require_dimensions(obstacle, 3)
            width, height, depth = obstacle.dimensions
            return scene.addBoxGeometry(obstacle_id, width, height, depth, matrix)
        if obstacle.obstacle_type == ObstacleType.SPHERE:
            self._require_dimensions(obstacle, 1)
            (radius,) = obstacle.dimensions
            return scene.addSphereGeometry(obstacle_id, radius, matrix)
        if obstacle.obstacle_type == ObstacleType.CYLINDER:
            self._require_dimensions(obstacle, 2)
            radius, length = obstacle.dimensions
            return scene.addCylinderGeometry(obstacle_id, radius, length, matrix)
        if obstacle.obstacle_type == ObstacleType.MESH:
            if not obstacle.mesh_path:
                raise ValueError("MESH obstacle requires mesh_path")
            return scene.addMeshGeometry(obstacle_id, obstacle.mesh_path, matrix)
        raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")

    def _require_dimensions(self, obstacle: Obstacle, n_dims: int) -> None:
        if len(obstacle.dimensions) != n_dims:
            raise ValueError(
                f"{obstacle.obstacle_type.name} obstacle requires {n_dims} dimensions, "
                f"got {len(obstacle.dimensions)}"
            )

    def _run_native_rrt(
        self,
        group_name: str,
        native_joint_names: tuple[str, ...],
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float,
    ) -> Any:
        scene = self._require_scene()
        options = roboplan_rrt.RRTOptions()
        options.group_name = group_name
        options.max_planning_time = timeout
        options.collision_check_use_bisection = True
        if hasattr(options, "collision_check_step_size"):
            options.collision_check_step_size = 0.02
        planner = roboplan_rrt.RRT(scene, options)
        start_config = self._to_native_joint_configuration(native_joint_names, q_start)
        goal_config = self._to_native_joint_configuration(native_joint_names, q_goal)
        result = planner.plan(start_config, goal_config)
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        return result

    def _to_native_joint_configuration(
        self, native_joint_names: tuple[str, ...], q: NDArray[np.float64]
    ) -> Any:
        return roboplan_core.JointConfiguration(
            list(native_joint_names), np.asarray(q, dtype=np.float64)
        )

    def _extract_native_path(
        self, result: Any
    ) -> tuple[tuple[str, ...] | None, list[NDArray[np.float64]]]:
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        if isinstance(result, (list, tuple)):
            return None, [np.asarray(q, dtype=np.float64) for q in result]
        result_joint_names = getattr(result, "joint_names", None)
        names = tuple(result_joint_names) if result_joint_names else None
        return names, [np.asarray(q, dtype=np.float64) for q in result.positions]
