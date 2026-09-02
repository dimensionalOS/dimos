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

"""RoboPlan-native motion planner bound to a :class:`RoboPlanWorld`."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from itertools import pairwise
import time
from typing import TYPE_CHECKING, Any

import numpy as np

try:
    import roboplan.cartesian_planning as roboplan_cartesian
    import roboplan.core as roboplan_core
    import roboplan.rrt as roboplan_rrt
except ImportError as exc:
    raise ImportError(
        "RoboPlanPlanner requires the optional roboplan dependency. "
        "Install the manipulation extra before selecting the roboplan backend."
    ) from exc

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig,
    RoboPlanPlannerConfig,
)
from dimos.manipulation.planning.planners.selected_joint_space import normalize_selection_target
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import (
    CartesianTarget,
    PlanningGroupID,
    PlanningResult,
)
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.manipulation.planning.world.roboplan_model import (
    ROBOPLAN_WORLD_FRAME,
    RoboPlanGroup,
)
from dimos.manipulation.planning.world.roboplan_world import (
    RoboPlanContext,
    RoboPlanWorld,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.protocols import WorldSpec

logger = setup_logger()

_CARTESIAN_COLLISION_STEP_SIZE = 0.05


class RoboPlanPlanner:
    """PlannerSpec implementation using a specific RoboPlan world scene."""

    def __init__(self, world: WorldSpec, config: RoboPlanPlannerConfig) -> None:
        if not isinstance(world, RoboPlanWorld):
            raise TypeError("RoboPlanPlanner requires a RoboPlanWorld")
        self._world = world
        self._config = config.model_copy(deep=True)

    def plan_joint_path(
        self,
        world: WorldSpec,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a path for the configured model's canonical joints."""
        if world is not self._world:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        try:
            q_start = self._world.ordered_joint_positions(start)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        try:
            q_goal = self._world.ordered_joint_positions(goal)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))
        if not self._world.is_ready():
            return PlanningResult(
                status=PlanningStatus.INVALID_START,
                message="RoboPlan planning scene is not ready: authoritative state is incomplete",
            )
        config = self._world.get_model_config()
        group = self._world.all_planning_group()
        with self._world.scratch_context() as ctx:
            self._world.set_joint_state(
                ctx,
                JointState(name=list(config.joint_names), position=q_start.tolist()),
            )
            return self._plan_group(
                ctx,
                group,
                dict(zip(config.joint_names, q_start, strict=True)),
                dict(zip(config.joint_names, q_goal, strict=True)),
                timeout,
                5000,
            )

    def plan_selected_joint_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan one or more non-overlapping groups through RoboPlan RRT."""
        if world is not self._world:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        if not selection.groups:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="No planning groups selected",
            )
        group = self._world.planning_group(selection.group_ids)
        if group is None:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan has no generated group for this selection",
            )
        try:
            normalized_goal = normalize_selection_target(selection, goal, "goal")
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))
        try:
            normalized_start = self._normalize_selection_start(selection, start)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        start_by_name = dict(zip(normalized_start.name, normalized_start.position, strict=True))
        with self._world.scratch_context() as ctx:
            self._apply_selected_state(ctx, normalized_start)
            return self._plan_group(
                ctx,
                group,
                start_by_name,
                dict(zip(normalized_goal.name, normalized_goal.position, strict=True)),
                timeout,
                max_iterations,
                output_names=selection.joint_names,
            )

    def plan_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        targets: Mapping[PlanningGroupID, CartesianTarget],
        config: RoboPlanCartesianPathConfig,
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        check_collision: bool = True,
    ) -> PlanningResult:
        """Plan synchronized TCP waypoint paths with official RoboPlan planning."""
        started = time.time()
        if world is not self._world:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        validation_error = self._validate_cartesian_request(selection, targets, auxiliary_groups)
        if validation_error is not None:
            return validation_error
        try:
            normalized_start = self._normalize_selection_start(selection, start)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))

        group = self._world.planning_group(selection.group_ids)
        if group is None:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan has no generated group for this selection",
            )

        try:
            with self._world.scratch_context() as ctx:
                self._apply_selected_state(ctx, normalized_start)
                cartesian_path = self._build_cartesian_path(ctx, selection, targets)
                trajectory = self._run_cartesian_planner(
                    ctx,
                    group,
                    cartesian_path,
                    config,
                )
                path, timestamps = self._path_from_cartesian_trajectory(
                    selection,
                    group,
                    trajectory,
                )
                collision_validation_failed = (
                    check_collision
                    and bool(path)
                    and not (self._combined_path_collision_free(ctx, path))
                )
        except (KeyError, RuntimeError, ValueError) as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message=f"RoboPlan Cartesian planning failed: {exc}",
            )

        if not path:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message="RoboPlan Cartesian planning failed: returned an empty trajectory",
            )
        if collision_validation_failed:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message="RoboPlan Cartesian trajectory failed DimOS collision post-validation",
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - started,
            path_length=compute_path_length(path),
            message="RoboPlan Cartesian path found",
            timestamps=timestamps,
        )

    def get_name(self) -> str:
        """Get planner name."""
        return "RoboPlan"

    def _normalize_selection_start(
        self,
        selection: PlanningGroupSelection,
        start: JointState,
    ) -> JointState:
        """Validate readiness and normalize the request's authoritative start."""
        if not self._world.is_ready():
            raise ValueError(
                "RoboPlan planning scene is not ready: authoritative state is incomplete"
            )
        return normalize_selection_target(selection, start, "start")

    def _validate_cartesian_request(
        self,
        selection: PlanningGroupSelection,
        targets: Mapping[PlanningGroupID, CartesianTarget],
        auxiliary_groups: Sequence[PlanningGroupID],
    ) -> PlanningResult | None:
        if not selection.groups:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="No planning groups selected",
            )
        if not targets:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="Cartesian planning requires at least one target group",
            )
        auxiliary_ids = tuple(auxiliary_groups)
        if len(set(auxiliary_ids)) != len(auxiliary_ids):
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="Auxiliary planning groups must be unique",
            )
        target_ids = set(targets)
        auxiliary_set = set(auxiliary_ids)
        if target_ids & auxiliary_set:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="Target and auxiliary planning groups must be disjoint",
            )
        selected_ids = set(selection.group_ids)
        if target_ids | auxiliary_set != selected_ids:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message=(
                    "Target and auxiliary planning groups must exactly cover the "
                    "planning-group selection"
                ),
            )

        groups_by_id = {group.id: group for group in selection.groups}
        for group_id, target in targets.items():
            group = groups_by_id[group_id]
            if group.tip_link is None:
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=f"Planning group '{group_id}' has no TCP tip link",
                )
            if not isinstance(target, Sequence) or isinstance(target, (str, bytes)):
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=f"Cartesian target for '{group_id}' must be a waypoint sequence",
                )
            if len(target) < 2:
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=f"Cartesian target for '{group_id}' requires at least two waypoints",
                )
            waypoint_type = (
                PoseStamped
                if isinstance(target[0], PoseStamped)
                else Transform
                if isinstance(target[0], Transform)
                else None
            )
            if waypoint_type is None or any(
                not isinstance(waypoint, waypoint_type) for waypoint in target
            ):
                return PlanningResult(
                    status=PlanningStatus.INVALID_GOAL,
                    message=(
                        f"Cartesian target for '{group_id}' must contain only PoseStamped "
                        "waypoints or only Transform waypoints"
                    ),
                )
            if any(waypoint.frame_id != "world" for waypoint in target):
                return PlanningResult(
                    status=PlanningStatus.UNSUPPORTED,
                    message="Cartesian planning supports only world-frame waypoints",
                )
        return None

    def _apply_selected_state(self, ctx: RoboPlanContext, state: JointState) -> None:
        positions = dict(zip(state.name, state.position, strict=True))
        config = self._world.get_model_config()
        q = ctx.q.copy()
        for index, name in enumerate(config.joint_names):
            if name in positions:
                q[index] = positions[name]
        ctx.q = q

    def _build_cartesian_path(
        self,
        ctx: RoboPlanContext,
        selection: PlanningGroupSelection,
        targets: Mapping[PlanningGroupID, CartesianTarget],
    ) -> Any:
        base_frames: list[str] = []
        tip_frames: list[str] = []
        waypoint_paths: list[list[NDArray[np.float64]]] = []
        for group in selection.groups:
            target = targets.get(group.id)
            if target is None:
                continue
            if group.tip_link is None:
                raise ValueError(f"Planning group '{group.id}' has no TCP tip link")
            start_pose = self._world.get_group_ee_pose(ctx, group.id)
            start_matrix = np.asarray(pose_to_matrix(start_pose), dtype=np.float64)
            target_matrices = [
                self._resolve_cartesian_waypoint(start_matrix, waypoint) for waypoint in target
            ]
            if not np.allclose(target_matrices[0], start_matrix, atol=1e-6, rtol=0.0):
                raise ValueError(
                    f"Cartesian target for '{group.id}' must begin at its current TCP pose"
                )
            base_frames.append(ROBOPLAN_WORLD_FRAME)
            tip_frames.append(self._world.native_link_name(group.tip_link))
            waypoint_paths.append(target_matrices)
        return roboplan_core.CartesianPath(base_frames, tip_frames, waypoint_paths)

    def _resolve_cartesian_waypoint(
        self,
        start_matrix: NDArray[np.float64],
        waypoint: PoseStamped | Transform,
    ) -> NDArray[np.float64]:
        if isinstance(waypoint, PoseStamped):
            return np.asarray(pose_to_matrix(waypoint), dtype=np.float64)
        target_matrix = start_matrix.copy()
        delta_matrix = np.asarray(waypoint.to_matrix(), dtype=np.float64)
        target_matrix[:3, 3] += delta_matrix[:3, 3]
        target_matrix[:3, :3] = delta_matrix[:3, :3] @ start_matrix[:3, :3]
        return target_matrix

    def _run_cartesian_planner(
        self,
        ctx: RoboPlanContext,
        group: RoboPlanGroup,
        path: Any,
        config: RoboPlanCartesianPathConfig,
    ) -> Any:
        options = self._cartesian_planner_options(group.name, config)
        with self._world._lock:
            scene = self._world._require_scene()
            scene_q = self._world._full_scene_q(ctx)
            scene.setJointPositions(scene_q)
            q_start = roboplan_core.JointConfiguration(
                list(scene.getJointNames()),
                scene_q,
            )
            planner = roboplan_cartesian.CartesianPathPlanner(scene, options)
            trajectory = planner.plan(path, q_start)
        if trajectory is None:
            raise ValueError("official planner returned no trajectory")
        return trajectory

    @staticmethod
    def _cartesian_planner_options(
        group_name: str,
        config: RoboPlanCartesianPathConfig,
    ) -> Any:
        options = roboplan_cartesian.CartesianPlannerOptions()
        options.group_name = group_name
        options.speed_mode = {
            "bounded": roboplan_cartesian.CartesianSpeedMode.Bounded,
            "time_optimal": roboplan_cartesian.CartesianSpeedMode.TimeOptimal,
        }[config.speed_mode]
        options.dt = config.dt
        options.max_linear_speed = config.max_linear_speed
        options.max_angular_speed = config.max_angular_speed
        options.max_linear_acceleration = config.max_linear_acceleration
        options.max_angular_acceleration = config.max_angular_acceleration
        options.max_position_error = config.max_position_error
        options.max_orientation_error = config.max_orientation_error
        options.position_cost = config.position_cost
        options.orientation_cost = config.orientation_cost
        options.task_gain = config.task_gain
        options.lm_damping = config.lm_damping
        options.regularization = config.regularization
        options.config_task_weight = config.config_task_weight
        options.velocity_scale = config.velocity_scale
        options.acceleration_scale = config.acceleration_scale
        options.toppra_blend_deviation = config.toppra_blend_deviation
        options.position_limit_gain = config.position_limit_gain
        return options

    def _path_from_cartesian_trajectory(
        self,
        selection: PlanningGroupSelection,
        group: RoboPlanGroup,
        trajectory: Any,
    ) -> tuple[list[JointState], list[float]]:
        result_names = tuple(getattr(trajectory, "joint_names", ()))
        if not result_names:
            raise ValueError("RoboPlan trajectory does not identify its joints")
        if not set(group.native_names).issubset(result_names):
            raise ValueError("RoboPlan trajectory omits joints from the selected group")
        positions = list(trajectory.positions)
        velocities = list(trajectory.velocities)
        timestamps = [float(value) for value in trajectory.times]
        if not positions:
            return [], []
        if len(velocities) != len(positions) or len(timestamps) != len(positions):
            raise ValueError("RoboPlan trajectory fields have inconsistent waypoint counts")

        native_by_public = dict(zip(group.public_names, group.native_names, strict=True))
        expected_names = list(selection.joint_names)
        path: list[JointState] = []
        for position_row, velocity_row in zip(positions, velocities, strict=True):
            position_values = np.asarray(position_row, dtype=np.float64)
            velocity_values = np.asarray(velocity_row, dtype=np.float64)
            if len(position_values) != len(result_names) or len(velocity_values) != len(
                result_names
            ):
                raise ValueError(
                    "RoboPlan trajectory waypoint length does not match its joint names"
                )
            position_by_native = dict(zip(result_names, position_values, strict=True))
            velocity_by_native = dict(zip(result_names, velocity_values, strict=True))
            path.append(
                JointState(
                    name=expected_names,
                    position=[
                        float(position_by_native[native_by_public[name]]) for name in expected_names
                    ],
                    velocity=[
                        float(velocity_by_native[native_by_public[name]]) for name in expected_names
                    ],
                )
            )
        return path, timestamps

    def _combined_path_collision_free(
        self,
        ctx: RoboPlanContext,
        path: Sequence[JointState],
    ) -> bool:
        for start, end in pairwise(path):
            q_start = np.asarray(start.position, dtype=np.float64)
            q_end = np.asarray(end.position, dtype=np.float64)
            max_change = float(np.max(np.abs(q_end - q_start), initial=0.0))
            steps = max(1, int(np.ceil(max_change / _CARTESIAN_COLLISION_STEP_SIZE)))
            for fraction in np.linspace(0.0, 1.0, steps + 1):
                sample = JointState(
                    name=start.name,
                    position=(q_start + fraction * (q_end - q_start)).tolist(),
                )
                self._apply_selected_state(ctx, sample)
                if not self._world.is_collision_free(ctx):
                    return False
        if len(path) == 1:
            self._apply_selected_state(ctx, path[0])
            return self._world.is_collision_free(ctx)
        return True

    def _plan_group(
        self,
        ctx: RoboPlanContext,
        group: RoboPlanGroup,
        start_by_name: Mapping[str, float],
        goal_by_name: Mapping[str, float],
        timeout: float,
        max_iterations: int,
        *,
        output_names: Sequence[str] | None = None,
    ) -> PlanningResult:
        started = time.time()
        try:
            q_start = np.asarray(
                [start_by_name[name] for name in group.public_names],
                dtype=np.float64,
            )
            q_goal = np.asarray(
                [goal_by_name[name] for name in group.public_names],
                dtype=np.float64,
            )
        except KeyError as exc:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message=f"Joint target is missing '{exc.args[0]}'",
            )
        try:
            with self._world._lock:
                scene = self._world._require_scene()
                scene.setJointPositions(self._world._full_scene_q(ctx))
                result = self._run_native_rrt(
                    group,
                    q_start,
                    q_goal,
                    timeout,
                    max_iterations,
                )
                result = self._shortcut_native_path(group, result)
            path = self._path_from_native(group, result, output_names=output_names)
        except ValueError as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message=f"RoboPlan-native planning failed: {exc}",
            )
        if not path:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message="RoboPlan-native planning failed: returned an empty path",
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - started,
            path_length=compute_path_length(path),
            message="RoboPlan path found",
        )

    def _run_native_rrt(
        self,
        group: RoboPlanGroup,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float,
        max_iterations: int,
    ) -> Any:
        options: Any = roboplan_rrt.RRTOptions()
        options.group_name = group.name
        options.max_planning_time = timeout
        options.max_nodes = max_iterations
        options.collision_check_use_bisection = True
        planner = roboplan_rrt.RRT(self._world._require_scene(), options)
        start = roboplan_core.JointConfiguration(list(group.native_names), q_start)
        goal = roboplan_core.JointConfiguration(list(group.native_names), q_goal)
        result = planner.plan(start, goal)
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        return result

    def _shortcut_native_path(
        self,
        group: RoboPlanGroup,
        path: roboplan_core.JointPath,
    ) -> roboplan_core.JointPath:
        config = self._config.path_shortcutting
        if not config.enabled:
            return path

        options = roboplan_core.PathShortcuttingOptions()
        options.group_name = group.name
        options.max_step_size = config.max_step_size
        options.max_iters = config.max_iters
        options.seed = config.seed
        options.max_convergence_iters = config.max_convergence_iters
        options.redundant_removal_iters = config.redundant_removal_iters
        shortcutter = roboplan_core.PathShortcutter(self._world._require_scene(), options)
        try:
            shortened = shortcutter.shortcut(path)
        except RuntimeError as exc:
            logger.warning(
                "RoboPlan path shortcutting failed; using raw path",
                error=str(exc),
            )
            return path

        try:
            self._validate_shortcut_path(group, path, shortened)
        except ValueError as exc:
            logger.warning(
                "RoboPlan path shortcutting returned an invalid path; using raw path",
                error=str(exc),
            )
            return path
        return shortened

    def _validate_shortcut_path(
        self,
        group: RoboPlanGroup,
        original: roboplan_core.JointPath,
        shortened: roboplan_core.JointPath,
    ) -> None:
        """Validate structural guarantees before accepting a shortened path."""
        original_path = self._path_from_native(group, original)
        shortened_path = self._path_from_native(group, shortened)
        if not shortened_path:
            raise ValueError("RoboPlan path shortcutter returned an empty path")
        if not np.allclose(
            shortened_path[0].position,
            original_path[0].position,
            atol=1e-9,
            rtol=0.0,
        ):
            raise ValueError("RoboPlan path shortcutter changed the start configuration")
        if not np.allclose(
            shortened_path[-1].position,
            original_path[-1].position,
            atol=1e-9,
            rtol=0.0,
        ):
            raise ValueError("RoboPlan path shortcutter changed the goal configuration")

    def _path_from_native(
        self,
        group: RoboPlanGroup,
        result: Any,
        *,
        output_names: Sequence[str] | None = None,
    ) -> list[JointState]:
        result_names = tuple(getattr(result, "joint_names", ()) or group.native_names)
        if set(result_names) != set(group.native_names):
            raise ValueError("RoboPlan path joint names do not match the selected group")
        public_by_native = dict(zip(group.native_names, group.public_names, strict=True))
        source_names = tuple(public_by_native[name] for name in result_names)
        ordered_output_names = (
            tuple(output_names) if output_names is not None else group.output_names
        )
        if len(ordered_output_names) != len(set(ordered_output_names)) or set(
            ordered_output_names
        ) != set(group.public_names):
            raise ValueError("RoboPlan output joint names do not match the selected group")
        path: list[JointState] = []
        for waypoint in result.positions:
            values = np.asarray(waypoint, dtype=np.float64)
            if len(values) != len(source_names):
                raise ValueError("RoboPlan path waypoint length does not match its names")
            positions = dict(zip(source_names, values, strict=True))
            path.append(
                JointState(
                    name=list(ordered_output_names),
                    position=[float(positions[name]) for name in ordered_output_names],
                )
            )
        return path
