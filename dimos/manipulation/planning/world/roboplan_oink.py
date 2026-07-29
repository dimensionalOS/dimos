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

"""Private request solver for RoboPlan-native OInK."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

try:
    import roboplan.core as roboplan_core
    import roboplan.optimal_ik as roboplan_optimal_ik
except ImportError as exc:
    raise ImportError(
        "RoboPlan native IK requires the roboplan package with optimal_ik. "
        "Install the manipulation extra before selecting the roboplan backend."
    ) from exc

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, RobotName
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.manipulation.planning.world.roboplan_model import RoboPlanGroup, RoboPlanModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import pose_to_matrix

_MAX_ITERATIONS_PER_ATTEMPT = 100


@dataclass(frozen=True)
class RoboPlanIKRobotState:
    """One robot's immutable state at the start of an IK request."""

    joint_names: tuple[str, ...]
    positions: NDArray[np.float64]
    lower_limits: NDArray[np.float64]
    upper_limits: NDArray[np.float64]


@dataclass(frozen=True)
class RoboPlanIKContext:
    """Backend state captured while RoboPlanWorld holds its scene lock."""

    scene: Any
    model: RoboPlanModel
    planning_groups: PlanningGroupRegistry
    robots: Mapping[RobotName, RoboPlanIKRobotState]
    scene_q: NDArray[np.float64]


def solve_roboplan_ik(
    context: RoboPlanIKContext,
    pose_targets: Mapping[PlanningGroup, PoseStamped],
    auxiliary_groups: Sequence[PlanningGroup] = (),
    seed: JointState | None = None,
    position_tolerance: float = 0.001,
    orientation_tolerance: float = 0.01,
    check_collision: bool = True,
    max_attempts: int = 10,
) -> IKResult:
    """Solve one locked RoboPlan request without restoring the shared scene."""
    if not pose_targets and not auxiliary_groups:
        return _failure(
            IKStatus.NO_SOLUTION,
            "At least one pose target or auxiliary group is required",
        )
    if max_attempts <= 0:
        return _failure(IKStatus.NO_SOLUTION, "max_attempts must be positive")
    if position_tolerance <= 0.0 or orientation_tolerance <= 0.0:
        return _failure(
            IKStatus.NO_SOLUTION,
            "IK position and orientation tolerances must be positive",
        )

    groups = tuple(pose_targets) + tuple(auxiliary_groups)
    try:
        for group in groups:
            if context.planning_groups.get(group.id) != group:
                raise ValueError(f"Planning group '{group.id}' does not match the RoboPlan world")
        for group, target in pose_targets.items():
            if not group.has_pose_target or group.tip_link is None:
                raise ValueError(f"Planning group '{group.id}' has no pose target frame")
            if target.frame_id != "world":
                return _failure(
                    IKStatus.UNSUPPORTED,
                    f"RoboPlan-native IK only supports frame_id='world', got '{target.frame_id}'",
                )
        selection = PlanningGroupSelection.from_groups(groups)
        native_group = context.model.groups.get(frozenset(selection.group_ids))
        if native_group is None:
            return _failure(
                IKStatus.UNSUPPORTED,
                "RoboPlan has no generated group for this planning-group selection",
            )
    except (KeyError, ValueError) as exc:
        return _failure(IKStatus.UNSUPPORTED, str(exc))

    try:
        seed_q = _seed_q(context, selection, native_group, seed)
        lower, upper = _selection_limits(context, selection, native_group)
        if not pose_targets:
            return _auxiliary_only_result(
                context,
                selection,
                native_group,
                seed_q,
                check_collision,
            )

        oink = roboplan_optimal_ik.Oink(context.scene, group_name=native_group.name)
        q_indices = np.asarray(oink.q_indices, dtype=np.int64)
        v_indices = np.asarray(oink.v_indices, dtype=np.int64)
        if len(q_indices) != len(native_group.native_names):
            raise ValueError(
                f"OInK exposes {len(q_indices)} position indices for "
                f"{len(native_group.native_names)} selected joints"
            )
        if len(v_indices) != int(oink.num_variables):
            raise ValueError(
                f"OInK exposes {len(v_indices)} velocity indices for "
                f"{oink.num_variables} optimization variables"
            )

        tasks = _make_tasks(context, oink, pose_targets)
        constraints = [roboplan_optimal_ik.PositionLimit(oink)]
        randomized_indices = _pose_target_indices(native_group, tuple(pose_targets))

        total_iterations = 0
        best_score = float("inf")
        best_errors = (float("inf"), float("inf"))
        collision_errors: tuple[float, float] | None = None
        for attempt in range(max_attempts):
            candidate_q = seed_q.copy()
            if attempt:
                candidate_q[list(randomized_indices)] = np.random.uniform(
                    lower[list(randomized_indices)],
                    upper[list(randomized_indices)],
                )

            for step in range(_MAX_ITERATIONS_PER_ATTEMPT + 1):
                full_q = context.scene_q.copy()
                full_q[q_indices] = candidate_q
                context.scene.setJointPositions(full_q)
                position_error, orientation_error = _target_errors(context, full_q, pose_targets)
                score = max(
                    position_error / position_tolerance,
                    orientation_error / orientation_tolerance,
                )
                if score < best_score:
                    best_score = score
                    best_errors = (position_error, orientation_error)

                if (
                    position_error <= position_tolerance
                    and orientation_error <= orientation_tolerance
                ):
                    if check_collision and bool(context.scene.hasCollisions(full_q)):
                        collision_errors = (position_error, orientation_error)
                        break
                    return IKResult(
                        status=IKStatus.SUCCESS,
                        joint_state=_joint_state(selection, native_group, candidate_q),
                        position_error=position_error,
                        orientation_error=orientation_error,
                        iterations=total_iterations,
                        message="RoboPlan OInK solution found",
                    )

                if step == _MAX_ITERATIONS_PER_ATTEMPT:
                    break
                delta_q = np.zeros(int(oink.num_variables), dtype=np.float64)
                oink.solveIk(context.scene, tasks, constraints, delta_q)
                full_delta_q = np.zeros_like(context.scene_q)
                full_delta_q[v_indices] = delta_q
                candidate_full_q = np.asarray(
                    context.scene.integrate(full_q, full_delta_q),
                    dtype=np.float64,
                )
                candidate_q = np.clip(candidate_full_q[q_indices], lower, upper)
                total_iterations += 1

        if collision_errors is not None:
            return _failure(
                IKStatus.COLLISION,
                "RoboPlan OInK converged only to colliding endpoints",
                position_error=collision_errors[0],
                orientation_error=collision_errors[1],
                iterations=total_iterations,
            )
        return _failure(
            IKStatus.NO_SOLUTION,
            f"RoboPlan OInK did not converge after {max_attempts} attempts",
            position_error=best_errors[0],
            orientation_error=best_errors[1],
            iterations=total_iterations,
        )
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        return _failure(
            IKStatus.NO_SOLUTION,
            f"RoboPlan OInK setup or mapping failed: {exc}",
        )
    except Exception as exc:
        return _failure(
            IKStatus.NO_SOLUTION,
            f"RoboPlan OInK solve failed: {exc}",
        )


def _seed_q(
    context: RoboPlanIKContext,
    selection: PlanningGroupSelection,
    native_group: RoboPlanGroup,
    seed: JointState | None,
) -> NDArray[np.float64]:
    positions = {
        f"{robot_name}/{joint_name}": float(position)
        for robot_name, robot in context.robots.items()
        for joint_name, position in zip(robot.joint_names, robot.positions, strict=True)
    }
    if seed is not None:
        if not seed.name:
            if len(seed.position) != len(selection.joint_names):
                raise ValueError(
                    f"Seed has {len(seed.position)} positions for "
                    f"{len(selection.joint_names)} selected joints"
                )
            positions.update(
                {
                    name: float(value)
                    for name, value in zip(selection.joint_names, seed.position, strict=True)
                }
            )
        else:
            if len(seed.name) != len(seed.position):
                raise ValueError(
                    f"Seed has {len(seed.name)} names but {len(seed.position)} positions"
                )
            aliases: dict[str, set[str]] = {}
            for group in selection.groups:
                for local_name, global_name in zip(
                    group.local_joint_names, group.joint_names, strict=True
                ):
                    aliases.setdefault(local_name, set()).add(global_name)
                    aliases.setdefault(global_name, set()).add(global_name)
            seen: set[str] = set()
            for seed_name, value in zip(seed.name, seed.position, strict=True):
                matches = aliases.get(seed_name, set())
                if not matches:
                    raise ValueError(f"Seed contains joint outside RoboPlan selection: {seed_name}")
                if len(matches) > 1:
                    raise ValueError(f"Seed joint name is ambiguous: {seed_name}")
                global_name = next(iter(matches))
                if global_name in seen:
                    raise ValueError(f"Seed contains duplicate selected joint: {global_name}")
                seen.add(global_name)
                positions[global_name] = float(value)

    public_by_native = dict(zip(native_group.native_names, native_group.public_names, strict=True))
    return np.asarray(
        [positions[public_by_native[name]] for name in native_group.native_names],
        dtype=np.float64,
    )


def _selection_limits(
    context: RoboPlanIKContext,
    selection: PlanningGroupSelection,
    native_group: RoboPlanGroup,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    lower_by_global: dict[str, float] = {}
    upper_by_global: dict[str, float] = {}
    for group in selection.groups:
        robot = context.robots[group.robot_name]
        indices = {name: index for index, name in enumerate(robot.joint_names)}
        for local_name, global_name in zip(group.local_joint_names, group.joint_names, strict=True):
            index = indices[local_name]
            lower_by_global[global_name] = float(robot.lower_limits[index])
            upper_by_global[global_name] = float(robot.upper_limits[index])

    public_by_native = dict(zip(native_group.native_names, native_group.public_names, strict=True))
    return (
        np.asarray(
            [lower_by_global[public_by_native[name]] for name in native_group.native_names],
            dtype=np.float64,
        ),
        np.asarray(
            [upper_by_global[public_by_native[name]] for name in native_group.native_names],
            dtype=np.float64,
        ),
    )


def _pose_target_indices(
    native_group: RoboPlanGroup,
    pose_groups: Sequence[PlanningGroup],
) -> tuple[int, ...]:
    pose_joint_names = {name for group in pose_groups for name in group.joint_names}
    return tuple(
        index
        for index, public_name in enumerate(native_group.public_names)
        if public_name in pose_joint_names
    )


def _make_tasks(
    context: RoboPlanIKContext,
    oink: Any,
    pose_targets: Mapping[PlanningGroup, PoseStamped],
) -> list[Any]:
    tasks: list[Any] = []
    for group, target_pose in pose_targets.items():
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group.id}' has no pose target frame")
        target = roboplan_core.CartesianConfiguration()
        target.base_frame = ""
        target.tip_frame = context.model.native_link(group.robot_name, group.tip_link)
        target.tform = pose_to_matrix(target_pose)
        tasks.append(roboplan_optimal_ik.FrameTask(oink, context.scene, target))
    return tasks


def _target_errors(
    context: RoboPlanIKContext,
    full_q: NDArray[np.float64],
    pose_targets: Mapping[PlanningGroup, PoseStamped],
) -> tuple[float, float]:
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    for group, target_pose in pose_targets.items():
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group.id}' has no pose target frame")
        current = np.asarray(
            context.scene.forwardKinematics(
                full_q,
                context.model.native_link(group.robot_name, group.tip_link),
                "",
            ),
            dtype=np.float64,
        )
        position_error, orientation_error = compute_pose_error(current, pose_to_matrix(target_pose))
        position_errors.append(position_error)
        orientation_errors.append(orientation_error)
    return max(position_errors), max(orientation_errors)


def _joint_state(
    selection: PlanningGroupSelection,
    native_group: RoboPlanGroup,
    native_q: NDArray[np.float64],
) -> JointState:
    if len(native_q) != len(native_group.native_names):
        raise ValueError(
            f"RoboPlan OInK returned {len(native_q)} selected positions for "
            f"{len(native_group.native_names)} selected joints"
        )
    positions = dict(zip(native_group.public_names, native_q.astype(float), strict=True))
    return JointState(
        name=list(selection.joint_names),
        position=[float(positions[name]) for name in selection.joint_names],
    )


def _auxiliary_only_result(
    context: RoboPlanIKContext,
    selection: PlanningGroupSelection,
    native_group: RoboPlanGroup,
    native_q: NDArray[np.float64],
    check_collision: bool,
) -> IKResult:
    index_by_native = {name: index for index, name in enumerate(context.scene.getJointNames())}
    full_q = context.scene_q.copy()
    for native_name, value in zip(native_group.native_names, native_q, strict=True):
        full_q[index_by_native[native_name]] = float(value)
    context.scene.setJointPositions(full_q)
    if check_collision and bool(context.scene.hasCollisions(full_q)):
        return _failure(
            IKStatus.COLLISION,
            "RoboPlan OInK auxiliary selection is in collision",
        )
    return IKResult(
        status=IKStatus.SUCCESS,
        joint_state=_joint_state(selection, native_group, native_q),
        iterations=0,
        message="RoboPlan OInK auxiliary selection accepted",
    )


def _failure(
    status: IKStatus,
    message: str,
    *,
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
