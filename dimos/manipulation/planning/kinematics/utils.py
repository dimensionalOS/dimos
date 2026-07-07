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

"""Shared IK-only helpers for planning-group-scoped kinematics backends."""

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.utils import filter_joint_state_to_selected_joints
from dimos.manipulation.planning.spec.models import IKResult, RobotName, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.sensor_msgs.JointState import JointState


def unique_pose_target_frame_for_robot(world: WorldSpec, robot_id: WorldRobotID) -> str | None:
    config = world.get_robot_config(robot_id)
    pose_target_frames = [
        group.tip_link for group in config.planning_groups if group.tip_link is not None
    ]
    unique_frames = list(dict.fromkeys(pose_target_frames))
    if len(unique_frames) != 1:
        return None
    return unique_frames[0]


def robot_ids_by_name(
    world: WorldSpec,
    robot_names: tuple[RobotName, ...],
) -> dict[RobotName, WorldRobotID]:
    robot_ids_by_name: dict[RobotName, WorldRobotID] = {}
    for robot_name in robot_names:
        matches = [
            robot_id
            for robot_id in world.get_robot_ids()
            if world.get_robot_config(robot_id).name == robot_name
        ]
        if not matches:
            raise ValueError(f"Robot '{robot_name}' not found")
        if len(matches) > 1:
            raise ValueError(f"Robot name '{robot_name}' is not unique in planning world")
        robot_ids_by_name[robot_name] = matches[0]
    return robot_ids_by_name


def seed_positions_with_world_fallback(
    world: WorldSpec,
    robot_id: WorldRobotID,
    robot_name: RobotName,
    local_joint_names: list[str],
    seed: JointState | None,
) -> NDArray[np.float64]:
    """Return full robot positions, reading world only for absent seed joints."""
    if seed is None:
        with world.scratch_context() as ctx:
            current = world.get_joint_state(ctx, robot_id)
        return positions_by_local_name(current, robot_name, local_joint_names)

    try:
        return positions_by_local_name(seed, robot_name, local_joint_names)
    except ValueError:
        with world.scratch_context() as ctx:
            current = world.get_joint_state(ctx, robot_id)
        fallback_positions = positions_by_local_name(current, robot_name, local_joint_names)
        seed_positions = partial_positions_by_local_name(seed, robot_name, local_joint_names)
        local_indices = {name: index for index, name in enumerate(local_joint_names)}
        for local_name, position in seed_positions.items():
            fallback_positions[local_indices[local_name]] = position
        return fallback_positions


def positions_by_local_name(
    joint_state: JointState,
    robot_name: RobotName,
    local_joint_names: list[str],
) -> NDArray[np.float64]:
    if not joint_state.name:
        if len(joint_state.position) != len(local_joint_names):
            raise ValueError(
                f"JointState has {len(joint_state.position)} positions for "
                f"{len(local_joint_names)} joints"
            )
        return np.asarray(joint_state.position, dtype=np.float64)

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    positions: list[float] = []
    missing: list[str] = []
    for local_name in local_joint_names:
        global_name = f"{robot_name}/{local_name}"
        if local_name in positions_by_name:
            positions.append(float(positions_by_name[local_name]))
        elif global_name in positions_by_name:
            positions.append(float(positions_by_name[global_name]))
        else:
            missing.append(local_name)
    if missing:
        raise ValueError(f"JointState missing joints: {missing}")
    return np.asarray(positions, dtype=np.float64)


def partial_positions_by_local_name(
    joint_state: JointState,
    robot_name: RobotName,
    local_joint_names: list[str],
) -> dict[str, float]:
    if len(joint_state.name) != len(joint_state.position):
        raise ValueError(
            f"Seed has {len(joint_state.name)} names but {len(joint_state.position)} positions"
        )
    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    known_local_names = set(local_joint_names)
    positions: dict[str, float] = {}
    for name, position in positions_by_name.items():
        if name in known_local_names:
            positions[name] = float(position)
            continue
        prefix = f"{robot_name}/"
        if name.startswith(prefix):
            local_name = name[len(prefix) :]
            if local_name in known_local_names:
                positions[local_name] = float(position)
                continue
        raise ValueError(f"Unrecognized seed joint '{name}'")
    return positions


def filter_result_to_group(result: IKResult, group: PlanningGroup) -> IKResult:
    return filter_result_to_selection(result, PlanningGroupSelection.from_groups((group,)))


def filter_result_to_selection(result: IKResult, selection: PlanningGroupSelection) -> IKResult:
    if result.joint_state is None:
        return result
    local_joint_names = tuple(
        local_name for group in selection.groups for local_name in group.local_joint_names
    )
    return IKResult(
        status=result.status,
        joint_state=filter_joint_state_to_selected_joints(
            result.joint_state,
            selection.joint_names,
            local_joint_names,
        ),
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        iterations=result.iterations,
        message=result.message,
    )


def groups_by_robot(groups: Sequence[PlanningGroup]) -> dict[RobotName, list[PlanningGroup]]:
    grouped: dict[RobotName, list[PlanningGroup]] = {}
    for group in groups:
        grouped.setdefault(group.robot_name, []).append(group)
    return grouped
