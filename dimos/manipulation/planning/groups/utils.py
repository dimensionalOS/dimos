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

"""Shared helpers for planning-group selectors and joint-state projection."""

from collections.abc import Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.groups.identifiers import (
    assert_global_joint_names,
    assert_local_joint_names,
    is_global_joint_name,
    make_global_joint_names,
)
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import (
    GlobalJointName,
    JointPath,
    LocalModelJointName,
    PlanningGroupID,
    RobotName,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def planning_group_id_from_selector(selector: PlanningGroupID | PlanningGroup) -> PlanningGroupID:
    """Return the planning-group ID represented by a selector."""
    if isinstance(selector, PlanningGroup):
        return selector.id
    return selector


def matching_global_joint_name(
    positions_by_name: Mapping[str, float], local_joint_name: LocalModelJointName
) -> GlobalJointName | None:
    """Find the unique global joint name ending with a local joint name."""
    suffix = f"/{local_joint_name}"
    matches = [name for name in positions_by_name if name.endswith(suffix)]
    if len(matches) == 1:
        return matches[0]
    return None


def filter_joint_state_to_selected_joints(
    joint_state: JointState,
    global_joint_names: Sequence[GlobalJointName],
    local_joint_names: Sequence[LocalModelJointName] = (),
) -> JointState:
    """Project a joint state to selected global joints.

    Values are looked up by global name first. When ``local_joint_names`` is
    provided, each corresponding local name is used as a fallback.
    """
    if local_joint_names and len(global_joint_names) != len(local_joint_names):
        raise ValueError("Global and local selected joint lists must have the same length")

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    selected_positions: list[float] = []
    missing: list[str] = []
    for index, global_name in enumerate(global_joint_names):
        if global_name in positions_by_name:
            selected_positions.append(float(positions_by_name[global_name]))
            continue
        if local_joint_names:
            local_name = local_joint_names[index]
            if local_name in positions_by_name:
                selected_positions.append(float(positions_by_name[local_name]))
                continue
        missing.append(global_name)

    if missing:
        raise ValueError(f"IK result is missing selected joints: {missing}")

    return JointState({"name": list(global_joint_names), "position": selected_positions})


def joint_target_to_global_names(
    group: PlanningGroup,
    target: JointState,
) -> JointState:
    """Convert a group joint target to global joint names in group order.

    Named targets may use either the public global planning names or the
    robot-local model names used by legacy robot-scoped callers, but the two
    namespaces must not be mixed in one target.
    """
    if not target.name:
        if len(target.position) != len(group.joint_names):
            raise ValueError(
                f"Target for '{group.id}' has {len(target.position)} positions, "
                f"expected {len(group.joint_names)}"
            )
        return JointState(name=list(group.joint_names), position=list(target.position))

    if len(target.name) != len(target.position):
        raise ValueError(
            f"Target for '{group.id}' has {len(target.name)} names but "
            f"{len(target.position)} positions"
        )

    target_names = list(target.name)
    global_flags = [is_global_joint_name(name) for name in target_names]
    if any(global_flags) and not all(global_flags):
        raise ValueError(
            f"Target for '{group.id}' mixes global and local joint names: {target_names}"
        )

    if all(global_flags):
        assert_global_joint_names(target_names)
        expected_names = group.joint_names
    else:
        assert_local_joint_names(target_names)
        expected_names = group.local_joint_names

    positions_by_name = dict(zip(target_names, target.position, strict=True))
    global_positions: list[float] = []
    missing: list[str] = []
    for expected_name in expected_names:
        if expected_name in positions_by_name:
            global_positions.append(positions_by_name[expected_name])
        else:
            missing.append(expected_name)
    if missing:
        raise ValueError(f"Target for '{group.id}' is missing joints: {missing}")

    extra = set(target_names) - set(expected_names)
    if extra:
        raise ValueError(f"Target for '{group.id}' has extra joints: {sorted(extra)}")
    return JointState(name=list(group.joint_names), position=global_positions)


def project_global_joint_path_to_robot(
    path: Sequence[JointState],
    *,
    robot_name: RobotName,
    local_joint_names: Sequence[LocalModelJointName],
    current_joint_state: JointState | None,
) -> JointPath:
    """Project a selected-global-joint path into one robot's local joint path."""
    if not path:
        return []

    selected_joint_names = tuple(path[0].name)
    assert_global_joint_names(selected_joint_names)
    if any(
        len(waypoint.name) != len(waypoint.position) or tuple(waypoint.name) != selected_joint_names
        for waypoint in path
    ):
        raise ValueError("inconsistent waypoint joint names")

    selected_joint_indices = dict(
        zip(selected_joint_names, range(len(selected_joint_names)), strict=True)
    )
    selected_joint_set = set(selected_joint_names)
    waypoint_positions = [[float(position) for position in waypoint.position] for waypoint in path]
    current_by_name = (
        dict(zip(current_joint_state.name, current_joint_state.position, strict=False))
        if current_joint_state is not None
        else {}
    )
    global_joint_names = make_global_joint_names(robot_name, tuple(local_joint_names))
    joint_pairs = list(zip(local_joint_names, global_joint_names, strict=True))
    try:
        base_positions = [
            0.0 if global_name in selected_joint_set else float(current_by_name[local_name])
            for local_name, global_name in joint_pairs
        ]
    except KeyError as exc:
        raise ValueError(f"missing joint '{exc.args[0]}'") from exc

    overlay_indices = [
        (local_index, selected_joint_indices[global_name])
        for local_index, (_, global_name) in enumerate(joint_pairs)
        if global_name in selected_joint_indices
    ]
    local_path: JointPath = []
    for waypoint_positions_by_joint in waypoint_positions:
        projected_positions = base_positions.copy()
        for local_index, selected_index in overlay_indices:
            projected_positions[local_index] = waypoint_positions_by_joint[selected_index]
        local_path.append(JointState(name=list(local_joint_names), position=projected_positions))
    return local_path


def joint_state_to_ordered_positions(
    joint_state: JointState,
    *,
    joint_names: Sequence[str],
    joint_name_mapping: Mapping[str, str],
) -> NDArray[np.float64]:
    """Convert a JointState to an array ordered by local robot joint names."""
    if not joint_state.name:
        if len(joint_state.position) != len(joint_names):
            raise ValueError("JointState position length must match configured joint count")
        return np.asarray(joint_state.position, dtype=np.float64)

    if len(joint_state.name) != len(joint_state.position):
        raise ValueError("JointState name and position lengths must match")

    joint_name_set = set(joint_names)
    name_to_pos: dict[str, float] = {}
    for name, position in zip(joint_state.name, joint_state.position, strict=True):
        if name in joint_name_set:
            resolved_name = name
        elif name in joint_name_mapping:
            resolved_name = joint_name_mapping[name]
        elif is_global_joint_name(name):
            resolved_name = name.split("/", maxsplit=1)[1]
            if resolved_name not in joint_name_set:
                raise ValueError(f"Unknown global joint name: {name}")
        else:
            raise ValueError(
                f"Unrecognized joint name '{name}': not a known local name, not in joint_name_mapping, and not a global name"
            )

        if resolved_name in name_to_pos:
            raise ValueError(f"JointState resolves duplicate joint '{resolved_name}'")
        name_to_pos[resolved_name] = float(position)

    missing = [name for name in joint_names if name not in name_to_pos]
    if missing:
        raise ValueError(f"JointState missing joints: {missing}")
    return np.asarray([name_to_pos[name] for name in joint_names], dtype=np.float64)
