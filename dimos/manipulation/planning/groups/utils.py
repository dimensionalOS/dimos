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

from dimos.manipulation.planning.groups.identifiers import (
    assert_global_joint_names,
    assert_local_joint_names,
    is_global_joint_name,
)
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import (
    GlobalJointName,
    LocalModelJointName,
    PlanningGroupID,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
    logger.warning(
        f"Expected exactly one global joint ending with local joint name "
        f"'{local_joint_name}', found {len(matches)} matches: {matches}"
    )
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
        raise ValueError(f"Joint state is missing selected joints: {missing}")

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
