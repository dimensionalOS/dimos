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

"""Shared helpers for planning-group selector and joint-state projection."""

from collections.abc import Mapping, Sequence

from dimos.manipulation.planning.spec.models import (
    LocalModelJointName,
    PlanningGroupDescriptor,
    PlanningGroupID,
    ResolvedJointName,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def planning_group_id_from_selector(
    selector: PlanningGroupID | PlanningGroupDescriptor,
) -> PlanningGroupID:
    """Return the planning-group ID represented by a selector."""
    if isinstance(selector, PlanningGroupDescriptor):
        return selector.id
    return selector


def matching_resolved_joint_name(
    positions_by_name: Mapping[str, float], local_joint_name: LocalModelJointName
) -> ResolvedJointName | None:
    """Find the unique resolved joint name ending with a local joint name."""
    suffix = f"/{local_joint_name}"
    matches = [name for name in positions_by_name if name.endswith(suffix)]
    if len(matches) == 1:
        return matches[0]
    return None


def filter_joint_state_to_selected_joints(
    joint_state: JointState,
    resolved_joint_names: Sequence[ResolvedJointName],
    local_joint_names: Sequence[LocalModelJointName] = (),
) -> JointState:
    """Project a joint state to selected resolved joints.

    Values are looked up by resolved name first. When ``local_joint_names`` is
    provided, each corresponding local name is used as a fallback.
    """
    if local_joint_names and len(resolved_joint_names) != len(local_joint_names):
        raise ValueError("Resolved and local selected joint lists must have the same length")

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    selected_positions: list[float] = []
    missing: list[str] = []
    for index, resolved_name in enumerate(resolved_joint_names):
        if resolved_name in positions_by_name:
            selected_positions.append(float(positions_by_name[resolved_name]))
            continue
        if local_joint_names:
            local_name = local_joint_names[index]
            if local_name in positions_by_name:
                selected_positions.append(float(positions_by_name[local_name]))
                continue
        missing.append(resolved_name)

    if missing:
        raise ValueError(f"IK result is missing selected joints: {missing}")

    return JointState({"name": list(resolved_joint_names), "position": selected_positions})
