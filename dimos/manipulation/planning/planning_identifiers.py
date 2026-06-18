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

"""Planning group and resolved-joint identifier helpers."""

from __future__ import annotations

from dimos.manipulation.planning.spec.models import (
    LocalModelJointName,
    PlanningGroupID,
    ResolvedJointName,
    RobotName,
)


def make_planning_group_id(robot_name: RobotName, group_name: str) -> PlanningGroupID:
    """Build a public planning group ID."""
    if not robot_name or "/" in robot_name:
        raise ValueError(f"Invalid robot name for planning group ID: {robot_name!r}")
    if not group_name or "/" in group_name:
        raise ValueError(f"Invalid planning group name: {group_name!r}")
    return f"{robot_name}/{group_name}"


def parse_planning_group_id(group_id: PlanningGroupID) -> tuple[RobotName, str]:
    """Split and validate a planning group ID."""
    parts = group_id.split("/", maxsplit=1)
    if len(parts) != 2 or not parts[0] or not parts[1] or "/" in parts[1]:
        raise ValueError(
            f"Invalid planning group ID {group_id!r}; expected '{{robot_name}}/{{group_name}}'"
        )
    return parts[0], parts[1]


def make_resolved_joint_name(
    robot_name: RobotName,
    local_joint_name: LocalModelJointName,
) -> ResolvedJointName:
    """Convert a local model joint name to a public resolved joint name."""
    if not robot_name or "/" in robot_name:
        raise ValueError(f"Invalid robot name for resolved joint name: {robot_name!r}")
    if not local_joint_name or "/" in local_joint_name:
        raise ValueError(f"Invalid local joint name: {local_joint_name!r}")
    return f"{robot_name}/{local_joint_name}"


def make_resolved_joint_names(
    robot_name: RobotName,
    local_joint_names: list[LocalModelJointName] | tuple[LocalModelJointName, ...],
) -> list[ResolvedJointName]:
    """Convert local model joint names to public resolved joint names."""
    return [make_resolved_joint_name(robot_name, name) for name in local_joint_names]


def local_joint_name_from_resolved(
    robot_name: RobotName,
    resolved_joint_name: ResolvedJointName,
) -> LocalModelJointName:
    """Validate and strip a resolved joint name for backend internals."""
    prefix = f"{robot_name}/"
    if not resolved_joint_name.startswith(prefix):
        raise ValueError(
            f"Resolved joint name {resolved_joint_name!r} does not belong to robot {robot_name!r}"
        )
    local_name = resolved_joint_name[len(prefix) :]
    if not local_name or "/" in local_name:
        raise ValueError(f"Invalid resolved joint name: {resolved_joint_name!r}")
    return local_name


__all__ = [
    "local_joint_name_from_resolved",
    "make_planning_group_id",
    "make_resolved_joint_name",
    "make_resolved_joint_names",
    "parse_planning_group_id",
]
