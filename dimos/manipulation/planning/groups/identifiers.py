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

"""Planning-group and global-joint identifier helpers."""

from __future__ import annotations

from collections.abc import Sequence

from dimos.manipulation.planning.spec.models import (
    GlobalJointName,
    LocalModelJointName,
    PlanningGroupID,
    RobotName,
)


def assert_valid_robot_name(robot_name: RobotName) -> None:
    """Validate a robot name for delimiter-based public IDs."""
    if not robot_name or "/" in robot_name:
        raise ValueError(f"Invalid robot name: {robot_name!r}")


def assert_valid_local_joint_name(local_joint_name: LocalModelJointName) -> None:
    """Validate a local model joint name for delimiter-based global joint names."""
    if not local_joint_name or "/" in local_joint_name:
        raise ValueError(f"Invalid local joint name: {local_joint_name!r}")


def assert_local_joint_names(names: Sequence[LocalModelJointName]) -> None:
    """Validate that names are local model joint names, not global joint names."""
    for name in names:
        assert_valid_local_joint_name(name)


def make_planning_group_id(robot_name: RobotName, group_name: str) -> PlanningGroupID:
    """Build a public planning group ID."""
    assert_valid_robot_name(robot_name)
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


def make_global_joint_name(
    robot_name: RobotName,
    local_joint_name: LocalModelJointName,
) -> GlobalJointName:
    """Convert a local model joint name to a public global joint name."""
    assert_valid_robot_name(robot_name)
    assert_valid_local_joint_name(local_joint_name)
    return f"{robot_name}/{local_joint_name}"


def make_global_joint_names(
    robot_name: RobotName,
    local_joint_names: list[LocalModelJointName] | tuple[LocalModelJointName, ...],
) -> list[GlobalJointName]:
    """Convert local model joint names to public global joint names."""
    return [make_global_joint_name(robot_name, name) for name in local_joint_names]


def is_global_joint_name(name: str) -> bool:
    """Return whether name has the exact global joint-name shape."""
    parts = name.split("/")
    return len(parts) == 2 and bool(parts[0]) and bool(parts[1])


def assert_global_joint_names(names: Sequence[GlobalJointName]) -> None:
    """Validate that names are global joint names."""
    invalid = [name for name in names if not is_global_joint_name(name)]
    if invalid:
        raise ValueError(f"Expected global joint names; got invalid names: {invalid}")


def local_joint_name_from_global(
    robot_name: RobotName,
    global_joint_name: GlobalJointName,
) -> LocalModelJointName:
    """Validate and strip a global joint name for backend internals."""
    assert_valid_robot_name(robot_name)
    prefix = f"{robot_name}/"
    if not global_joint_name.startswith(prefix):
        raise ValueError(
            f"Global joint name {global_joint_name!r} does not belong to robot {robot_name!r}"
        )
    local_name = global_joint_name[len(prefix) :]
    try:
        assert_valid_local_joint_name(local_name)
    except ValueError as exc:
        raise ValueError(f"Invalid global joint name: {global_joint_name!r}") from exc
    return local_name
