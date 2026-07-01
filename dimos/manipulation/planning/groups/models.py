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

"""Backend-independent planning-group domain models."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, TypeAlias

from dimos.manipulation.planning.spec.models import (
    GlobalJointName,
    LocalModelJointName,
    PlanningGroupID,
    RobotName,
)

PlanningGroupSource: TypeAlias = Literal["srdf", "fallback"]


@dataclass(frozen=True)
class PlanningGroupDefinition:
    """Model-level declaration of a planning group.

    Joint names are local model names. The definition is safe to store on
    ``RobotModelConfig`` and is not bound to any runtime world robot ID.
    """

    name: str
    joint_names: tuple[LocalModelJointName, ...]
    base_link: str
    tip_link: str | None = None
    source: PlanningGroupSource = "srdf"

    @property
    def has_pose_target(self) -> bool:
        """Whether this group has a valid pose target frame."""
        return self.tip_link is not None


@dataclass(frozen=True)
class PlanningGroup:
    """Public backend-independent planning group.

    A planning group exposes stable public IDs and global joint names for
    planning APIs. It intentionally does not include backend runtime robot IDs.
    """

    id: PlanningGroupID
    robot_name: RobotName
    group_name: str
    joint_names: tuple[GlobalJointName, ...]
    local_joint_names: tuple[LocalModelJointName, ...]
    base_link: str
    tip_link: str | None = None
    source: PlanningGroupSource = "srdf"

    @property
    def has_pose_target(self) -> bool:
        """Whether this group can be directly pose-targeted."""
        return self.tip_link is not None


@dataclass(frozen=True)
class PlanningGroupSelection:
    """Validated ordered selection of planning groups.

    Selection validates ID existence and selected-joint overlap outside any
    world backend. Requested group order is preserved.
    """

    groups: tuple[PlanningGroup, ...]
    group_ids: tuple[PlanningGroupID, ...]
    joint_names: tuple[GlobalJointName, ...]
    robot_names: tuple[RobotName, ...]

    @classmethod
    def from_groups(cls, groups: tuple[PlanningGroup, ...]) -> PlanningGroupSelection:
        """Build a selection, rejecting overlapping selected global joints."""
        seen_joints: dict[GlobalJointName, PlanningGroupID] = {}
        joint_names: list[GlobalJointName] = []
        robot_names: list[RobotName] = []
        for group in groups:
            if group.robot_name not in robot_names:
                robot_names.append(group.robot_name)
            for joint_name in group.joint_names:
                previous_group_id = seen_joints.get(joint_name)
                if previous_group_id is not None:
                    raise ValueError(
                        "Selected planning groups overlap on global joint "
                        f"{joint_name}: {previous_group_id} and {group.id}"
                    )
                seen_joints[joint_name] = group.id
                joint_names.append(joint_name)

        return cls(
            groups=groups,
            group_ids=tuple(group.id for group in groups),
            joint_names=tuple(joint_names),
            robot_names=tuple(robot_names),
        )
