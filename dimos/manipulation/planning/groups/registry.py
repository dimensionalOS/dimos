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

"""Backend-independent planning-group registry."""

from __future__ import annotations

from collections.abc import Iterable
from typing import TYPE_CHECKING

from dimos.manipulation.planning.groups.discovery import FALLBACK_PLANNING_GROUP_NAME
from dimos.manipulation.planning.groups.identifiers import (
    make_global_joint_names,
    make_planning_group_id,
)
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.config import RobotModelConfig


class PlanningGroupRegistry:
    """Registry of public planning groups derived from robot configs."""

    def __init__(self, robot_configs: Iterable[RobotModelConfig] = ()) -> None:
        self._groups: dict[PlanningGroupID, PlanningGroup] = {}
        self._groups_by_robot: dict[RobotName, list[PlanningGroup]] = {}
        for config in robot_configs:
            self.add_robot(config)

    def add_robot(self, config: RobotModelConfig) -> None:
        """Register all planning groups declared by one robot config."""
        if config.name in self._groups_by_robot:
            raise ValueError(f"Robot '{config.name}' is already registered")

        robot_groups: list[PlanningGroup] = []
        for definition in config.planning_groups:
            group_id = make_planning_group_id(config.name, definition.name)
            if group_id in self._groups:
                raise ValueError(f"Planning group '{group_id}' is already registered")
            group = PlanningGroup(
                id=group_id,
                robot_name=config.name,
                group_name=definition.name,
                joint_names=tuple(make_global_joint_names(config.name, definition.joint_names)),
                local_joint_names=definition.joint_names,
                base_link=definition.base_link,
                tip_link=definition.tip_link,
                source=definition.source,
            )
            self._groups[group_id] = group
            robot_groups.append(group)
        self._groups_by_robot[config.name] = robot_groups

    def list(self) -> tuple[PlanningGroup, ...]:
        """List planning groups in robot registration order."""
        groups: list[PlanningGroup] = []
        for robot_groups in self._groups_by_robot.values():
            groups.extend(robot_groups)
        return tuple(groups)

    def get(self, group_id: PlanningGroupID) -> PlanningGroup:
        """Return one planning group by public ID."""
        try:
            return self._groups[group_id]
        except KeyError as exc:
            raise KeyError(f"Unknown planning group ID: {group_id}") from exc

    def select(self, group_ids: Iterable[PlanningGroupID]) -> PlanningGroupSelection:
        """Validate and return an ordered planning-group selection."""
        return PlanningGroupSelection.from_groups(
            tuple(self.get(group_id) for group_id in group_ids)
        )

    def groups_for_robot(self, robot_name: RobotName) -> tuple[PlanningGroup, ...]:
        """Return planning groups for one robot."""
        return tuple(self._groups_by_robot.get(robot_name, ()))

    def default_group_id_for_robot(self, robot_name: RobotName) -> PlanningGroupID | None:
        """Return the generated fallback group ID for robot-scoped wrappers."""
        group_id = make_planning_group_id(robot_name, FALLBACK_PLANNING_GROUP_NAME)
        return group_id if group_id in self._groups else None

    def primary_pose_group_id_for_robot(self, robot_name: RobotName) -> PlanningGroupID | None:
        """Return the first pose-targetable group ID for compatibility paths."""
        # TODO: Replace this compatibility selection with either one TF publication per
        # pose-targetable planning group or backend-level whole-robot TF publishing.
        for group in self.groups_for_robot(robot_name):
            if group.has_pose_target:
                return group.id
        return None
