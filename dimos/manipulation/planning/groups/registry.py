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

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.spec.models import PlanningGroupID

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.config import RobotModelConfig


class PlanningGroupRegistry:
    """Registry of public planning groups derived from one model config."""

    def __init__(self, model_configs: Iterable[RobotModelConfig] = ()) -> None:
        self._groups: dict[PlanningGroupID, PlanningGroup] = {}
        for config in model_configs:
            self.add_model(config)

    def add_model(self, config: RobotModelConfig) -> None:
        """Register all planning groups declared by the model config."""
        if self._groups:
            raise ValueError("A model is already registered")
        for definition in config.planning_groups:
            group_id = definition.name
            if group_id in self._groups:
                raise ValueError(f"Planning group '{group_id}' is already registered")
            group = PlanningGroup(
                id=group_id,
                joint_names=definition.joint_names,
                base_link=definition.base_link,
                tip_link=definition.tip_link,
                source=definition.source,
            )
            self._groups[group_id] = group

    def list(self) -> tuple[PlanningGroup, ...]:
        """List planning groups in robot registration order."""
        return tuple(self._groups.values())

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

    def default_group_id(self) -> PlanningGroupID | None:
        """Return the sole group ID when selection is unambiguous."""
        groups = self.list()
        return groups[0].id if len(groups) == 1 else None

    def primary_pose_group_id(self) -> PlanningGroupID | None:
        """Return the unique pose-targetable group ID."""
        pose_groups = [group for group in self.list() if group.has_pose_target]
        if not pose_groups:
            return None
        if len(pose_groups) > 1:
            raise ValueError(
                f"Model has {len(pose_groups)} pose-targetable planning groups; "
                "use an explicit planning group ID"
            )
        return pose_groups[0].id
