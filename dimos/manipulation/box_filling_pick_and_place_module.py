# Copyright 2026 Dimensional Inc.
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

"""Box-filling application policy layered on generic pick and place."""

from __future__ import annotations

from dataclasses import dataclass

from pydantic import Field, FiniteFloat

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.manipulation.pick_and_place_module import (
    PickAndPlaceModule,
    PickAndPlaceModuleConfig,
)
from dimos.manipulation.skill_errors import ManipulationSkillError


class BoxFillingPickAndPlaceModuleConfig(PickAndPlaceModuleConfig):
    """Configuration for destination-container placement policy."""

    box_wall_thickness: FiniteFloat = Field(default=0.01, gt=0.0)
    drop_clearance: FiniteFloat = Field(default=0.02, ge=0.0)


@dataclass(frozen=True)
class _DestinationBox:
    snapshot_version: int
    center_x: float
    center_y: float
    rim_z: float
    opening_width: float
    opening_depth: float


class BoxFillingPickAndPlaceModule(PickAndPlaceModule):
    """Add destination-box selection and fit-checked placement."""

    config: BoxFillingPickAndPlaceModuleConfig

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._destination_box: _DestinationBox | None = None

    @skill
    def select_destination_container(self, number: int) -> SkillResult[ManipulationSkillError]:
        """Use one object from the latest scan as the destination open box.

        Args:
            number: One-based object number returned by the latest scan_objects call.
        """
        if number < 1 or number > len(self._detection_snapshot):
            return SkillResult.fail("INVALID_INPUT", f"No detected object numbered {number}")
        box = self._detection_snapshot[number - 1]
        wall = float(self.config.box_wall_thickness)
        opening_width = box.size.x - 2.0 * wall
        opening_depth = box.size.y - 2.0 * wall
        if opening_width <= 0.0 or opening_depth <= 0.0:
            return SkillResult.fail("INVALID_INPUT", "Container opening is too small")
        self._destination_box = _DestinationBox(
            snapshot_version=self._snapshot_version,
            center_x=box.center.x,
            center_y=box.center.y,
            rim_z=box.center.z + box.size.z / 2.0,
            opening_width=opening_width,
            opening_depth=opening_depth,
        )
        return SkillResult.ok(
            f"Selected {number}. {box.name} as destination container",
            number=number,
            opening_width=opening_width,
            opening_depth=opening_depth,
            rim_z=self._destination_box.rim_z,
        )

    @skill
    def place_in_destination(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Place the verified held object above the selected box opening.

        Args:
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        destination = self._destination_box
        if destination is None:
            return SkillResult.fail(
                "INVALID_STATE", "Select a destination container before placing"
            )
        if destination.snapshot_version != self._snapshot_version:
            self._destination_box = None
            return SkillResult.fail(
                "INVALID_STATE", "Destination is stale; scan and select it again"
            )
        held_size = self._held_object_size
        if held_size is None:
            return SkillResult.fail(
                "INVALID_STATE", "No verified held object is available to place"
            )
        if held_size.x > destination.opening_width or held_size.y > destination.opening_depth:
            return SkillResult.fail(
                "INVALID_INPUT", "Held object does not fit inside the destination opening"
            )
        object_z = destination.rim_z + held_size.z / 2.0 + float(self.config.drop_clearance)
        return self.place_at(
            destination.center_x,
            destination.center_y,
            object_z,
            robot_name,
        )
