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

"""Runtime CAN selection for the Seeed B601 coordinator."""

from __future__ import annotations

from typing import Literal

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.robot.manipulators.seeed_b601_dm.config import b601_dm_hardware


class B601DMCoordinatorConfig(ControlCoordinatorConfig):
    can_port: str | None = None
    gripper_opening_direction: Literal["increasing_position", "decreasing_position"] | None = None
    gripper_current: float | None = None


class B601DMCoordinator(ControlCoordinator):
    """Use in-memory hardware unless explicit CAN and gripper settings are supplied."""

    config: B601DMCoordinatorConfig

    def _setup_from_config(self) -> None:
        self.config.hardware = [
            b601_dm_hardware(
                can_port=self.config.can_port,
                gripper_opening_direction=self.config.gripper_opening_direction,
                gripper_current=self.config.gripper_current,
            )
        ]
        super()._setup_from_config()
