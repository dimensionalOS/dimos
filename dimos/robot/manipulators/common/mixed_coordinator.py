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

"""Connection selection for independent xArm/Piper assemblies."""

from typing import Self

from pydantic import model_validator

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.robot.manipulators.common.connection import OptionalDeviceAddress
from dimos.robot.manipulators.piper.coordinator import resolve_piper
from dimos.robot.manipulators.xarm.coordinator import resolve_xarm


class MixedArmCoordinatorConfig(ControlCoordinatorConfig):
    xarm_address: OptionalDeviceAddress = None
    piper_address: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.xarm_address is None) != (self.piper_address is None):
            raise ValueError("Supply both xArm and Piper addresses, or neither for mock hardware")
        return self


class MixedArmCoordinator(ControlCoordinator):
    config: MixedArmCoordinatorConfig

    def _setup_from_config(self) -> None:
        xarm, piper = self.config.hardware
        self.config.hardware = [
            resolve_xarm(xarm, 6, self.config.xarm_address, ""),
            resolve_piper(piper, self.config.piper_address, ""),
        ]
        super()._setup_from_config()
