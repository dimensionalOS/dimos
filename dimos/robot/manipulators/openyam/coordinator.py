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

"""OpenYAM local CAN connection configuration."""

from dataclasses import replace

from dimos.control.components import HardwareComponent
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.robot.manipulators.common.connection import SingleArmCoordinator
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.openyam.config import openyam_hardware


class OpenYamCoordinator(SingleArmCoordinator):
    def _resolve_hardware(self, component: HardwareComponent) -> HardwareComponent:
        resolved = openyam_hardware(
            address=self.config.address, simulation=self.config.g.simulation
        )
        return replace(
            component,
            adapter_type=resolved.adapter_type,
            address=resolved.address,
            adapter_kwargs=resolved.adapter_kwargs,
            limits=resolved.limits,
        )


class OpenYamTwistCoordinator(OpenYamCoordinator, ArmTwistCoordinator):
    pass


class OpenYamTeleopCoordinator(OpenYamCoordinator, TeleopControlCoordinator):
    pass
