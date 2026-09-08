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

"""Piper connection ownership and command-port variants."""

from dataclasses import replace
from typing import ClassVar

from dimos.control.components import HardwareComponent
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.robot.manipulators.common.connection import SingleArmCoordinator
from dimos.robot.manipulators.common.coordinators import ArmPoseCoordinator, ArmTwistCoordinator
from dimos.robot.manipulators.piper.config import piper_hardware


def resolve_piper(
    component: HardwareComponent, address: str | None, simulation: str
) -> HardwareComponent:
    resolved = piper_hardware(address=address, simulation=simulation)
    return replace(
        component,
        adapter_type=resolved.adapter_type,
        address=resolved.address,
        limits=component.limits if resolved.adapter_type == "mock" else None,
    )


class PiperCoordinator(SingleArmCoordinator):
    supports_simulation: ClassVar[bool] = True

    def _resolve_hardware(self, component: HardwareComponent) -> HardwareComponent:
        return resolve_piper(
            component,
            self.config.address,
            self.config.g.simulation if self.supports_simulation else "",
        )


class PiperTwistCoordinator(PiperCoordinator, ArmTwistCoordinator):
    supports_simulation = False


class PiperPoseCoordinator(PiperCoordinator, ArmPoseCoordinator):
    supports_simulation = False


class PiperTeleopCoordinator(PiperCoordinator, TeleopControlCoordinator):
    pass
