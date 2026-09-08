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

"""xArm connection ownership and command-port variants."""

from dataclasses import replace
from typing import ClassVar

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.robot.manipulators.common.connection import (
    PairedArmCoordinatorConfig,
    SingleArmCoordinator,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.config import xarm6_hardware, xarm7_hardware


def resolve_xarm(
    component: HardwareComponent, dof: int, address: str | None, simulation: str
) -> HardwareComponent:
    factory = xarm7_hardware if dof == 7 else xarm6_hardware
    resolved = factory(address=address, simulation=simulation)
    return replace(
        component,
        adapter_type=resolved.adapter_type,
        address=resolved.address,
        limits=component.limits if resolved.adapter_type == "mock" else None,
        adapter_kwargs={**component.adapter_kwargs, "arm_dof": dof},
    )


class XArm6Coordinator(SingleArmCoordinator):
    arm_dof: ClassVar[int] = 6
    supports_simulation: ClassVar[bool] = True

    def _resolve_hardware(self, component: HardwareComponent) -> HardwareComponent:
        return resolve_xarm(
            component,
            self.arm_dof,
            self.config.address,
            self.config.g.simulation if self.supports_simulation else "",
        )


class XArm7Coordinator(XArm6Coordinator):
    arm_dof = 7


class XArm6HardwareCoordinator(XArm6Coordinator):
    supports_simulation = False


class XArm6TwistCoordinator(XArm6Coordinator, ArmTwistCoordinator):
    pass


class XArm7TwistCoordinator(XArm7Coordinator, ArmTwistCoordinator):
    pass


class XArm6TeleopCoordinator(XArm6Coordinator, TeleopControlCoordinator):
    pass


class XArm7TeleopCoordinator(XArm7Coordinator, TeleopControlCoordinator):
    pass


class DualXArmCoordinator(ControlCoordinator):
    config: PairedArmCoordinatorConfig

    def _setup_from_config(self) -> None:
        left, right = self.config.hardware
        self.config.hardware = [
            resolve_xarm(left, 7, self.config.left_address, self.config.g.simulation),
            resolve_xarm(right, 6, self.config.right_address, self.config.g.simulation),
        ]
        super()._setup_from_config()
