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

"""Local connection configuration for manipulator assemblies."""

from abc import ABC, abstractmethod
from typing import Annotated, Self

from pydantic import AfterValidator, model_validator

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig


def validate_address(value: str | None) -> str | None:
    if value is not None and (not value.strip() or value != value.strip()):
        raise ValueError("Device address must be nonempty and have no surrounding whitespace")
    return value


OptionalDeviceAddress = Annotated[str | None, AfterValidator(validate_address)]


class SingleArmCoordinatorConfig(ControlCoordinatorConfig):
    address: OptionalDeviceAddress = None


class PairedArmCoordinatorConfig(ControlCoordinatorConfig):
    left_address: OptionalDeviceAddress = None
    right_address: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.left_address is None) != (self.right_address is None):
            raise ValueError("Supply both left and right addresses, or neither for mock hardware")
        return self


class PairedCanCoordinatorConfig(ControlCoordinatorConfig):
    left_can_port: OptionalDeviceAddress = None
    right_can_port: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.left_can_port is None) != (self.right_can_port is None):
            raise ValueError("Supply both left and right CAN ports, or neither for mock hardware")
        if self.left_can_port is not None and self.left_can_port == self.right_can_port:
            raise ValueError("Left and right CAN ports must be distinct")
        return self


class SingleArmCoordinator(ControlCoordinator, ABC):
    """Resolve the robot's connection after local configuration is available."""

    config: SingleArmCoordinatorConfig

    @abstractmethod
    def _resolve_hardware(self, component: HardwareComponent) -> HardwareComponent:
        """Resolve a blueprint's fixed hardware description using local connection settings."""
        ...

    def _setup_from_config(self) -> None:
        if len(self.config.hardware) != 1:
            raise ValueError("Single-arm coordinator requires exactly one hardware component")
        self.config.hardware = [self._resolve_hardware(self.config.hardware[0])]
        super()._setup_from_config()
