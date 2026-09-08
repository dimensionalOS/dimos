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

"""Typed connection backends; importing schemas never loads robot implementations."""

from typing import Annotated, Literal, Self

from pydantic import AfterValidator, Field, model_validator

from dimos.protocol.service.spec import BaseConfig


def validate_address(value: str | None) -> str | None:
    if value is not None and (not value.strip() or value != value.strip()):
        raise ValueError("Device address must be nonempty and have no surrounding whitespace")
    return value


OptionalDeviceAddress = Annotated[str | None, AfterValidator(validate_address)]


class SingleAddressConfig(BaseConfig):
    address: OptionalDeviceAddress = None


class XArmConnectionConfig(SingleAddressConfig):
    # The hardware-only variant preserves blueprints without simulator wiring.
    backend: Literal["xarm", "xarm_hardware"] = "xarm"
    dof: Literal[6, 7] = 7


class PiperConnectionConfig(SingleAddressConfig):
    backend: Literal["piper", "piper_hardware"] = "piper"


class A1ZConnectionConfig(SingleAddressConfig):
    backend: Literal["a1z"] = "a1z"


class A750ConnectionConfig(SingleAddressConfig):
    backend: Literal["a750"] = "a750"


class OpenYamConnectionConfig(SingleAddressConfig):
    backend: Literal["openyam"] = "openyam"


class DualXArmConnectionConfig(BaseConfig):
    backend: Literal["dual_xarm"] = "dual_xarm"
    left_address: OptionalDeviceAddress = None
    right_address: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.left_address is None) != (self.right_address is None):
            raise ValueError("Supply both left and right addresses, or neither for mock hardware")
        return self


class PairedCanConfig(BaseConfig):
    left_can_port: OptionalDeviceAddress = None
    right_can_port: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.left_can_port is None) != (self.right_can_port is None):
            raise ValueError("Supply both left and right CAN ports, or neither for mock hardware")
        if self.left_can_port is not None and self.left_can_port == self.right_can_port:
            raise ValueError("Left and right CAN ports must be distinct")
        return self


class OpenArmConnectionConfig(PairedCanConfig):
    backend: Literal["openarm"] = "openarm"


class DualOpenYamConnectionConfig(PairedCanConfig):
    backend: Literal["dual_openyam"] = "dual_openyam"


class MixedArmConnectionConfig(BaseConfig):
    backend: Literal["xarm_piper"] = "xarm_piper"
    xarm_address: OptionalDeviceAddress = None
    piper_address: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.xarm_address is None) != (self.piper_address is None):
            raise ValueError("Supply both xArm and Piper addresses, or neither for mock hardware")
        return self


HardwareConnectionConfig = Annotated[
    XArmConnectionConfig
    | PiperConnectionConfig
    | A1ZConnectionConfig
    | A750ConnectionConfig
    | OpenYamConnectionConfig
    | DualXArmConnectionConfig
    | OpenArmConnectionConfig
    | DualOpenYamConnectionConfig
    | MixedArmConnectionConfig,
    Field(discriminator="backend"),
]
