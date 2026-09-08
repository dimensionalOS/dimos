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

"""Assembly-local connection settings and dispatch to robot-owned factories."""

from dataclasses import replace
from importlib import import_module
from typing import Annotated, Literal, Self

from pydantic import AfterValidator, Field, model_validator

from dimos.control.components import HardwareComponent
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.protocol.service.spec import BaseConfig


def validate_address(value: str | None) -> str | None:
    if value is not None and (not value.strip() or value != value.strip()):
        raise ValueError("Device address must be nonempty and have no surrounding whitespace")
    return value


OptionalDeviceAddress = Annotated[str | None, AfterValidator(validate_address)]


class SingleArmConnectionConfig(BaseConfig):
    backend: Literal[
        "xarm6", "xarm7", "xarm6_hardware", "piper", "piper_hardware", "a1z", "a750", "openyam"
    ]
    address: OptionalDeviceAddress = None


class PairedConnectionConfig(BaseConfig):
    backend: Literal["dual_xarm"] = "dual_xarm"
    left_address: OptionalDeviceAddress = None
    right_address: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.left_address is None) != (self.right_address is None):
            raise ValueError("Supply both left and right addresses, or neither for mock hardware")
        return self


class PairedCanConnectionConfig(BaseConfig):
    backend: Literal["openarm", "dual_openyam"]
    left_can_port: OptionalDeviceAddress = None
    right_can_port: OptionalDeviceAddress = None

    @model_validator(mode="after")
    def complete_pair(self) -> Self:
        if (self.left_can_port is None) != (self.right_can_port is None):
            raise ValueError("Supply both left and right CAN ports, or neither for mock hardware")
        if self.left_can_port is not None and self.left_can_port == self.right_can_port:
            raise ValueError("Left and right CAN ports must be distinct")
        return self


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
    SingleArmConnectionConfig
    | PairedConnectionConfig
    | PairedCanConnectionConfig
    | MixedArmConnectionConfig,
    Field(discriminator="backend"),
]


def merge_hardware(component: HardwareComponent, resolved: HardwareComponent) -> HardwareComponent:
    """Apply connection selection without replacing blueprint-owned hardware settings."""
    kwargs = {**resolved.adapter_kwargs, **component.adapter_kwargs}
    runtime = resolved.adapter_kwargs.get("runtime_config")
    if isinstance(runtime, DamiaoRuntimeConfig):
        kwargs["runtime_config"] = replace(
            component.adapter_kwargs.get("runtime_config", runtime), bus_devices=runtime.bus_devices
        )
    return replace(
        component,
        adapter_type=resolved.adapter_type,
        address=resolved.address,
        limits=component.limits if resolved.adapter_type in ("mock", "mock_whole_body") else None,
        adapter_kwargs=kwargs,
    )


def resolve_connection(
    config: HardwareConnectionConfig, hardware: list[HardwareComponent], simulation: str
) -> list[HardwareComponent]:
    counts = (
        (0, 1)
        if isinstance(config, PairedCanConnectionConfig)
        else ((1,) if isinstance(config, SingleArmConnectionConfig) else (2,))
    )
    if len(hardware) not in counts:
        raise ValueError(f"{config.backend} requires {counts} hardware descriptions")
    backend = config.backend.removesuffix("_hardware")
    module = {
        "xarm6": "xarm.config",
        "xarm7": "xarm.config",
        "dual_xarm": "xarm.config",
        "xarm_piper": "common.mixed",
    }.get(backend, f"{backend}.config")
    factory = import_module(f"dimos.robot.manipulators.{module}").resolve_connection
    result: list[HardwareComponent] = factory(config, hardware, simulation)
    return result
