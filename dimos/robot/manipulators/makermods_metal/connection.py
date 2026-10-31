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

"""Metal on the shared Damiao connection lifecycle."""

from __future__ import annotations

from dataclasses import replace

from pydantic import model_validator
from typing_extensions import Self

from dimos.control.components import HardwareComponent
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
    damiao_control_model,
)
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_MODEL,
    metal_hardware,
)


def metal_connection_model(*, robot: str | None = None) -> RobotModel:
    """The existing arm model and normalized grippers, optionally namespaced."""
    model = damiao_control_model(METAL_MODEL, ("gripper",))
    if robot is not None:
        names = (*METAL_ARM_JOINTS, "gripper")
        model = model.with_renamed_joints({name: f"{robot}/{name}" for name in names})
    return model


class MetalConnectionConfig(DamiaoConnectionConfig):
    source: str = "metal"
    can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = metal_connection_model()
        return self


class MetalConnection(DamiaoConnection):
    config: MetalConnectionConfig

    def hardware_component(self) -> HardwareComponent:
        component = metal_hardware(can_port=self.config.can_port)
        return replace(component, joints=[*METAL_ARM_JOINTS, "gripper"])
