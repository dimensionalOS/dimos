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

"""OpenYam on the shared Damiao connection lifecycle."""

from __future__ import annotations

from dataclasses import replace

from pydantic import model_validator
from typing_extensions import Self

from dimos.control.components import HardwareComponent
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
    damiao_control_model,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    make_openyam_model_config,
    openyam_hardware,
)


def yam_connection_model(*, robot: str | None = None) -> RobotModel:
    """The existing arm model and normalized grippers, optionally namespaced."""
    model = damiao_control_model(
        make_openyam_model_config().model.with_fixed_joints("joint7", "joint8"), ("gripper",)
    )
    if robot is not None:
        names = (*OPENYAM_ARM_JOINTS, "gripper")
        model = model.with_renamed_joints({name: f"{robot}/{name}" for name in names})
    return model


class OpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "yam"
    can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = yam_connection_model()
        return self


class OpenYamConnection(DamiaoConnection):
    config: OpenYamConnectionConfig

    def hardware_component(self) -> HardwareComponent:
        component = openyam_hardware(
            bus_devices={"openyam": self.config.can_port} if self.config.can_port else None
        )
        component = replace(
            component,
            adapter_type="mock_whole_body" if self.config.can_port is None else "openyam_damiao",
            adapter_kwargs={}
            if self.config.can_port is None
            else {
                "runtime_config": DamiaoRuntimeConfig(
                    bus_devices={"openyam": self.config.can_port}, gravity_comp=True
                )
            },
        )
        return replace(component, joints=[*OPENYAM_ARM_JOINTS, "gripper"])
