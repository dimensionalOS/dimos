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

"""Metal connection settings: one CAN bus and the Metal robot data."""

from __future__ import annotations

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.makermods_metal.config import METAL


class MetalConnectionConfig(DamiaoConnectionConfig):
    source: str = "metal"
    can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = METAL.model
        return self


class MetalConnection(DamiaoConnection):
    """One Metal arm and jaw with the MakerMods firmware profiles."""

    config: MetalConnectionConfig
    robot = METAL

    def can_ports(self) -> list[str]:
        return [self.config.can_port]
