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

"""OpenYAM connection settings: one CAN bus and the OpenYAM robot data."""

from __future__ import annotations

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM,
)


class OpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "yam"
    can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = OPENYAM.model
        return self


class OpenYamConnection(DamiaoConnection):
    """One OpenYAM arm and jaw on a Damiao bus.

    With ``teach=True`` the arm runs with zero stiffness and teaching damping,
    gravity is compensated, and the jaw is passive: it follows the operator
    instead of holding an opening.
    """

    config: OpenYamConnectionConfig
    robot = OPENYAM

    def can_ports(self) -> list[str]:
        return [self.config.can_port]
