# Copyright 2025-2026 Dimensional Inc.
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

"""Shared config types for Go2 backend connections.

Kept in one place so the registry-based backend swap sees the same
`ConnectionConfig.model_fields` shape across `Go2WebRtcConnection`,
`Go2MujocoConnection`, `Go2ReplayConnection`, and `Go2FleetConnection`.
"""

from __future__ import annotations

from enum import Enum

from pydantic import Field

from dimos.core.module import ModuleConfig


class Go2Mode(str, Enum):
    DEFAULT = "default"
    RAGE = "rage"


class ConnectionConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT
