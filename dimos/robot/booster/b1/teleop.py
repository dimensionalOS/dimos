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

"""NativeModule wrapper for Booster B1 teleoperation."""

from __future__ import annotations

import os

from pydantic import Field

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist

_DEFAULT_COMMAND_TIMEOUT_SEC = 0.25
_BUILD_COMMAND = "nix build .#booster-b1-teleop-native"


def network_interface_from_env() -> str:
    network_interface = os.environ.get("ROBOT_INTERFACE")
    if not network_interface:
        raise ValueError(
            "ROBOT_INTERFACE must name the network interface connected to the Booster B1"
        )
    return network_interface


class BoosterB1TeleopConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/booster_b1_teleop_native"
    build_command: str | None = _BUILD_COMMAND
    log_format: LogFormat = LogFormat.TEXT
    network_interface: str = Field(default_factory=network_interface_from_env)
    command_timeout_sec: float = Field(default=_DEFAULT_COMMAND_TIMEOUT_SEC, gt=0.0)


class BoosterB1Teleop(NativeModule):
    """Booster B1 teleoperation backed by the native C++ SDK."""

    config: BoosterB1TeleopConfig

    cmd_vel: In[Twist]
