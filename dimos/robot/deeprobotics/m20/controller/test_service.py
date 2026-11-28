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

"""The service's launch line and its client agree with M20Controller."""

import json
from typing import get_type_hints

from dimos.robot.deeprobotics.m20.controller.module import M20Controller, M20ControllerConfig
from dimos.robot.deeprobotics.m20.controller.service import (
    SERVICE_PREFIX,
    M20ControllerClient,
    launch_line,
    ports,
)


def test_the_launch_line_puts_every_port_under_the_service_prefix() -> None:
    blob = json.loads(launch_line(M20ControllerConfig(mode="owner", policy=True)))
    inputs, outputs = ports()
    assert set(blob["topics"]) == set(inputs) | set(outputs)
    assert blob["topics"]["cmd_vel"] == SERVICE_PREFIX + "cmd_vel/geometry_msgs.Twist"
    assert blob["config"]["mode"] == "owner" and blob["config"]["policy"] is True
    assert "navigation_terrain" not in blob["config"]


def test_the_client_has_the_controllers_ports() -> None:
    def streams(cls: type) -> dict[str, object]:
        return {
            k: v
            for k, v in get_type_hints(cls).items()
            if k not in ("config",) and hasattr(v, "__origin__")
        }

    assert streams(M20ControllerClient).keys() >= {
        "cmd_vel",
        "joint_state",
        "controller_report",
        "usage_mode",
    }
    assert {
        k: str(v) for k, v in streams(M20ControllerClient).items() if k in streams(M20Controller)
    } == {k: str(v) for k, v in streams(M20Controller).items() if k in streams(M20ControllerClient)}
