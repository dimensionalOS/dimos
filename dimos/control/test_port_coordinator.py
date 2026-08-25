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

from dimos.control.port_coordinator import PortControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class _ConnectionModule(Module):
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]


def test_port_coordinator_and_connection_have_complementary_typed_ports() -> None:
    blueprint = autoconnect(
        PortControlCoordinator.blueprint(),
        _ConnectionModule.blueprint(),
    )

    directions = {
        name: sorted(
            stream.direction
            for atom in blueprint.blueprints
            for stream in atom.streams
            if stream.name == name
        )
        for name in ("motor_command", "motor_states")
    }

    assert directions == {
        "motor_command": ["in", "out"],
        "motor_states": ["in", "out"],
    }
