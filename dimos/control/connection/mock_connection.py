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

"""A pretend robot arm, for running and testing without hardware.

It takes joint positions on ``position_command`` and reports them on
``joint_state``. Its joints are wherever they were last told to be, at once,
and it reports every other reading as the last value it was told, or zero.
It keeps the latest commands passed to ``write`` and counts calls to
``halt``, so a test can see what reached the "hardware".
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Any

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.ports import JointPort
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel

#: Seven turning joints, joint1 to joint7, each allowed one turn either way.
MOCK_ARM_MODEL = RobotModel.from_file(Path(__file__).with_name("mock_arm.urdf"))


class MockConnectionConfig(ConnectionConfig):
    """Settings for ``MockConnectionModule``.

    Attributes:
        source: The arm's name, the first part of every key.
        model: The arm's model; the seven-joint ``MOCK_ARM_MODEL`` unless
            replaced.
        position_command: What the position port drives: every joint in the
            model unless told otherwise.
    """

    source: str = "mock"
    model: RobotModel | None = MOCK_ARM_MODEL
    position_command: JointPort = JointPort()


class MockConnectionModule(ConnectionModule):
    """A pretend arm, driven by joint position. See the module docstring."""

    config: MockConnectionConfig

    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        #: The last 1000 commands passed to ``write``, oldest first.
        self.writes: deque[dict[str, float]] = deque(maxlen=1000)
        #: How many times ``halt`` has been called.
        self.halts = 0
        self._readings: dict[str, float] = {}

    def connect(self) -> None:
        self._readings = dict.fromkeys(self.description.state_keys(), 0.0)

    def read_state(self) -> dict[str, float]:
        return dict(self._readings)

    def write(self, values: dict[str, float]) -> None:
        self.writes.append(dict(values))
        # Replaced whole, so the reading thread never sees it half-changed.
        # Only keys it reports are kept, so its readings stay valid.
        self._readings = {key: values.get(key, value) for key, value in self._readings.items()}

    def halt(self) -> None:
        self.halts += 1
