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
``joint_state``. Its joints are wherever they were last told to be, at once.
It keeps the latest commands passed to ``write`` and counts calls to
``halt``, so a test can see what reached the "hardware".
"""

from __future__ import annotations

from collections import deque
import math
from typing import Any

from dimos.control.connection.connection_module import ConnectionModule
from dimos.control.contract.description import ControlDescription, Limits
from dimos.control.contract.keys import POSITION, Key
from dimos.control.contract.presets import manipulator_description
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState


class MockConnectionConfig(ModuleConfig):
    """Settings for ``MockConnectionModule``.

    Attributes:
        source: The arm's name, the first part of every key, e.g. "mock".
        joints: How many joints, named joint1, joint2, ...
        position_limit: How far each joint may be told to go either side of
            0, in radians. A command past it is refused.
        state_rate_hz: How many times a second it reports its joints.
        deadman_timeout_s: How long it waits for a command, in seconds,
            before halting.
    """

    source: str = "mock"
    joints: int = 7
    position_limit: float = math.pi
    state_rate_hz: float = 100.0
    deadman_timeout_s: float = 0.1


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

    def describe(self) -> ControlDescription | list[ControlDescription]:
        source = self.config.source
        joints = [f"joint{i}" for i in range(1, self.config.joints + 1)]
        limit = self.config.position_limit
        return manipulator_description(
            source,
            joints,
            limits={Key.of(source, joint, POSITION): Limits(-limit, limit) for joint in joints},
            state=(POSITION,),
            command=(POSITION,),
            state_rate_hz=self.config.state_rate_hz,
            deadman_timeout_s=self.config.deadman_timeout_s,
        )

    def connect(self) -> None:
        descriptions = self.describe_control().descriptions
        self._readings = {key: 0.0 for d in descriptions for key in d.state_keys()}

    def read_state(self) -> dict[str, float] | None:
        return dict(self._readings)

    def write(self, values: dict[str, float]) -> None:
        self.writes.append(dict(values))
        # Replaced whole, so the reading thread never sees it half-changed.
        # Only keys it reports are kept, so its readings stay valid.
        self._readings = {key: values.get(key, value) for key, value in self._readings.items()}

    def halt(self) -> None:
        self.halts += 1
