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

"""An ideal position-controlled joint connection for software simulation."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


class MockJointConnectionConfig(ConnectionConfig):
    source: str = "arm"
    model: RobotModel
    initial_positions: dict[str, float] = Field(default_factory=dict)
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def declare_position_joints(self) -> MockJointConnectionConfig:
        """This concrete connection drives every movable model joint by position."""
        if "joint_commands" not in self.model_fields_set:
            self.joint_commands = {
                joint.name: (Interface.POSITION,) for joint in movable_joints(self.model)
            }
        return self


class MockJointConnection(ConnectionModule):
    """Echo accepted position targets as measured joints; no dynamics or device SDK."""

    config: MockJointConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._positions: dict[str, float] = {}
        self._frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    def connect(self) -> None:
        names = [joint.name for joint in movable_joints(self.config.model)]
        positions = self.config.initial_positions or dict.fromkeys(names, 0.0)
        if set(positions) != set(names) or not np.isfinite(list(positions.values())).all():
            raise ValueError("Initial positions must give one finite value for every model joint")
        self._positions = {name: positions[name] for name in names}
        self._frame = self.frame(self.config.model.load().root_link)

    @rpc
    def start(self) -> None:
        try:
            super().start()
            self.register_disposable(Disposable(self.position_command.subscribe(self._on_position)))
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()
        except Exception:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_position(self, message: JointState) -> None:
        try:
            values = joint_state_to_values(message, (Interface.POSITION,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

    def write(self, values: Values) -> None:
        prefix = f"{self.config.source}/"
        self._positions.update(
            {
                key.removeprefix(prefix).removesuffix("/position"): value
                for key, value in values.items()
            }
        )

    def halt(self) -> None:
        """Positions already hold where they were last commanded."""

    def _read_loop(self) -> None:
        while not self._stop_reading.is_set():
            with self._hardware_lock:
                positions = dict(self._positions)
            self.joint_state.publish(
                JointState(
                    ts=time.time(),
                    frame_id=self._frame,
                    name=[f"{self.config.source}/{name}" for name in positions],
                    position=list(positions.values()),
                    velocity=[0.0] * len(positions),
                    effort=[0.0] * len(positions),
                )
            )
            self._stop_reading.wait(1.0 / self.config.state_rate_hz)
