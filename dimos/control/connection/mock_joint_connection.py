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

"""An ideal joint connection for software simulation: commands come straight back as readings."""

from __future__ import annotations

import functools
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
from dimos.robot.assets.model import JointDescription, RobotModel

_ECHOED = ((Interface.POSITION,), (Interface.VELOCITY,))


def mock_base_model(source: str = "base") -> RobotModel:
    """A holonomic base's three body speeds, ``<source>/base_forward``,
    ``<source>/base_lateral`` (up to 1 m/s) and ``<source>/base_turn`` (up to 2 rad/s).

    They take speeds only and have no position.
    """
    axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    return RobotModel.from_root_link("base_link", name="mock_base").with_added_joints(
        *(
            JointDescription(
                name=f"{source}/base_{axis_name}",
                type="continuous" if axis_name == "turn" else "prismatic",
                parent_link="base_link",
                child_link=f"mock_base_{axis_name}",
                axis=axis,
                velocity=speed,
            )
            for axis_name, axis, speed in zip(
                ("forward", "lateral", "turn"), axes, (1.0, 1.0, 2.0), strict=True
            )
        )
    )


def mock_arm_model(joints: int) -> RobotModel:
    """A chain of ``joints`` turning joints, joint1 first, for a mock arm.

    Each may go to any angle within half a turn either side of 0 (radians),
    at up to 1 rad/s.
    """
    return RobotModel.from_root_link("link0", name="mock_arm").with_added_joints(
        *(
            JointDescription(
                name=f"joint{i + 1}",
                type="revolute",
                parent_link=f"link{i}",
                child_link=f"link{i + 1}",
                lower=-np.pi,
                upper=np.pi,
                velocity=1.0,
            )
            for i in range(joints)
        )
    )


class MockJointConnectionConfig(ConnectionConfig):
    """What the mock is told and how it starts.

    Attributes:
        initial_positions: Starting angle (rad) or distance (m) of every
            position-driven joint, by model joint name. Empty starts them at 0.
        state_rate_hz: How many times a second readings go out.
    """

    source: str = "arm"
    model: RobotModel
    initial_positions: dict[str, float] = Field(default_factory=dict)
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def declare_position_joints(self) -> MockJointConnectionConfig:
        """Drive every movable model joint by position unless told otherwise."""
        if "joint_commands" not in self.model_fields_set:
            self.joint_commands = {
                joint.name: (Interface.POSITION,) for joint in movable_joints(self.model)
            }
        for name, interfaces in self.joint_commands.items():
            if tuple(interfaces) not in _ECHOED:
                raise ValueError(
                    f"{name}: the mock takes a position or a velocity, not {interfaces}"
                )
        return self


class MockJointConnection(ConnectionModule):
    """Report each accepted command back as the joint's reading; no dynamics or device SDK.

    A position-driven joint reports its last target as its position. A
    velocity-driven joint reports its last commanded speed and no position;
    when commands stop it reports zero.
    """

    config: MockJointConnectionConfig
    position_command: In[JointState]
    velocity_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._positions: dict[str, float] = {}
        self._speeds: dict[str, float] = {}
        self._frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    def connect(self) -> None:
        commands = self.config.joint_commands
        names = [j.name for j in movable_joints(self.config.model) if j.name in commands]
        driven = [name for name in names if commands[name] == (Interface.POSITION,)]
        positions = self.config.initial_positions or dict.fromkeys(driven, 0.0)
        if set(positions) != set(driven) or not np.isfinite(list(positions.values())).all():
            raise ValueError(
                "Initial positions must give one finite value for every position joint"
            )
        self._positions = {self._full_name(name): positions[name] for name in driven}
        self._speeds = {self._full_name(name): 0.0 for name in names if name not in positions}
        self._frame = self.frame(self.config.model.load().root_link)

    @rpc
    def start(self) -> None:
        try:
            super().start()
            declared = {i for interfaces in self.config.joint_commands.values() for i in interfaces}
            for port, interface in (
                (self.position_command, Interface.POSITION),
                (self.velocity_command, Interface.VELOCITY),
            ):
                if interface not in declared:
                    continue
                self.register_disposable(
                    Disposable(port.subscribe(functools.partial(self._on_command, interface)))
                )
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

    def _on_command(self, interface: Interface, message: JointState) -> None:
        try:
            values = joint_state_to_values(message, (interface,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

    def write(self, values: Values) -> None:
        for key, value in values.items():
            joint, _, interface = key.rpartition("/")
            target = self._positions if interface == Interface.POSITION else self._speeds
            target[joint] = value

    def halt(self) -> None:
        """Positions hold where they were last commanded; speeds drop to zero."""
        self._speeds = dict.fromkeys(self._speeds, 0.0)

    def _read_loop(self) -> None:
        while not self._stop_reading.is_set():
            with self._hardware_lock:
                positions, speeds = dict(self._positions), dict(self._speeds)
            ts = time.time()
            if positions:
                self.joint_state.publish(
                    JointState(
                        ts=ts,
                        frame_id=self._frame,
                        name=list(positions),
                        position=list(positions.values()),
                        velocity=[0.0] * len(positions),
                        effort=[0.0] * len(positions),
                    )
                )
            if speeds:
                self.joint_state.publish(
                    JointState(
                        ts=ts,
                        frame_id=self._frame,
                        name=list(speeds),
                        velocity=list(speeds.values()),
                    )
                )
            self._stop_reading.wait(1.0 / self.config.state_rate_hz)

    def _full_name(self, joint: str) -> str:
        """A model joint's name as commands and readings carry it, e.g. "arm/joint1"."""
        return joint if "/" in joint else f"{self.config.source}/{joint}"
