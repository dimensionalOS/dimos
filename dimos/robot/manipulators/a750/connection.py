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

"""An A-750 arm as a connection module, over its serial SDK, a750_control."""

from __future__ import annotations

import contextlib
import threading
import time
from typing import Any

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.a750.config import A750_DOF, a750_model

_READ_HZ = 100.0


class A750Config(ConnectionConfig):
    """Where the A-750 is plugged in, and how fast it moves.

    Attributes:
        device_path: The arm's serial port, e.g. "/dev/ttyACM0".
        source: The robot's name, in front of each joint's name, e.g. "arm"
            gives "arm/joint1".
        velocity_ratio: The speed sent with each joint target, as a fraction
            of the joint's top speed, from 0 to 1.
    """

    device_path: str
    source: str = "arm"
    joint_commands: dict[str, tuple[Interface, ...]] = {}
    velocity_ratio: float = Field(default=1.0, gt=0.0, le=1.0)

    @model_validator(mode="after")
    def _model_from_urdf(self) -> Self:
        if self.model is None:
            self.model = a750_model()
        if not self.joint_commands:
            # Every joint, finger included, takes positions only.
            self.joint_commands = {
                joint.name: (Interface.POSITION,) for joint in movable_joints(self.model)
            }
        return self


class A750Connection(ConnectionModule):
    """An A-750 driven over its serial port at ``config.device_path``.

    Joint targets come in on ``position_command`` in rad, the gripper as its
    joint ``finger`` in m (0.06 open). A joint left out keeps its last target.
    ``joint_state`` goes out about 100 times a second. From connecting until the
    first command, and whenever commands stop, the arm holds where it is.
    """

    config: A750Config

    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        prefix = f"{self.config.source}{SEPARATOR}"
        self._joints = [f"{prefix}joint{i}" for i in range(1, A750_DOF + 1)]
        self._finger = f"{prefix}finger"
        self._sdk: Any = None
        # The last targets sent: the joints' in rad, the finger's in m.
        self._targets: list[float] = []
        self._finger_target = 0.0
        # While True, the read loop re-sends the targets: the SDK lets the arm go
        # limp 0.2 s after the last one it was sent.
        self._holding = False
        self._send_lock = threading.Lock()
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="A750Connection-read"
        )
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None:
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        sdk, self._sdk = self._sdk, None
        if sdk is not None:
            # Also stops the SDK's control loop.
            sdk.disconnect()

    def connect(self) -> None:
        path = self.config.device_path
        sdk = _open_sdk(path)
        try:
            sdk.connect()
            sdk.start_control_loop()
            state = sdk.get_current_state()
        except Exception as error:
            with contextlib.suppress(Exception):
                sdk.disconnect()
            raise ConnectionError(f"no A-750 answers at {path}: {error}") from error
        self._sdk = sdk
        with self._send_lock:
            self._targets = _positions(state)
            self._finger_target = float(state.gripper.pos_m)
            self._holding = True

    def write(self, values: Values) -> None:
        targets = [values.get(f"{joint}/{Interface.POSITION}") for joint in self._joints]
        with self._send_lock:
            self._targets = [
                old if new is None else new for new, old in zip(targets, self._targets, strict=True)
            ]
            self._finger_target = values.get(
                f"{self._finger}/{Interface.POSITION}", self._finger_target
            )
            self._holding = False
            self._send()

    def halt(self) -> None:
        # The gripper keeps its last target, so it does not let go of what it holds.
        with self._send_lock:
            self._targets = _positions(self._sdk.get_current_state())
            self._holding = True
            self._send()

    def _send(self) -> None:
        """Send the joints' and the finger's targets together: after a pause the
        SDK forgets the joints' target when only the finger's is sent, and the
        other way round."""
        self._sdk.command_joint_positions(self._targets, self.config.velocity_ratio)
        self._sdk.command_gripper_position(self._finger_target)

    def _read_loop(self) -> None:
        period = 1.0 / _READ_HZ
        due = time.monotonic()
        while not self._stop_reading.wait(max(0.0, due - time.monotonic())):
            due = max(due + period, time.monotonic())
            try:
                state = self._sdk.get_current_state()
                joints = [getattr(state, f"joint{i}") for i in range(1, A750_DOF + 1)]
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        name=[*self._joints, self._finger],
                        position=[*(j.pos_rad for j in joints), state.gripper.pos_m],
                        velocity=[*(j.vel_rads for j in joints), state.gripper.vel_ms],
                        effort=[*(j.torque_nm for j in joints), state.gripper.force_n],
                    )
                )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
            with self._send_lock:
                if self._holding:
                    try:
                        self._send()
                    except Exception as error:
                        self._report(f"holding failed: {type(error).__name__}: {error}")


def _open_sdk(device_path: str) -> Any:
    """The SDK's handle on the arm at ``device_path``, not yet connected."""
    # Imported here so blueprints that use this module load without the SDK installed.
    import a750_control

    return a750_control.Robot(device_path)


def _positions(state: Any) -> list[float]:
    """The arm joints' positions (rad) in an SDK state."""
    return [float(getattr(state, f"joint{i}").pos_rad) for i in range(1, A750_DOF + 1)]
