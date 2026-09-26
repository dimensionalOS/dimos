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

"""A pretend robot driver, for running and testing without hardware.

It behaves like a real driver from the outside -- same ports, same calls, same
checks and stops -- but its "hardware" just does what it is told. Pick a shape:

  arm   a 7-joint arm with a gripper, driven by position or by speed
  pd    a 29-joint humanoid held by stiffness and damping, with a tilt sensor
  base  a base that drives in any direction on the floor

Each reading step moves it by one period of the state rate: a joint told a
position is there at once, a joint told a speed moves at that speed, and a
base moves as its speeds say. A joint given no stiffness is not pulled to its
target position at all; it only moves at its target speed, so a damping stop
leaves it wherever it is.
"""

from __future__ import annotations

import threading
import time
from typing import Literal

from dimos.control.connection.connected_hardware import ConnectedHardware, Frame
from dimos.control.connection.helpers import integrate_planar_twist
from dimos.control.connection.module import ConnectionRpcMixin
from dimos.control.contract.description import (
    ControlDescription,
    Estop,
    EstopKind,
    EstopRecovery,
    ProcessLoss,
    ResourceKind,
)
from dimos.control.contract.keys import (
    EFFORT,
    KP,
    POSITION,
    QW,
    VELOCITY,
    VX,
    VY,
    WZ,
    YAW,
    Unit,
    X,
    Y,
)
from dimos.control.contract.presets import (
    GripperSpec,
    ImuSpec,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig

MockShape = Literal["arm", "pd", "base"]

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
PD_JOINTS = tuple(f"joint{i}" for i in range(1, 30))


class MockConnectionConfig(ModuleConfig):
    """Settings for ``MockConnection``.

    Attributes:
        shape: Which kind of robot to pretend to be.
        source: What to call it; the first part of every name it uses.
        background: Run its reading and supervising loops on their own
            threads. False to step it by hand with ``hw.poll_state()`` and
            ``hw.supervise()``.
    """

    shape: MockShape = "arm"
    source: str = "mock"
    background: bool = True


def mock_description(shape: MockShape, source: str) -> ControlDescription:
    """The description ``MockConnection`` uses for a given shape.

    Args:
        shape: Which kind of robot.
        source: What to call it.
    """
    if shape == "arm":
        return manipulator_description(
            source,
            ARM_JOINTS,
            limits={},
            groups=(frozenset({POSITION}), frozenset({VELOCITY})),
            gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085),
            process_loss=ProcessLoss.SIMULATION,
        )
    if shape == "pd":
        return pd_joint_description(
            source,
            PD_JOINTS,
            limits={},
            kp=60.0,
            kd=1.5,
            damp_kd=5.0,
            imu=ImuSpec(frame_id=f"{source}_imu"),
            # The humanoid preset switches motors off in an emergency, which
            # only a real driver can do. The pretend one damps instead.
            estop=Estop(kind=EstopKind.DAMP, recovery=EstopRecovery.CLEAR),
            process_loss=ProcessLoss.SIMULATION,
        )
    return twist_base_description(source, limits={}, process_loss=ProcessLoss.SIMULATION)


class MockConnection(Module, ConnectionRpcMixin):
    """A pretend robot driver. See the module docstring."""

    config: MockConnectionConfig

    @rpc
    def start(self) -> None:
        super().start()
        self._lock = threading.Lock()
        self._desc = mock_description(self.config.shape, self.config.source)
        self._plant: dict[str, float] = {}
        self._last: Frame | None = None
        self._fault: str | None = None
        self._writes = 0
        self.hw = ConnectedHardware(self, hooks=self, state_mode="poll")
        self.register_hardware(self.hw)
        self.hw.start(background=self.config.background)

    @rpc
    def inject_fault(self, reason: str | None) -> None:
        """Make the pretend hardware report a fault, or ``None`` to stop."""
        with self._lock:
            self._fault = reason

    @rpc
    def set_measured(self, values: dict[str, float]) -> None:
        """Overwrite some readings, e.g. to start a joint somewhere else.

        Args:
            values: Readings by full name, e.g. ``{"mock/joint1/position": 0.5}``.
        """
        with self._lock:
            self._plant.update(values)

    @rpc
    def commands_received(self) -> int:
        """How many frames have been sent to the pretend hardware, including
        ones sent while stopping."""
        with self._lock:
            return self._writes

    def connect(self) -> None:
        with self._lock:
            self._plant = {key: 0.0 for key in self._desc.state_keys()}
            qw = f"{self._desc.source}/imu/{QW}"
            if qw in self._plant:
                self._plant[qw] = 1.0

    def describe(self) -> ControlDescription:
        return self._desc

    def write(self, frame: Frame) -> None:
        with self._lock:
            self._last = frame
            self._writes += 1

    def read_state(self) -> tuple[dict[str, float], float]:
        dt = 1.0 / self._desc.timing.state_rate_hz
        with self._lock:
            commanded = self._last.values if self._last else {}
            for resource in self._desc.resources:
                if resource.kind is ResourceKind.JOINT:
                    self._step_joint(resource.name, commanded, dt)
                elif resource.kind is ResourceKind.BASE:
                    self._step_base(resource.name, commanded, dt)
            return dict(self._plant), time.time()

    def fault(self) -> str | None:
        with self._lock:
            return self._fault

    def shutdown(self) -> None:
        pass

    def _step_joint(self, name: str, commanded: dict[str, float], dt: float) -> None:
        prefix = f"{self._desc.source}/{name}/"
        position = self._plant.get(prefix + POSITION, 0.0)
        # With no stiffness a position target pulls on nothing.
        stiff = commanded.get(prefix + KP) != 0.0
        if prefix + POSITION in commanded and stiff:
            target = commanded[prefix + POSITION]
            speed = (target - position) / dt
            position = target
        else:
            speed = commanded.get(prefix + VELOCITY, 0.0)
            position += speed * dt
        self._set(prefix + POSITION, position)
        self._set(prefix + VELOCITY, speed)
        self._set(prefix + EFFORT, commanded.get(prefix + EFFORT, 0.0))

    def _step_base(self, name: str, commanded: dict[str, float], dt: float) -> None:
        prefix = f"{self._desc.source}/{name}/"
        vx, vy, wz = (commanded.get(prefix + axis, 0.0) for axis in (VX, VY, WZ))
        x, y, yaw = integrate_planar_twist(
            self._plant.get(prefix + X, 0.0),
            self._plant.get(prefix + Y, 0.0),
            self._plant.get(prefix + YAW, 0.0),
            vx,
            vy,
            wz,
            dt,
        )
        for key, value in ((X, x), (Y, y), (YAW, yaw), (VX, vx), (VY, vy), (WZ, wz)):
            self._set(prefix + key, value)

    def _set(self, key: str, value: float) -> None:
        # Only readings the description promises are kept, so a shape that
        # does not report, say, speed never starts to.
        if key in self._plant:
            self._plant[key] = value
