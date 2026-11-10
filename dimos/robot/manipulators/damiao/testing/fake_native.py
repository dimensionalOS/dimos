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

"""The native CAN boundary a no-hardware Damiao test sees.

The unit fixtures import these classes directly. The worker acceptance run also
loads this module through :mod:`sitecustomize` from this directory, which the
test driver puts on ``PYTHONPATH`` *before* it builds any worker. Every process
that imports :mod:`can_motor_control` after that gets these in-memory classes
instead of the vendor ones, and the vendor transport constructors are sealed so
they raise instead of opening a bus. Nothing here is imported by production
code.

Events recorded by the fake boundary are appended as JSON lines to the path in
``DIMOS_FAKE_DAMIAO_LOG`` when that variable is set. A read is logged from
``FakeRobot.refresh`` and a write from ``FakeArm.mit_control``, each with its
process, thread and read epoch, so a test can prove the read and the write
happened in one cycle and one thread.
"""

from __future__ import annotations

import json
import os
import threading
import time
from types import SimpleNamespace
from typing import Any, ClassVar

import numpy as np

LOG_ENV = "DIMOS_FAKE_DAMIAO_LOG"
FAIL_ENV = "DIMOS_FAKE_DAMIAO_FAIL"
#: What a triggered feedback fault raises, so tests can assert the latch.
FAILURE_MESSAGE = "fake Damiao feedback failure requested by the test"

#: Read epoch, bumped by every ``refresh``; a write records the epoch it saw.
_epoch = 0


def _log(event: str, **fields: Any) -> None:
    path = os.environ.get(LOG_ENV)
    if not path:
        return
    record = {
        "event": event,
        "pid": os.getpid(),
        "tid": threading.get_ident(),
        "thread": threading.current_thread().name,
        "time": time.monotonic(),
        "epoch": _epoch,
        **fields,
    }
    data = (json.dumps(record, sort_keys=True) + "\n").encode()
    # One O_APPEND write per record keeps lines whole across worker processes.
    handle = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        os.write(handle, data)
    finally:
        os.close(handle)


class FakeArm:
    """One bus's arm motors. A worker write also moves the reported position."""

    #: The hosted worker boundary follows commands; unit fixtures keep the
    #: reported state fixed unless a test sets it.
    follows_commands: ClassVar[bool] = False

    def __init__(self, count: int, initial: float = 0.2) -> None:
        self.q = np.full(count, initial, dtype=np.float64)
        self.dq = np.full(count, 0.3, dtype=np.float64)
        self.tau = np.full(count, 0.4, dtype=np.float64)
        self.commands: list[np.ndarray] = []
        self.modes: list[str] = []
        self.error: Exception | None = None

    def positions(self) -> np.ndarray:
        return self.q.copy()

    def velocities(self) -> np.ndarray:
        return self.dq.copy()

    def torques(self) -> np.ndarray:
        return self.tau.copy()

    def mit_control(self, rows: np.ndarray) -> None:
        if self.error:
            raise self.error
        rows = np.asarray(rows, dtype=np.float64)
        self.commands.append(rows.copy())
        if self.follows_commands:
            self.q = rows[:, 2].copy()
        _log("write", q=self.q.tolist())

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)


class FakeGripper:
    """One jaw, tracked by its normalised opening."""

    follows_mit: ClassVar[bool] = False

    def __init__(self) -> None:
        self.opening = 0.4
        self.motor = SimpleNamespace(position=0.4)
        self.commands: list[float] = []
        self.modes: list[str] = []
        self.mit_commands: list[tuple[float, float, float, float, float]] = []
        self.enable_count = 0
        self.disable_count = 0

    def set_opening(self, opening: float) -> None:
        self.commands.append(opening)
        self.opening = opening

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)

    def enable(self) -> None:
        self.enable_count += 1

    def disable(self) -> None:
        self.disable_count += 1

    def mit_control(self, kp: float, kd: float, q: float, dq: float, tau: float) -> None:
        self.mit_commands.append((kp, kd, q, dq, tau))
        if self.follows_mit:
            self.opening = q


class FakeRobot:
    """The in-memory robot the vendor ``Robot`` would be."""

    #: Zero is inside every joint range of the shipped Damiao models, so the
    #: worker boundary starts there; unit fixtures use the richer 0.2 pose.
    default_position: ClassVar[float] = 0.2
    arm_type: ClassVar[type[FakeArm]] = FakeArm
    gripper_type: ClassVar[type[FakeGripper]] = FakeGripper

    def __init__(self) -> None:
        self.groups: dict[str, FakeArm | FakeGripper] = {}
        self.events: list[str] = []
        self.connected = False
        self.enable_error: Exception | None = None
        self.disable_error: Exception | None = None
        self.refresh_error: Exception | None = None
        self.tick_error: Exception | None = None
        self.bus_specs: list[tuple[Any, ...]] = []
        self.arm_specs: list[tuple[Any, ...]] = []
        self.gripper_specs: list[tuple[Any, ...]] = []

    def __getitem__(self, name: str) -> FakeArm | FakeGripper:
        return self.groups[name]

    def connect(self) -> None:
        self.events.append("connect")
        self.connected = True
        _log("connect")

    def enable(self) -> None:
        self.events.append("enable")
        _log("enable")
        if self.enable_error:
            raise self.enable_error

    def disable(self) -> None:
        self.events.append("disable")
        _log("disable")
        if self.disable_error:
            raise self.disable_error

    def refresh(self) -> None:
        global _epoch
        _epoch += 1
        self.events.append("refresh")
        if self.refresh_error:
            raise self.refresh_error
        trigger = os.environ.get(FAIL_ENV)
        if trigger and os.path.exists(trigger):
            raise RuntimeError(FAILURE_MESSAGE)
        _log("read")

    def tick(self, deadline: int) -> None:
        assert deadline > 0
        self.events.append("tick")
        if self.tick_error:
            raise self.tick_error


class Builder:
    """Build a fake robot, or attach groups to one a fixture already owns."""

    def __init__(self, robot: FakeRobot | None = None) -> None:
        self.robot = robot if robot is not None else FakeRobot()

    def add_bus(self, name: str, bus: Any, codec: Any) -> Builder:
        self.robot.bus_specs.append((name, bus, codec))
        return self

    def add_arm(self, name: str, *, bus: str, motors: list[Any]) -> Builder:
        self.robot.arm_specs.append((name, bus, motors))
        self.robot.groups[name] = self.robot.arm_type(len(motors), self.robot.default_position)
        return self

    def add_gripper(self, name: str, **kwargs: Any) -> Builder:
        self.robot.gripper_specs.append((name, kwargs))
        self.robot.groups[name] = self.robot.gripper_type()
        return self

    def build(self) -> FakeRobot:
        return self.robot


class _WorkerArm(FakeArm):
    follows_commands = True


class _WorkerGripper(FakeGripper):
    follows_mit = True


class _WorkerRobot(FakeRobot):
    default_position = 0.0
    arm_type = _WorkerArm
    gripper_type = _WorkerGripper


class FakeMotorSpec:
    def __init__(self, *args: Any) -> None:
        self.args = args


class FakeBus:
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        pass


#: The vendor constructors, kept so a test can prove they are sealed.
_originals: dict[str, Any] = {}


def _seal(original: Any, name: str) -> None:
    """Make an unpatched real transport constructor fail instead of opening a bus."""

    def blocked(*args: Any, **kwargs: Any) -> Any:
        raise RuntimeError(f"physical CAN transport {name} is disabled in tests")

    # The vendor boundary is a native (__new__-based) extension: sealing
    # __init__ alone is ignored, so try both.
    for attribute in ("__new__", "__init__"):
        try:
            setattr(original, attribute, blocked)
        except (AttributeError, TypeError):
            pass


def install() -> None:
    """Replace the vendor boundary with in-memory classes, once per process."""
    import can_motor_control

    if getattr(can_motor_control, "_dimos_fake_native", False):
        return
    for name in ("Robot", "SocketCanBus", "GsUsbBus", "DmSerialBus"):
        original = getattr(can_motor_control, name, None)
        if original is not None:
            _originals[name] = original
            _seal(original, name)
    can_motor_control.Robot = _WorkerRobot
    can_motor_control.Robot.builder = staticmethod(lambda: Builder(_WorkerRobot()))  # type: ignore[attr-defined]
    can_motor_control.Arm = FakeArm
    can_motor_control.Gripper = FakeGripper
    can_motor_control.MotorSpec = FakeMotorSpec
    can_motor_control.SocketCanBus = FakeBus
    can_motor_control.GsUsbBus = FakeBus
    can_motor_control.DmSerialBus = FakeBus
    can_motor_control._dimos_fake_native = True


def installed() -> bool:
    """Whether this process is running against the fake native boundary."""
    import can_motor_control

    return bool(getattr(can_motor_control, "_dimos_fake_native", False))


def sealed_boundary_is_fail_closed() -> bool:
    """Whether constructing any unpatched vendor transport raises instead."""
    if not _originals:
        return False
    for original in _originals.values():
        try:
            original("can0")  # type: ignore[call-arg]
        except RuntimeError:
            continue
        except Exception:
            return False
        return False
    return True
