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

"""Stand-ins for the tests: a clock that only moves when told, a module whose
ports record everything, a driver that records what it is sent, and three
small robots built with the presets.

Nothing here uses the network or real time.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import replace
from typing import Any

import pytest

from dimos.control.connection.connected_hardware import ConnectedHardware, Frame
from dimos.control.contract.description import (
    ControlDescription,
    Estop,
    EstopKind,
    EstopRecovery,
    Limits,
    Omission,
)
from dimos.control.contract.keys import EFFORT, POSITION, VELOCITY, VX, VY, WZ, Key, Unit
from dimos.control.contract.presets import (
    GripperSpec,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.msgs.control_msgs.ControlValues import ControlValues

ARM = ("j1", "j2")


class FakeClock:
    """Seconds that only pass when a test says so."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class RecordingOut:
    """An output port that keeps everything published on it."""

    def __init__(self) -> None:
        self.published: list[ControlValues] = []

    def publish(self, msg: ControlValues) -> None:
        self.published.append(msg)


class SendingIn:
    """An input port a test can push messages into."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[ControlValues], Any]] = []

    def subscribe(self, cb: Callable[[ControlValues], Any]) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def send(self, msg: ControlValues) -> None:
        for cb in list(self.subscribers):
            cb(msg)


class FakeModule:
    """Just the two ports a driver module has."""

    def __init__(self) -> None:
        self.control_state = RecordingOut()
        self.control_command = SendingIn()


class FakeHooks:
    """A driver that does what it is told and remembers it.

    Only the required hooks exist. A test adds an optional one by setting it,
    e.g. ``hooks.set_native = calls.append``.
    """

    def __init__(self, desc: ControlDescription) -> None:
        self.desc = desc
        self.written: list[Frame] = []
        self.connected = False
        self.shut_down = False
        self.sample: tuple[dict[str, float], float] | None = None
        self.fail_writes = 0

    def connect(self) -> None:
        self.connected = True

    def describe(self) -> ControlDescription:
        return self.desc

    def write(self, frame: Frame) -> None:
        if self.fail_writes:
            self.fail_writes -= 1
            raise RuntimeError("hardware said no")
        self.written.append(frame)

    def shutdown(self) -> None:
        self.shut_down = True

    def read_state(self) -> tuple[dict[str, float], float] | None:
        return self.sample


class LocalTransport:
    """An in-process transport, for giving a real module's ports somewhere to go."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)
        for cb in list(self.subscribers):
            cb(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, cb: Callable[[Any], Any], selfstream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def arm_description(**overrides: Any) -> ControlDescription:
    """A 2-joint arm plus gripper, driven by position or by speed.

    Reports position and effort but not speed. Stops by holding.
    """
    desc = manipulator_description(
        "arm",
        ARM,
        limits={Key.of("arm", j, POSITION): Limits(-3.0, 3.0) for j in ARM},
        state=(POSITION, EFFORT),
        groups=(frozenset({POSITION}), frozenset({VELOCITY})),
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085),
    )
    return replace(desc, **overrides)


def pd_description(**overrides: Any) -> ControlDescription:
    """A 2-joint body held by stiffness and damping. Its speed targets are
    left unset when not sent. Stops by damping; emergency-stops by damping."""
    desc = pd_joint_description(
        "g1",
        ARM,
        limits={},
        kp=60.0,
        kd=1.5,
        damp_kd=5.0,
        omission={Key.of("g1", j, VELOCITY): Omission.UNSET for j in ARM},
        estop=Estop(kind=EstopKind.DAMP, recovery=EstopRecovery.CLEAR),
    )
    return replace(desc, **overrides)


def base_description(**overrides: Any) -> ControlDescription:
    """A base that drives in any direction. Stops by zeroing its speeds."""
    desc = twist_base_description(
        "base",
        limits={Key.of("base", "base", axis): Limits(-2.0, 2.0) for axis in (VX, VY, WZ)},
    )
    return replace(desc, **overrides)


def fast_hooks(desc: ControlDescription, seconds: float = 0.05) -> ControlDescription:
    """The same description with a short limit on how long a hook may take,
    so a test of a hook that hangs does not wait long."""
    return replace(desc, timing=replace(desc.timing, hook_timeout_s=seconds))


def arm_position_command(
    epoch: int, sequence: int, positions: tuple[float, float] = (0.1, 0.2), gripper: float = 0.04
) -> ControlValues:
    """A complete position command for ``arm_description``."""
    names = [f"arm/{j}/position" for j in ARM] + ["arm/gripper/position"]
    return ControlValues(
        source="coordinator",
        epoch=epoch,
        sequence=sequence,
        interface_names=names,
        values=[*positions, gripper],
    )


def arm_velocity_command(
    epoch: int, sequence: int, speeds: tuple[float, float] = (0.5, -0.5), gripper: float = 0.04
) -> ControlValues:
    """A complete speed command for ``arm_description``."""
    names = [f"arm/{j}/velocity" for j in ARM] + ["arm/gripper/position"]
    return ControlValues(
        source="coordinator",
        epoch=epoch,
        sequence=sequence,
        interface_names=names,
        values=[*speeds, gripper],
    )


def arm_reading(positions: tuple[float, float] = (0.0, 0.0)) -> dict[str, float]:
    """A complete set of readings for ``arm_description``."""
    values = {f"arm/{j}/position": p for j, p in zip(ARM, positions, strict=True)}
    values |= {f"arm/{j}/effort": 0.0 for j in ARM}
    values["arm/gripper/position"] = 0.0
    return values


def full_reading(desc: ControlDescription, value: float = 0.0) -> dict[str, float]:
    """A complete set of readings for any description, every value the same."""
    return dict.fromkeys(desc.state_keys(), value)


class Rig:
    """One started ``ConnectedHardware`` with everything around it faked."""

    def __init__(self, desc: ControlDescription, state_mode: str = "push", **hooks: Any) -> None:
        self.clock = FakeClock()
        self.module = FakeModule()
        self.hooks = FakeHooks(desc)
        for name, hook in hooks.items():
            setattr(self.hooks, name, hook)
        self.hw = ConnectedHardware(
            self.module,
            self.hooks,
            state_mode=state_mode,  # type: ignore[arg-type]
            clock=self.clock,
            description_epoch=7,
        )
        self.hw.start(background=False)
        # Well clear of the small numbers tests pick by hand.
        self.op = 1000

    def next_op(self) -> int:
        self.op += 1
        return self.op

    def feed(self, values: dict[str, float] | None = None) -> None:
        """Deliver one set of readings now."""
        desc = self.hw.describe_control()
        self.hw.ingest_state(values if values is not None else full_reading(desc), self.clock())

    def arm(self, epoch: int = 1) -> None:
        """Take it from STANDBY to ARMED."""
        self.feed()
        assert self.hw.prepare_arm(self.next_op()).ok
        assert self.hw.commit_arm(self.next_op(), epoch).ok

    def send(self, frame: ControlValues) -> None:
        self.module.control_command.send(frame)

    def rejections(self) -> dict[str, int]:
        return dict(self.hw.status().rejections)


@pytest.fixture
def rig_for() -> Iterator[Callable[..., Rig]]:
    """Build rigs; every one is stopped at the end of the test."""
    rigs: list[Rig] = []

    def build(desc: ControlDescription, state_mode: str = "push", **hooks: Any) -> Rig:
        rig = Rig(desc, state_mode, **hooks)
        rigs.append(rig)
        return rig

    yield build
    for rig in rigs:
        rig.hw.stop()
