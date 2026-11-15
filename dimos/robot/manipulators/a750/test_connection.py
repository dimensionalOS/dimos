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

"""Tests for ``A750Connection`` against a fake SDK, with no arm attached.

The fake stands in for ``a750_control.Robot`` and records what the module
sends. Every port gets an in-process transport, and the robot model is a
small URDF written here, so nothing opens a serial port.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.a750 import connection
from dimos.robot.manipulators.a750.connection import A750Connection
from dimos.utils.testing.waiting import wait_until

# Long enough that the deadman never halts mid-test, except where a test sets its own.
DEADMAN_S = 5.0


class FakeA750:
    """An A-750 at rest, at the end of ``a750_control.Robot``.

    Every call is kept in ``calls``. ``get_current_state`` answers from
    ``positions`` (rad), ``speeds`` (rad/s), ``torques`` (N·m) and the
    gripper's ``finger`` (m), ``finger_speed`` (m/s) and ``finger_force`` (N).
    """

    def __init__(self) -> None:
        self.positions = [0.0] * 6
        self.speeds = [0.0] * 6
        self.torques = [0.0] * 6
        self.finger, self.finger_speed, self.finger_force = 0.03, 0.0, 0.0
        self.calls: list[tuple[str, tuple[Any, ...]]] = []

    def connect(self) -> None:
        self.calls.append(("connect", ()))

    def disconnect(self) -> None:
        self.calls.append(("disconnect", ()))

    def start_control_loop(self) -> None:
        self.calls.append(("start_control_loop", ()))

    def command_joint_positions(self, positions: list[float], velocity_ratio: float) -> None:
        self.calls.append(("command_joint_positions", (list(positions), velocity_ratio)))

    def command_gripper_position(self, position: float) -> None:
        self.calls.append(("command_gripper_position", (position,)))

    def get_current_state(self) -> SimpleNamespace:
        joints = {
            f"joint{i}": SimpleNamespace(pos_rad=p, vel_rads=v, torque_nm=t)
            for i, (p, v, t) in enumerate(
                zip(self.positions, self.speeds, self.torques, strict=True), 1
            )
        }
        gripper = SimpleNamespace(
            pos_m=self.finger, vel_ms=self.finger_speed, force_n=self.finger_force
        )
        return SimpleNamespace(gripper=gripper, **joints)

    def sent(self, name: str) -> list[tuple[Any, ...]]:
        """The arguments of each call to ``name`` so far, oldest first."""
        return [args for each, args in self.calls if each == name]


class LocalTransport:
    """An in-process transport: publishing calls every subscriber at once."""

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


def a750_like_model(tmp_path: Path) -> RobotModel:
    """A stand-in for the A-750's URDF: joint1 to joint6, each within 2 rad
    of 0, and a finger from 0 to 0.06 m."""
    limit = '<limit lower="-2" upper="2" velocity="3" effort="100"/>'
    joints = [
        f'<joint name="joint{i}" type="revolute"><parent link="link{i - 1}"/>'
        f'<child link="link{i}"/>{limit}</joint>'
        for i in range(1, 7)
    ]
    joints.append(
        '<joint name="finger" type="prismatic"><parent link="link6"/><child link="finger"/>'
        '<limit lower="0" upper="0.06" velocity="1" effort="50"/></joint>'
    )
    links = [f'<link name="link{i}"/>' for i in range(7)] + ['<link name="finger"/>']
    path = tmp_path / "a750.urdf"
    path.write_text(f'<robot name="a750">{"".join(links)}{"".join(joints)}</robot>')
    return RobotModel.from_file(path)


class Rig:
    """One ``A750Connection`` on a fake SDK, with an in-process transport on every port."""

    def __init__(self, module: A750Connection, sdk: FakeA750) -> None:
        self.module = module
        self.sdk = sdk
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def move(self, **positions: float) -> None:
        """Send targets in rad (m for the finger) on ``position_command``, e.g. ``move(joint1=0.2)``."""
        names = [f"arm/{joint}" for joint in positions]
        self.ports["position_command"].publish(
            JointState(name=names, position=list(positions.values()))
        )

    def readings(self) -> list[JointState]:
        return list(self.ports["joint_state"].sent)


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build an ``A750Connection`` on a fake SDK, start it, and stop every one
    at the end of the test."""
    made: list[Rig] = []

    def build(**config: Any) -> Rig:
        fake = FakeA750()
        monkeypatch.setattr(connection, "_open_sdk", lambda path: fake)
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        module = A750Connection(
            device_path="/dev/fake-a750", model=a750_like_model(tmp_path), **config
        )
        made.append(Rig(module, fake))
        module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def test_start_connects_and_runs_the_control_loop_and_stop_disconnects(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    assert [name for name, _ in r.sdk.calls[:2]] == ["connect", "start_control_loop"]
    r.module.stop()
    assert r.sdk.calls[-1] == ("disconnect", ())


def test_joint_targets_and_the_finger_go_out_together(rig: Callable[..., Rig]) -> None:
    r = rig(velocity_ratio=0.5)
    r.move(joint1=0.2, finger=0.05)
    r.move(joint6=-0.1)
    # A joint left out keeps its last target, and the finger is sent every time.
    # (Before the first command, the arm was held where it started.)
    assert r.sdk.sent("command_joint_positions")[-2:] == [
        ([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
        ([0.2, 0.0, 0.0, 0.0, 0.0, -0.1], 0.5),
    ]
    assert r.sdk.sent("command_gripper_position")[-2:] == [(0.05,), (0.05,)]


def test_joint_state_names_each_joint_and_the_finger(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.sdk.positions = [1.5, 0, 0, 0, 0, 0]
    r.sdk.speeds = [0.5, 0, 0, 0, 0, 0]
    r.sdk.torques = [2.0, 0, 0, 0, 0, 0]
    r.sdk.finger, r.sdk.finger_force = 0.04, 10.0

    wait_until(lambda: any(m.position[0] == 1.5 for m in r.readings()), timeout=2.0)
    msg = next(m for m in r.readings() if m.position[0] == 1.5)
    assert msg.name == [*(f"arm/joint{i}" for i in range(1, 7)), "arm/finger"]
    assert msg.position == pytest.approx([1.5, 0, 0, 0, 0, 0, 0.04])
    assert msg.velocity == pytest.approx([0.5, 0, 0, 0, 0, 0, 0])
    assert msg.effort == pytest.approx([2.0, 0, 0, 0, 0, 0, 10.0])


def test_no_command_within_the_deadman_holds_the_arm_where_it_is(
    rig: Callable[..., Rig],
) -> None:
    r = rig(deadman_timeout_s=0.05)
    r.move(joint1=0.5, finger=0.02)
    r.sdk.positions = [0.3, 0, 0, 0, 0, 0]
    wait_until(lambda: r.module.status().deadman_fired, timeout=2.0)
    # The joints are held where they were read; the finger keeps squeezing.
    assert r.sdk.sent("command_joint_positions")[-1] == ([0.3, 0, 0, 0, 0, 0], 1.0)
    assert r.sdk.sent("command_gripper_position")[-1] == (0.02,)
    # The hold is sent again and again, so the SDK never lets the arm go limp.
    held = len(r.sdk.sent("command_joint_positions"))
    wait_until(lambda: len(r.sdk.sent("command_joint_positions")) > held + 2, timeout=2.0)
