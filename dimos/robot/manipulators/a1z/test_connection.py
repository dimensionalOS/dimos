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

"""Tests for ``A1ZConnection`` against a fake SDK, with no arm attached.

The fake stands in for the a1z SDK's ``ArmRobot`` where the module opens it
and records what the module does to it. Every port gets an in-process
transport, and the robot model is a small URDF written here, so nothing
touches a CAN bus.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.a1z import connection
from dimos.robot.manipulators.a1z.connection import A1ZConnection, A1ZError
from dimos.utils.testing.waiting import wait_until

PORT = "can9"
# Long enough that the deadman never halts mid-test.
DEADMAN_S = 5.0
KP = [80.0, 80.0, 80.0, 50.0, 20.0, 20.0]
KD = [3.0, 3.0, 3.0, 0.7, 0.4, 0.4]


class FakeMotor:
    def __init__(self, name: str, calls: list[tuple[Any, ...]]) -> None:
        self.name = name
        self.last_feedback: object | None = object()
        self._calls = calls

    def disable(self) -> None:
        self._calls.append(("disable", self.name))


class FakeArm:
    """An A1Z at rest, at the end of ``get_a1z_robot``, motors off.

    Every call is kept in ``calls``. ``state`` is what ``get_joint_state``
    returns: positions (rad), speeds (rad/s), torques (N·m) and motor codes.
    ``move_joints`` puts the arm at its target at once; ``gripper_open`` is
    the gripper's reported opening, 0 to 1.
    """

    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []
        self.running = False
        self.estopped = False
        self._gravity = 1.0
        self.state: dict[str, Any] = {
            "pos": np.zeros(6),
            "vel": np.zeros(6),
            "eff": np.zeros(6),
            "error_codes": np.ones(6, dtype=int),
        }
        self.gripper_open = 0.3
        arm = [FakeMotor(f"joint{i}", self.calls) for i in range(1, 7)]
        self._motor_chain = SimpleNamespace(_motor_a_list=arm[:3], _motor_b_list=arm[3:])
        self.gripper = SimpleNamespace(
            _motor=FakeMotor("gripper", self.calls), get_feedback_norm=lambda: self.gripper_open
        )
        self._bus = SimpleNamespace(shutdown=lambda: self.calls.append(("shutdown",)))

    @property
    def is_running(self) -> bool:
        return self.running

    @property
    def is_estopped(self) -> bool:
        return self.estopped

    @property
    def gravity_comp_factor(self) -> float:
        return self._gravity

    @gravity_comp_factor.setter
    def gravity_comp_factor(self, value: float) -> None:
        self._gravity = value
        self.calls.append(("gravity", value))

    def start(self, initial_kp: Any = None, initial_kd: Any = None) -> None:
        self.running, self.estopped = True, False
        self.calls.append(("start", list(initial_kp), list(initial_kd)))

    def stop(self) -> None:
        self.running = False
        self.calls.append(("stop",))

    def estop(self) -> None:
        self.estopped = True
        self.calls.append(("estop",))

    def release(self) -> None:
        self.estopped = False
        self.calls.append(("release",))

    def get_robot_info(self) -> dict[str, Any]:
        return {"default_kp": np.asarray(KP), "default_kd": np.asarray(KD)}

    def get_joint_state(self) -> dict[str, Any]:
        return {key: value.copy() for key, value in self.state.items()}

    def command_joint_state(self, state: dict[str, Any]) -> None:
        self.calls.append(("hold", list(state["pos"]), list(state["kp"]), self._gravity))

    def command_joint_pos(self, positions: Any) -> None:
        self.calls.append(("command_joint_pos", [float(p) for p in positions]))

    def move_joints(self, positions: Any, speed: float = 0.5) -> None:
        self.calls.append(("move_joints", [float(p) for p in positions], speed))
        self.state["pos"] = np.asarray(positions, dtype=float)

    def command_gripper(self, position: float) -> None:
        self.calls.append(("command_gripper", position))

    def sent(self, name: str) -> list[tuple[Any, ...]]:
        """The arguments of each call to ``name`` so far, oldest first."""
        return [call[1:] for call in self.calls if call[0] == name]


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


def a1z_like_model(tmp_path: Path) -> RobotModel:
    """A stand-in for the A1Z's model: arm_joint1 to arm_joint6, each within
    2 rad of 0 at up to 3 rad/s, and a gripper opening from 0 to 0.1 m."""
    limit = '<limit lower="-2" upper="2" velocity="3" effort="20"/>'
    joints = [
        f'<joint name="arm_joint{i}" type="revolute"><parent link="link{i - 1}"/>'
        f'<child link="link{i}"/>{limit}</joint>'
        for i in range(1, 7)
    ]
    joints.append(
        '<joint name="gripper" type="prismatic"><parent link="link6"/><child link="jaw"/>'
        '<limit lower="0" upper="0.1" velocity="0" effort="0"/></joint>'
    )
    links = ['<link name="jaw"/>', *(f'<link name="link{i}"/>' for i in range(7))]
    path = tmp_path / "a1z.urdf"
    path.write_text(f'<robot name="a1z">{"".join(links)}{"".join(joints)}</robot>')
    return RobotModel.from_file(path)


class Rig:
    """One ``A1ZConnection`` on a fake SDK, with an in-process transport on every port."""

    def __init__(self, module: A1ZConnection, arm: FakeArm) -> None:
        self.module = module
        self.arm = arm
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def move(self, **positions: float) -> None:
        """Send targets in rad (m for the gripper) on ``position_command``, e.g. ``move(arm_joint1=0.2)``."""
        names = [f"arm/{joint}" for joint in positions]
        self.ports["position_command"].publish(
            JointState(name=names, position=list(positions.values()))
        )

    def readings(self) -> list[JointState]:
        return list(self.ports["joint_state"].sent)

    def last_error(self) -> str:
        return str(self.module.status().last_error)


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build an ``A1ZConnection`` with its gripper on a fake SDK, start it,
    and stop every one at the end of the test."""
    # One gravity ramp step, and quick samples.
    monkeypatch.setattr(connection, "_STARTUP_SAMPLE_PERIOD_S", 0.001)
    monkeypatch.setattr(connection, "_STARTUP_RAMP_DURATION_S", 0.0)
    made: list[Rig] = []

    def build(arm: FakeArm | None = None, **config: Any) -> Rig:
        fake = arm or FakeArm()
        monkeypatch.setattr(connection, "_open_sdk", lambda port, **_: fake)
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        module = A1ZConnection(can_port=PORT, model=a1z_like_model(tmp_path), **config)
        made.append(Rig(module, fake))
        module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def test_start_holds_the_arm_where_it_is_before_gravity_comes_in(rig: Callable[..., Rig]) -> None:
    arm = FakeArm()
    arm.state["pos"] = np.array([0.1, 0.5, -0.5, 0.0, 0.0, 0.0])
    r = rig(arm, home=None)
    # Gravity off, the loop started with no stiffness and half damping, every
    # motor heard, then held where it is at full stiffness with gravity still off.
    assert arm.calls[:3] == [
        ("gravity", 0.0),
        ("start", [0.0] * 6, [k / 2 for k in KD]),
        ("hold", [0.1, 0.5, -0.5, 0.0, 0.0, 0.0], KP, 0.0),
    ]
    # Gravity stays off a few samples, then ramps in to the full factor.
    assert [args[0] for args in arm.sent("gravity")] == [0.0] * 6 + [1.0]
    assert arm.sent("move_joints") == []
    assert r.module.status().last_error is None


def test_joint_targets_go_out_in_radians_within_the_jump_guard(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(arm_joint1=0.2, arm_joint6=-0.1)
    # A joint left out stays where it is.
    assert r.arm.sent("command_joint_pos")[-1] == ([0.2, 0.0, 0.0, 0.0, 0.0, -0.1],)
    # 3 rad/s for 0.1 s allows 0.3 rad from where joint 3 is (0).
    sent = len(r.arm.sent("command_joint_pos"))
    r.move(arm_joint3=0.4)
    assert "arm/arm_joint3 told 0.400 rad" in r.last_error()
    # The refused target halts: the arm is held where it is.
    assert r.arm.sent("command_joint_pos")[sent:] == [([0.0] * 6,)]


def test_joint_state_names_each_joint_with_its_position_speed_and_torque(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.arm.state["pos"] = np.array([1.5, 0.5, -0.5, 0.0, 0.0, 0.0])
    r.arm.state["vel"] = np.array([0.25, 0.0, 0.0, 0.0, 0.0, -0.5])
    r.arm.state["eff"] = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def moved() -> list[JointState]:
        return [m for m in r.readings() if len(m.name) == 6 and m.position[0] > 1.0]

    wait_until(lambda: bool(moved()), timeout=2.0)
    msg = moved()[0]
    assert msg.name == [f"arm/arm_joint{i}" for i in range(1, 7)]
    assert msg.position == pytest.approx([1.5, 0.5, -0.5, 0, 0, 0])
    assert msg.velocity == pytest.approx([0.25, 0, 0, 0, 0, -0.5])
    assert msg.effort == pytest.approx([2.0, 0, 0, 0, 0, 0])


def test_the_gripper_goes_out_as_a_fraction_and_comes_in_in_metres(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(gripper=0.05)
    r.move(gripper=0.05)
    r.move(gripper=0.1)
    # Sent only when it changes; the gripper alone moves no arm joint.
    assert r.arm.sent("command_gripper") == [(0.5,), (1.0,)]
    assert r.arm.sent("command_joint_pos") == []
    wait_until(lambda: any(m.name == ["arm/gripper"] for m in r.readings()), timeout=2.0)
    gripper = next(m for m in r.readings() if m.name == ["arm/gripper"])
    assert gripper.position == pytest.approx([0.03])


def test_the_deadman_holds_the_arm_where_it_is(rig: Callable[..., Rig]) -> None:
    r = rig(deadman_timeout_s=0.05)
    r.move(arm_joint1=0.1)
    wait_until(lambda: r.module.status().deadman_fired, timeout=2.0)
    assert r.arm.sent("command_joint_pos") == [([0.1, 0.0, 0.0, 0.0, 0.0, 0.0],), ([0.0] * 6,)]


def test_an_arm_the_sdk_stopped_takes_no_command_until_its_errors_are_cleared(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.arm.estopped = True
    wait_until(lambda: "soft e-stop" in r.last_error(), timeout=2.0)
    r.move(arm_joint1=0.1)
    assert r.arm.sent("command_joint_pos") == []

    r.module.clear_errors()
    # Released, and held where it is.
    assert r.arm.calls[-2:] == [("release",), ("command_joint_pos", [0.0] * 6)]
    r.move(arm_joint1=0.1)
    assert r.arm.sent("command_joint_pos")[-1] == ([0.1, 0.0, 0.0, 0.0, 0.0, 0.0],)


def test_a_failed_start_takes_gravity_away_then_turns_the_motors_off(
    rig: Callable[..., Rig],
) -> None:
    arm = FakeArm()
    # Joint 3 is moving: the start refuses it.
    arm.state["vel"][2] = 0.75
    with pytest.raises(A1ZError, match="arm_joint3 moves at 0.750 rad/s"):
        rig(arm)
    after = arm.calls[arm.calls.index(("estop",)) - 1 :]
    disables = [("disable", f"joint{i}") for i in range(1, 7)] + [("disable", "gripper")]
    assert after == [("gravity", 0.0), ("estop",), ("stop",), *disables, *disables, ("shutdown",)]


def test_with_a_home_the_arm_goes_there_at_start_and_before_its_motors_turn_off(
    rig: Callable[..., Rig],
) -> None:
    home = (0.0, 0.5, -0.5, 0.0, 0.0, 0.0)
    r = rig(home=home)
    assert r.arm.sent("move_joints") == [(list(home), 0.5)]
    r.arm.state["pos"] = np.array([0.1, 0.4, -0.4, 0.0, 0.0, 0.0])
    r.module.stop()
    assert r.arm.sent("move_joints")[-1] == (list(home), 1.0)
    moved_home = r.arm.calls.index(("move_joints", list(home), 1.0))
    assert ("stop",) in r.arm.calls[moved_home:]
    assert r.arm.calls[-1] == ("shutdown",)


def test_a_home_outside_the_limits_is_refused_before_the_arm_is_touched(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    opened: list[str] = []
    monkeypatch.setattr(connection, "_open_sdk", lambda port, **_: opened.append(port))
    module = A1ZConnection(
        can_port=PORT, home=(10.0, 0.0, 0.0, 0.0, 0.0, 0.0), model=a1z_like_model(tmp_path)
    )
    Rig(module, FakeArm())
    try:
        with pytest.raises(ValueError, match="arm_joint1"):
            module.start()
        assert opened == []
    finally:
        module.stop()
