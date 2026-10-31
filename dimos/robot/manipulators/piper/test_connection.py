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

"""Tests for ``PiperConnection`` against a fake SDK, with no arm attached.

The fake stands in for piper_sdk's ``C_PiperInterface_V2`` where the module
opens it and records what the module sends. Every port gets an in-process
transport, and the robot model is a small URDF written here, so nothing
touches a CAN bus.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.control.contract.limits import limits
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.piper import connection
from dimos.robot.manipulators.piper.connection import PiperConfig, PiperConnection, PiperError
from dimos.utils.testing.waiting import wait_until

PORT = "can9"
# Long enough that the deadman never halts mid-test.
DEADMAN_S = 5.0
# The fake's status codes, as the SDK reports them.
STANDBY, CAN_CONTROL, TEACHING = 0x00, 0x01, 0x02
COLLISION = 0x07

Call = tuple[str, tuple[Any, ...], dict[str, Any]]


class FakePiper:
    """A Piper at rest, powered up in standby, at the end of ``C_PiperInterface_V2``.

    Every command is kept in ``calls``. Reads answer from ``positions``
    (0.001 degrees), ``speeds`` (0.001 rad/s), ``efforts`` (0.001 N·m),
    ``gap`` (the gripper's, 0.001 mm), ``ctrl_mode``, ``arm_status`` and
    ``motors_on``, each read a new CAN frame unless ``silent``; ``status_reads``
    counts the status reads. Commands act as on the arm: MotionCtrl_2 sets the
    control mode, EnablePiper turns the motors on, a joint target is reached at
    once, a reset clears the status and turns them off.
    """

    def __init__(self, arm_status: int = 0) -> None:
        self.positions = [0] * 6
        self.speeds = [0] * 6
        self.efforts = [0] * 6
        self.gap = 30_000
        self.ctrl_mode = STANDBY
        self.arm_status = arm_status
        self.motors_on = False
        self.silent = False
        self.status_reads = 0
        self.calls: list[Call] = []
        self._frames = 0

    def MotionCtrl_1(self, *args: int) -> None:
        self._record("MotionCtrl_1", *args)
        if args[0] == 0x02:
            self.arm_status, self.ctrl_mode, self.motors_on = 0, STANDBY, False

    def MotionCtrl_2(self, *args: int) -> None:
        self._record("MotionCtrl_2", *args)
        self.ctrl_mode = args[0]

    def EnablePiper(self) -> bool:
        self._record("EnablePiper")
        # Like the SDK: whether the motors were on, then the command to turn them on.
        was_on, self.motors_on = self.motors_on, True
        return was_on

    def JointCtrl(self, *args: int) -> None:
        self._record("JointCtrl", *args)
        self.positions = list(args)

    def GripperCtrl(self, *args: int) -> None:
        self._record("GripperCtrl", *args)

    def DisconnectPort(self) -> None:
        self._record("DisconnectPort")

    def GetArmJointMsgs(self) -> SimpleNamespace:
        joints = {f"joint_{i}": value for i, value in enumerate(self.positions, 1)}
        return SimpleNamespace(time_stamp=self._frame(), joint_state=SimpleNamespace(**joints))

    def GetArmHighSpdInfoMsgs(self) -> SimpleNamespace:
        motors = {
            f"motor_{i}": SimpleNamespace(motor_speed=speed, effort=effort)
            for i, (speed, effort) in enumerate(zip(self.speeds, self.efforts, strict=True), 1)
        }
        return SimpleNamespace(time_stamp=self._frame(), **motors)

    def GetArmGripperMsgs(self) -> SimpleNamespace:
        state = SimpleNamespace(grippers_angle=self.gap)
        return SimpleNamespace(time_stamp=self._frame(), gripper_state=state)

    def GetArmStatus(self) -> SimpleNamespace:
        self.status_reads += 1
        state = SimpleNamespace(ctrl_mode=self.ctrl_mode, arm_status=self.arm_status)
        return SimpleNamespace(time_stamp=self._frame(), arm_status=state)

    def GetArmEnableStatus(self) -> list[bool]:
        return [self.motors_on] * 6

    def sent(self, name: str) -> list[tuple[Any, ...]]:
        """The arguments of each call to ``name`` so far, oldest first."""
        return [args for each, args, _ in self.calls if each == name]

    def _record(self, name: str, *args: Any, **kwargs: Any) -> None:
        self.calls.append((name, args, kwargs))

    def _frame(self) -> float:
        """The time of the latest CAN frame: a new one on every read, unless silent."""
        if not self.silent:
            self._frames += 1
        return float(self._frames)


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


def piper_like_model(tmp_path: Path) -> RobotModel:
    """A stand-in for the Piper's URDF: joint1 to joint6, each within 2 rad of
    0 at up to 3 rad/s, and a gripper gap from 0 to 0.07 m with two fingers
    that copy it."""
    limit = '<limit lower="-2" upper="2" velocity="3" effort="100"/>'
    joints = [
        f'<joint name="joint{i}" type="revolute"><parent link="link{i - 1}"/>'
        f'<child link="link{i}"/>{limit}</joint>'
        for i in range(1, 7)
    ]
    grip = '<limit lower="0" upper="0.07" velocity="3" effort="10"/>'
    joints += [
        '<joint name="gripper" type="prismatic"><parent link="link6"/>'
        f'<child link="gripper_link"/>{grip}</joint>',
        *(
            f'<joint name="gripper_joint{i}" type="prismatic"><parent link="link6"/>'
            f'<child link="finger{i}"/>{grip}<mimic joint="gripper" multiplier="0.5"/></joint>'
            for i in (1, 2)
        ),
    ]
    links = [f'<link name="{name}"/>' for name in ("gripper_link", "finger1", "finger2")]
    links += [f'<link name="link{i}"/>' for i in range(7)]
    path = tmp_path / "piper.urdf"
    path.write_text(f'<robot name="piper">{"".join(links)}{"".join(joints)}</robot>')
    return RobotModel.from_file(path)


class Rig:
    """One ``PiperConnection`` on a fake SDK, with an in-process transport on every port."""

    def __init__(self, module: PiperConnection, sdk: FakePiper) -> None:
        self.module = module
        self.sdk = sdk
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def move(self, **positions: float) -> None:
        """Send targets in rad (m for the gripper) on ``position_command``, e.g. ``move(joint1=0.2)``."""
        names = [f"arm/{joint}" for joint in positions]
        msg = JointState(name=names, position=list(positions.values()))
        self.ports["position_command"].publish(msg)

    def readings(self) -> list[JointState]:
        return list(self.ports["joint_state"].sent)

    def last_error(self) -> str:
        return str(self.module.status().last_error)


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build a ``PiperConnection`` with its gripper on a fake SDK, start it,
    and stop every one at the end of the test."""
    monkeypatch.setattr(connection, "_RESET_SETTLE_S", 0.0)
    made: list[Rig] = []

    def build(sdk: FakePiper | None = None) -> Rig:
        fake = sdk or FakePiper()
        monkeypatch.setattr(connection, "_open_sdk", lambda port: fake)
        module = PiperConnection(
            can_port=PORT,
            gripper=True,
            deadman_timeout_s=DEADMAN_S,
            model=piper_like_model(tmp_path),
        )
        made.append(Rig(module, fake))
        module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def test_start_resets_the_arm_turns_its_motors_on_and_sends_it_home(
    rig: Callable[..., Rig],
) -> None:
    sdk = FakePiper(arm_status=COLLISION)
    sdk.positions = [10_000, 0, 0, 0, 0, 0]
    r = rig(sdk)
    assert [name for name, _, _ in r.sdk.calls] == [
        "MotionCtrl_1",
        "EnablePiper",
        "EnablePiper",
        "MotionCtrl_2",
        "JointCtrl",
        "GripperCtrl",
        "GripperCtrl",
        "JointCtrl",
        "MotionCtrl_2",
        "JointCtrl",
        "MotionCtrl_2",
    ]
    assert r.sdk.sent("MotionCtrl_1") == [(0x02, 0, 0)]
    # Held where it is (again once it takes commands), then sent home (all joints at 0) at
    # 30 %, then back to half speed.
    assert r.sdk.sent("JointCtrl") == [(10_000, 0, 0, 0, 0, 0)] * 2 + [(0, 0, 0, 0, 0, 0)]
    assert [args[2] for args in r.sdk.sent("MotionCtrl_2")] == [50, 30, 50]
    # Off with its errors cleared, then on, at the 30 mm it is at.
    assert [args[0::2] for args in r.sdk.sent("GripperCtrl")] == [(30_000, 0x02), (30_000, 0x01)]
    assert r.module.status().last_error is None


def test_joint_targets_go_out_in_thousandths_of_a_degree(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(joint1=0.2, joint6=-0.1)
    # A joint left out stays where it is.
    assert r.sdk.sent("JointCtrl")[-1] == (11_459, 0, 0, 0, 0, -5_730)


def test_joint_state_names_each_joint_with_its_position_speed_and_torque(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.sdk.positions = [90_000, -45_000, 0, 0, 0, 0]
    r.sdk.speeds = [1_500, 0, 0, 0, 0, -250]
    r.sdk.efforts = [2_000, 0, 0, 0, 0, 0]

    def moved() -> list[JointState]:
        return [m for m in r.readings() if len(m.name) == 6 and m.position[0] > 1.0]

    wait_until(lambda: bool(moved()), timeout=2.0)
    msg = moved()[0]
    assert msg.name == [f"arm/joint{i}" for i in range(1, 7)]
    assert msg.position == pytest.approx([1.5708, -0.7854, 0, 0, 0, 0], abs=1e-4)
    assert msg.velocity == pytest.approx([1.5, 0, 0, 0, 0, -0.25])
    assert msg.effort == pytest.approx([2.0, 0, 0, 0, 0, 0])


def test_the_gripper_goes_out_as_its_gap_with_every_command(rig: Callable[..., Rig]) -> None:
    r = rig()
    before = len(r.sdk.sent("GripperCtrl"))
    sent = len(r.sdk.sent("JointCtrl"))
    r.move(gripper=0.07)
    r.move(gripper=0.07)
    r.move(gripper=0.0)
    # The gap in 0.001 mm, 1 N·m of squeeze, on.
    assert r.sdk.sent("GripperCtrl")[before:] == [
        (70_000, 1000, 0x01, 0),
        (70_000, 1000, 0x01, 0),
        (0, 1000, 0x01, 0),
    ]
    # The gripper alone moves no arm joint.
    assert len(r.sdk.sent("JointCtrl")) == sent


def test_the_gripper_is_read_in_metres_in_a_message_of_its_own(rig: Callable[..., Rig]) -> None:
    r = rig()
    wait_until(lambda: any(m.name == ["arm/gripper"] for m in r.readings()), timeout=2.0)
    gripper = next(m for m in r.readings() if m.name == ["arm/gripper"])
    assert gripper.position == pytest.approx([0.03])
    assert gripper.velocity == gripper.effort == []


def test_a_target_past_the_jump_guard_is_refused_and_nothing_reaches_the_sdk(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    before = list(r.sdk.calls)
    # 3 rad/s for 0.1 s allows 0.3 rad from where joint 3 is (0).
    r.move(joint3=0.4, gripper=0.05)
    assert r.sdk.calls == before
    assert "arm/joint3 told 0.400 rad" in r.last_error()
    r.move(joint3=0.3)
    assert r.sdk.sent("JointCtrl")[-1] == (0, 0, 17_189, 0, 0, 0)


def test_an_arm_stopped_outside_dimos_takes_no_command_until_its_errors_are_cleared(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    # Readings flow once the arm is taking commands.
    wait_until(lambda: bool(r.readings()), timeout=2.0)
    # What the arm reports once its power is cut and back: standby, motors off.
    r.sdk.ctrl_mode, r.sdk.motors_on = STANDBY, False
    wait_until(lambda: "stopped outside DimOS" in r.last_error(), timeout=2.0)
    sent = len(r.sdk.sent("JointCtrl"))
    r.move(joint1=0.1)
    assert len(r.sdk.sent("JointCtrl")) == sent

    r.module.clear_errors()
    resets = len(r.sdk.sent("MotionCtrl_1"))
    r.move(joint1=0.1)
    assert r.sdk.sent("JointCtrl")[-1][0] == 5_730
    # Already taking commands: a second clear does not reset the arm again.
    r.module.clear_errors()
    assert len(r.sdk.sent("MotionCtrl_1")) == resets


def test_teach_mode_stops_every_command(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.sdk.ctrl_mode = TEACHING
    wait_until(lambda: "control mode 2" in r.last_error(), timeout=2.0)
    sent = len(r.sdk.sent("JointCtrl"))
    r.move(joint1=0.1)
    assert len(r.sdk.sent("JointCtrl")) == sent


def test_a_silent_arm_is_not_reported_and_takes_no_joint_target(
    rig: Callable[..., Rig], monkeypatch: pytest.MonkeyPatch
) -> None:
    r = rig()
    wait_until(lambda: bool(r.readings()), timeout=2.0)
    r.sdk.silent = True
    # Two status reads later, a reading started before the arm fell silent is done.
    reads = r.sdk.status_reads
    wait_until(lambda: r.sdk.status_reads >= reads + 2, timeout=2.0)
    count = len(r.readings())
    real = time.monotonic
    monkeypatch.setattr(time, "monotonic", lambda: real() + 1.0)
    reads = r.sdk.status_reads
    wait_until(lambda: r.sdk.status_reads >= reads + 2, timeout=2.0)
    assert len(r.readings()) == count
    sent = len(r.sdk.sent("JointCtrl"))
    with pytest.raises(PiperError, match="the joints were last read .* s ago"):
        r.module.write({"arm/joint1/position": 0.1})
    assert len(r.sdk.sent("JointCtrl")) == sent


def test_stopping_sends_the_arm_home_then_puts_it_in_its_emergency_stop(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(joint1=0.1)
    before = len(r.sdk.calls)
    r.module.stop()
    assert r.sdk.sent("JointCtrl")[-1] == (0, 0, 0, 0, 0, 0)
    assert r.sdk.calls[-2:] == [("MotionCtrl_1", (0x01, 0, 0), {}), ("DisconnectPort", (), {})]
    assert len(r.sdk.calls) > before + 2
    assert not r.module.status().connected


def test_an_arm_with_a_fault_is_not_sent_home_when_stopping(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.sdk.ctrl_mode = TEACHING
    wait_until(lambda: "control mode 2" in r.last_error(), timeout=2.0)
    before = len(r.sdk.calls)
    r.module.stop()
    assert r.sdk.calls[before:] == [("MotionCtrl_1", (0x01, 0, 0), {}), ("DisconnectPort", (), {})]


@pytest.mark.self_hosted
def test_the_piper_urdf_drives_six_joints_and_the_gripper_but_not_its_fingers() -> None:
    table = limits(PiperConfig(can_port=PORT, gripper=True))
    assert list(table) == [
        *(f"arm/joint{i}/position" for i in range(1, 7)),
        "arm/gripper/position",
    ]
    assert table["arm/gripper/position"].hi == pytest.approx(0.07)
