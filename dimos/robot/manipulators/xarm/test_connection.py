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

"""Tests for ``XArmConnection`` against a fake SDK, with no arm attached.

The fake stands in for ``XArmAPI`` where the module opens it and records what
the module sends. Every port gets an in-process transport, and the robot model
is a small URDF written here, so nothing touches the network.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import functools
from pathlib import Path
import time
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.limits import limits
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.xarm import connection
from dimos.robot.manipulators.xarm.config import XARM_ARMS, xarm_model
from dimos.robot.manipulators.xarm.connection import XArmConfig, XArmConnection, XArmError
from dimos.utils.testing.waiting import wait_until

IP = "192.0.2.1"
# Long enough that the deadman never halts mid-test, except where a test sets its own.
DEADMAN_S = 5.0
REST = [0.0] * 7

Call = tuple[str, tuple[Any, ...], dict[str, Any]]


class FakeXArmAPI:
    """An xArm7 at rest, at the end of the SDK's ``XArmAPI``.

    Every call that sends something is kept in ``calls`` and returns 0, or
    the code in ``fail`` once, e.g. ``fail["set_servo_angle_j"] = 9``. Reads
    answer from ``positions``, ``speeds``, ``efforts`` (seven each, as the SDK
    gives) and ``gripper_pulses``.
    """

    SENDS = frozenset(
        {
            "motion_enable",
            "set_mode",
            "set_state",
            "clean_warn",
            "clean_error",
            "set_servo_angle_j",
            "vc_set_joint_velocity",
            "set_gripper_enable",
            "set_gripper_position",
            "disconnect",
        }
    )

    def __init__(
        self,
        axis: int = 7,
        device_type: int = 7,
        version: tuple[int, int, int] = (2, 4, 0),
        error_code: int = 0,
    ) -> None:
        self.connected = True
        self.axis = axis
        self.device_type = device_type
        self.version_number = version
        self.error_code = error_code
        self.warn_code = 0
        self.positions = list(REST)
        self.speeds = list(REST)
        self.efforts = list(REST)
        self.gripper_pulses = 850.0
        self.calls: list[Call] = []
        self.fail: dict[str, int] = {}
        self.on_error_warn: Callable[[dict[str, int]], None] | None = None

    def __getattr__(self, name: str) -> Callable[..., int]:
        if name not in FakeXArmAPI.SENDS:
            raise AttributeError(name)
        return functools.partial(self._send, name)

    def _send(self, name: str, *args: Any, **kwargs: Any) -> int:
        self.calls.append((name, args, kwargs))
        return self.fail.pop(name, 0)

    def get_joint_states(self, is_radian: bool = False) -> tuple[int, list[list[float]]]:
        assert is_radian
        return 0, [list(self.positions), list(self.speeds), list(self.efforts)]

    def get_gripper_position(self) -> tuple[int, float]:
        return 0, self.gripper_pulses

    def register_error_warn_changed_callback(
        self, callback: Callable[[dict[str, int]], None]
    ) -> bool:
        self.on_error_warn = callback
        return True

    def sent(self, name: str) -> list[Call]:
        """The calls to ``name`` so far, oldest first."""
        return [call for call in self.calls if call[0] == name]


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


def xarm_like_model(tmp_path: Path, dof: int, gripper: bool) -> RobotModel:
    """A stand-in for the xArm's URDF: joint1 to joint``dof``, each within
    3.1 rad of 0 at up to 3.14 rad/s; with ``gripper``, a drive_joint from 0
    (open) to 0.85 rad (closed) and a finger that copies it."""
    limit = '<limit lower="-3.1" upper="3.1" velocity="3.14" effort="50"/>'
    joints = [
        f'<joint name="joint{i}" type="revolute"><parent link="link{i - 1}"/>'
        f'<child link="link{i}"/>{limit}</joint>'
        for i in range(1, dof + 1)
    ]
    links = [f'<link name="link{i}"/>' for i in range(dof + 1)]
    if gripper:
        grip = '<limit lower="0" upper="0.85" velocity="2" effort="50"/>'
        links += ['<link name="knuckle"/>', '<link name="finger"/>']
        joints += [
            f'<joint name="drive_joint" type="revolute"><parent link="link{dof}"/>'
            f'<child link="knuckle"/>{grip}</joint>',
            f'<joint name="finger_joint" type="revolute"><parent link="link{dof}"/>'
            f'<child link="finger"/>{grip}<mimic joint="drive_joint"/></joint>',
        ]
    path = tmp_path / f"xarm{dof}{'_gripper' if gripper else ''}.urdf"
    path.write_text(f'<robot name="xarm">{"".join(links)}{"".join(joints)}</robot>')
    return RobotModel.from_file(path)


class Rig:
    """One ``XArmConnection`` on a fake SDK, with an in-process transport on every port."""

    def __init__(self, module: XArmConnection, sdk: FakeXArmAPI) -> None:
        self.module = module
        self.sdk = sdk
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def move(self, **positions: float) -> None:
        """Send joint targets in rad on ``position_command``, e.g. ``move(joint1=0.2)``."""
        names = [f"arm/{joint}" for joint in positions]
        msg = JointState(name=names, position=list(positions.values()))
        self.ports["position_command"].publish(msg)

    def jog(self, **speeds: float) -> None:
        """Send joint speeds in rad/s on ``velocity_command``, e.g. ``jog(joint1=0.5)``."""
        names = [f"arm/{joint}" for joint in speeds]
        msg = JointState(name=names, velocity=list(speeds.values()))
        self.ports["velocity_command"].publish(msg)

    def readings(self) -> list[JointState]:
        return list(self.ports["joint_state"].sent)


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build an ``XArmConnection`` on a fake SDK, start it unless told not to,
    and stop every one at the end of the test."""
    made: list[Rig] = []

    def build(sdk: FakeXArmAPI | None = None, start: bool = True, **config: Any) -> Rig:
        fake = sdk or FakeXArmAPI()
        monkeypatch.setattr(connection, "XArmAPI", lambda ip, is_radian: fake)
        config.setdefault("arm", "xarm7")
        config.setdefault("ip", IP)
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        dof = XARM_ARMS[config["arm"]][0]
        config.setdefault(
            "model", xarm_like_model(tmp_path, dof, config.get("gripper") is not None)
        )
        made.append(Rig(XArmConnection(**config), fake))
        if start:
            made[-1].module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


@pytest.mark.parametrize(
    ("found", "arm", "match"),
    [
        ({"axis": 6, "device_type": 6}, "xarm7", "says xarm7, but .* is an xarm6"),
        ({"axis": 6, "device_type": 9}, "xarm6", "says xarm6, but .* is an lite6"),
        ({"axis": 6, "device_type": 12}, "xarm6", "says xarm6, but .* is an uf850"),
        ({"version": (1, 8, 3)}, "xarm7", "firmware 1.8.3; .* needs 1.9.0"),
    ],
)
def test_the_wrong_arm_or_old_firmware_fails_start_and_lets_go_of_it(
    rig: Callable[..., Rig], found: dict[str, Any], arm: str, match: str
) -> None:
    sdk = FakeXArmAPI(**found)
    r = rig(sdk, start=False, arm=arm)
    with pytest.raises(XArmError, match=match):
        r.module.start()
    assert [call[0] for call in sdk.calls] == ["disconnect"]


def test_nobody_at_the_address_fails_start_naming_it(
    rig: Callable[..., Rig], monkeypatch: pytest.MonkeyPatch
) -> None:
    r = rig(start=False)

    def unreachable(ip: str, is_radian: bool) -> FakeXArmAPI:
        raise Exception("connect socket failed")

    monkeypatch.setattr(connection, "XArmAPI", unreachable)
    with pytest.raises(ConnectionError, match=f"no xArm answers at {IP}: connect socket failed"):
        r.module.start()


def test_an_arm_in_error_at_start_takes_no_command_until_its_errors_are_cleared(
    rig: Callable[..., Rig],
) -> None:
    r = rig(FakeXArmAPI(error_code=22))
    assert "error 22" in str(r.module.status().last_error)
    assert r.sdk.calls == []

    r.move(joint1=0.1)
    assert r.sdk.calls == []
    assert "clear_errors()" in str(r.module.status().last_error)

    r.module.clear_errors()
    assert [(name, args) for name, args, _ in r.sdk.calls] == [
        ("clean_error", ()),
        ("clean_warn", ()),
        ("motion_enable", ()),
        ("set_mode", (1,)),
        ("set_state", (0,)),
    ]
    r.move(joint1=0.1)
    assert len(r.sdk.sent("set_servo_angle_j")) == 1


def test_an_error_while_running_stops_every_command(rig: Callable[..., Rig]) -> None:
    r = rig()
    assert r.sdk.on_error_warn is not None
    r.sdk.on_error_warn({"error_code": 31, "warn_code": 0})
    r.move(joint1=0.1)
    r.jog(joint1=0.1)
    assert r.sdk.sent("set_servo_angle_j") == r.sdk.sent("vc_set_joint_velocity") == []
    assert "error 31" in str(r.module.status().last_error)


def test_a_position_command_is_a_servo_target_in_radians(rig: Callable[..., Rig]) -> None:
    r = rig()
    # Ready for joint targets from the start: motors on, servo mode, ready.
    assert [call[0] for call in r.sdk.calls] == ["motion_enable", "set_mode", "set_state"]
    r.move(joint1=0.2, joint7=-0.1)
    # A joint left out stays where it is.
    assert r.sdk.sent("set_servo_angle_j") == [
        ("set_servo_angle_j", ([0.2, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1],), {"is_radian": True})
    ]
    assert len(r.sdk.sent("set_mode")) == 1


def test_a_velocity_command_expires_on_the_arm_and_modes_change_only_when_needed(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.jog(joint1=0.5)
    r.jog(joint1=0.5)
    r.move(joint1=0.0)
    r.move(joint1=0.0)
    r.jog(joint2=-0.3)

    assert [args for _, args, _ in r.sdk.sent("set_mode")] == [(1,), (4,), (1,), (4,)]
    jogs = r.sdk.sent("vc_set_joint_velocity")
    assert [args[0] for _, args, _ in jogs] == [
        [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    assert jogs[0][2] == {"is_radian": True, "duration": 2 * DEADMAN_S}


def test_the_gripper_goes_out_in_sdk_pulses_only_when_its_target_changes(
    rig: Callable[..., Rig],
) -> None:
    r = rig(gripper="xarm")
    assert len(r.sdk.sent("set_gripper_enable")) == 1
    r.move(drive_joint=0.0)
    r.move(drive_joint=0.0)
    r.move(drive_joint=0.85)
    pulses = [args[0] for _, args, _ in r.sdk.sent("set_gripper_position")]
    assert pulses == pytest.approx([850.0, 0.0])
    # The gripper alone moves no arm joint.
    assert r.sdk.sent("set_servo_angle_j") == []


def test_the_gripper_is_read_in_radians_in_a_message_of_its_own(rig: Callable[..., Rig]) -> None:
    sdk = FakeXArmAPI()
    sdk.gripper_pulses = 425.0
    r = rig(sdk, gripper="xarm")
    wait_until(lambda: any(m.name == ["arm/drive_joint"] for m in r.readings()), timeout=2.0)
    gripper = next(m for m in r.readings() if m.name == ["arm/drive_joint"])
    assert gripper.position == pytest.approx([0.425])
    assert gripper.velocity == gripper.effort == []


def test_a_target_past_the_jump_guard_is_refused_and_nothing_reaches_the_sdk(
    rig: Callable[..., Rig],
) -> None:
    r = rig(gripper="xarm")
    before = list(r.sdk.calls)
    # 3.14 rad/s for 0.1 s allows 0.314 rad from where joint 3 is (0).
    r.move(joint3=0.4, drive_joint=0.5)
    assert r.sdk.calls == before
    assert "arm/joint3 told 0.400 rad" in str(r.module.status().last_error)
    r.move(joint3=0.3)
    assert len(r.sdk.sent("set_servo_angle_j")) == 1


def test_a_refused_sdk_call_halts_the_arm_and_shows_in_status(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.sdk.fail["vc_set_joint_velocity"] = 12
    r.jog(joint1=0.5)
    assert "vc_set_joint_velocity returned code 12" in str(r.module.status().last_error)
    # The halt after the failure tells every joint 0 rad/s.
    assert r.sdk.sent("vc_set_joint_velocity")[-1] == (
        "vc_set_joint_velocity",
        (REST,),
        {"is_radian": True},
    )


@pytest.mark.parametrize(
    ("fault", "rearmed"),
    [({}, True), ({"error_code": 31}, False), ({"warn_code": 11}, False)],
)
def test_an_arm_stopped_outside_dimos_is_made_ready_again_unless_it_has_a_fault(
    rig: Callable[..., Rig], fault: dict[str, int], rearmed: bool
) -> None:
    r = rig()
    # Set straight on the SDK, as if its error callback has not run yet.
    for name, code in fault.items():
        setattr(r.sdk, name, code)
    before = len(r.sdk.calls)
    # What the arm answers once xArm Studio or its stop button has stopped it.
    r.sdk.fail["set_servo_angle_j"] = 9
    r.move(joint1=0.1)
    if rearmed:
        assert [(name, args) for name, args, _ in r.sdk.calls[before:]] == [
            ("set_servo_angle_j", ([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],)),
            ("set_mode", (1,)),
            ("set_state", (0,)),
            ("set_servo_angle_j", ([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],)),
        ]
        assert r.module.status().last_error is None
    else:
        assert [call[0] for call in r.sdk.calls[before:]] == ["set_servo_angle_j"]
        assert "returned code 9" in str(r.module.status().last_error)
        # Still not ready: the next command is dropped before it reaches the SDK.
        r.move(joint1=0.1)
        assert len(r.sdk.sent("set_servo_angle_j")) == 1
        assert "set_servo_angle_j dropped" in str(r.module.status().last_error)


def test_joint_targets_are_refused_once_the_joint_reading_is_old(
    rig: Callable[..., Rig], monkeypatch: pytest.MonkeyPatch
) -> None:
    r = rig()
    monkeypatch.setattr(r.sdk, "get_joint_states", lambda is_radian: (3, None))
    wait_until(lambda: "reading failed" in str(r.module.status().last_error), timeout=2.0)
    real = time.monotonic
    monkeypatch.setattr(time, "monotonic", lambda: real() + 1.0)
    with pytest.raises(XArmError, match="the joints were last read .* s ago"):
        r.module.write({"arm/joint1/position": 0.1})
    # Speeds do not depend on where the joints are, so they still go out.
    r.jog(joint1=0.5)
    assert r.sdk.sent("set_servo_angle_j") == []
    assert len(r.sdk.sent("vc_set_joint_velocity")) == 1


def test_the_deadman_halts_a_jog_with_zero_speeds(rig: Callable[..., Rig]) -> None:
    r = rig(deadman_timeout_s=0.05)
    r.jog(joint1=0.5)
    wait_until(lambda: r.module.status().deadman_fired, timeout=2.0)
    assert r.sdk.sent("vc_set_joint_velocity")[-1][1] == (REST,)


def test_joint_state_names_each_joint_with_its_position_speed_and_torque(
    rig: Callable[..., Rig],
) -> None:
    sdk = FakeXArmAPI(axis=6, device_type=6)
    # The SDK always gives seven of each; a six-axis arm uses the first six.
    sdk.positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 9.0]
    sdk.speeds = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 9.0]
    sdk.efforts = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 9.0]
    r = rig(sdk, arm="xarm6")
    wait_until(lambda: bool(r.readings()), timeout=2.0)
    msg = r.readings()[-1]
    assert msg.name == [f"arm/joint{i}" for i in range(1, 7)]
    assert msg.position == sdk.positions[:6]
    assert msg.velocity == sdk.speeds[:6]
    assert msg.effort == sdk.efforts[:6]


def test_the_coordinator_takes_the_arm_and_gripper_messages_as_one_robot(
    rig: Callable[..., Rig], tmp_path: Path
) -> None:
    r = rig(gripper="xarm")
    names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "drive_joint"]
    model = xarm_like_model(tmp_path, 7, gripper=True)
    seen = ConnectionSource(
        "arm",
        ["position_command", "velocity_command"],
        ["joint_state"],
        model.with_renamed_joints({joint: f"arm/{joint}" for joint in names}),
        reading_timeout_s=1.0,
    )
    wait_until(lambda: any(m.name == ["arm/drive_joint"] for m in r.readings()), timeout=2.0)
    for msg in r.readings():
        seen.on_message("joint_state", msg)
    assert seen.ready_for_control()
    assert seen.read_joints().joint_positions["arm/drive_joint"] == pytest.approx(0.0)


def test_stopping_stops_the_arm_and_lets_go_of_it(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.module.stop()
    assert r.sdk.calls[-2:] == [("set_state", (4,), {}), ("disconnect", (), {})]
    assert not r.module.status().connected


def test_only_the_xarm_gripper_is_driven() -> None:
    with pytest.raises(ValidationError, match="not the BIO"):
        XArmConfig(arm="xarm7", ip=IP, gripper="bio")
    with pytest.raises(ValidationError, match="Lite 6 gripper"):
        XArmConfig(arm="lite6", ip=IP, gripper="xarm")


def test_the_model_comes_from_the_arm_and_gripper() -> None:
    assert XArmConfig(arm="xarm6", ip=IP, gripper="xarm").model == xarm_model("xarm6", "xarm")


@pytest.mark.self_hosted
def test_the_xarm7_urdf_drives_seven_joints_and_the_gripper_but_not_its_followers() -> None:
    table = limits(XArmConfig(arm="xarm7", ip=IP, gripper="xarm"))
    joints = list(dict.fromkeys(key.rsplit("/", 1)[0] for key in table))
    assert joints == [*(f"arm/joint{i}" for i in range(1, 8)), "arm/drive_joint"]
    assert table["arm/drive_joint/position"].hi == pytest.approx(0.85)
    assert table["arm/joint1/velocity"].hi == pytest.approx(3.14)
