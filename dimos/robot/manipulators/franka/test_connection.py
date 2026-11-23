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

"""Tests for ``FrankaConnection`` against a fake pylibfranka, with no arm attached.

The fake stands in for the ``pylibfranka`` module where the connection imports
it and records what the connection sends. Every port gets an in-process
transport, and the robot model is a small URDF written here, so nothing
touches the network.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from enum import Enum
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.control.contract.keys import Interface
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.franka import connection
from dimos.robot.manipulators.franka.config import (
    FR3_FINGER_JOINT,
    FR3_VELOCITY_MAX,
    fr3_joint_names,
)
from dimos.robot.manipulators.franka.connection import (
    FrankaConfig,
    FrankaConnection,
    FrankaError,
)
from dimos.utils.testing.waiting import wait_until

IP = "192.0.2.1"
# Long enough that the deadman never halts mid-test, except where a test sets its own.
DEADMAN_S = 5.0
REST = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


class Mode(Enum):
    Idle = 1
    Move = 2
    Reflex = 4
    UserStopped = 5


class FakeErrors(list):  # type: ignore[type-arg]
    """libfranka's error list: falsy when empty, the active error names otherwise."""


def robot_state(q: list[float], mode: Mode = Mode.Idle, errors: list[str] | None = None) -> Any:
    return SimpleNamespace(
        q=list(q),
        dq=[0.01 * (i + 1) for i in range(7)],
        tau_J=[0.1 * (i + 1) for i in range(7)],
        robot_mode=mode,
        current_errors=FakeErrors(errors or []),
        last_motion_errors=FakeErrors([]),
    )


class FakeLibfranka:
    """The parts of ``pylibfranka`` the connection uses, recording every send in ``calls``."""

    class RealtimeConfig(Enum):
        # libfranka's own names.
        kEnforce = 0  # noqa: N815
        kIgnore = 1  # noqa: N815

    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []
        self.lock = threading.Lock()
        self.q = list(REST)
        self.mode = Mode.Idle
        self.errors: list[str] = []
        self.width = 0.08
        self.unreachable = False
        self.refuse_target = False
        # Every Hand call raises as a dropped link does.
        self.hand_down = False
        # The session's own feedback: None while it runs, else libfranka's message.
        self.session_ended: str | None = None
        fake = self

        class Robot:
            def __init__(self, ip: str, realtime: Any) -> None:
                if fake.unreachable:
                    raise RuntimeError("Connection timeout")
                fake.record("Robot", ip, realtime)

            def read_once(self) -> Any:
                return robot_state(fake.q, fake.mode, fake.errors)

            def set_collision_behavior(self, *args: Any) -> None:
                fake.record("set_collision_behavior", *args)

            def automatic_error_recovery(self) -> None:
                fake.record("automatic_error_recovery")
                fake.mode, fake.errors = Mode.Idle, []

        class Gripper:
            def __init__(self, ip: str) -> None:
                fake.record("Gripper", ip)

            def read_once(self) -> Any:
                if fake.hand_down:
                    raise RuntimeError("Net Exception")
                return SimpleNamespace(width=fake.width, max_width=0.08)

            def move(self, width: float, speed: float) -> bool:
                if fake.hand_down:
                    raise RuntimeError("Net Exception")
                fake.record("move", width, speed)
                fake.width = width
                return True

            def grasp(self, width: float, speed: float, force: float, **eps: float) -> bool:
                fake.record("grasp", width, speed, force, **eps)
                fake.width = 0.03  # stopped on an object
                return True

        class Handler:
            def set_joint_position_target(self, target: Any) -> Any:
                fake.record("set_joint_position_target", list(target.joint_positions))
                if fake.refuse_target:
                    return SimpleNamespace(was_successful=False, error_message="aborted")
                return SimpleNamespace(was_successful=True, error_message=None)

            def get_target_feedback(self) -> Any:
                if fake.session_ended is not None:
                    return SimpleNamespace(
                        status=SimpleNamespace(name="kAborted"), error_message=fake.session_ended
                    )
                return SimpleNamespace(
                    status=SimpleNamespace(name="kExecuting"), error_message=None
                )

            def stop_control(self) -> None:
                # Finishes with all-zero joint positions; the connection must never call it.
                fake.record("stop_control")

            def __del__(self) -> None:
                # Destroying libfranka's handler cancels the motion (a StopMove).
                fake.record("cancel_motion")

        class AsyncPositionControlHandler:
            @staticmethod
            def Configuration(**kwargs: Any) -> Any:
                return SimpleNamespace(**kwargs)

            @staticmethod
            def JointPositionTarget(joint_positions: list[float]) -> Any:
                return SimpleNamespace(joint_positions=joint_positions)

            @staticmethod
            def configure(robot: Any, config: Any) -> Any:
                fake.record("configure", **vars(config))
                return SimpleNamespace(handler=Handler(), error_message=None)

        self.Robot = Robot
        self.Gripper = Gripper
        self.AsyncPositionControlHandler = AsyncPositionControlHandler

    def record(self, name: str, *args: Any, **kwargs: Any) -> None:
        with self.lock:
            self.calls.append((name, args, kwargs))

    def sent(self, name: str) -> list[tuple[tuple[Any, ...], dict[str, Any]]]:
        with self.lock:
            return [(args, kwargs) for call, args, kwargs in self.calls if call == name]

    def targets(self) -> list[list[float]]:
        return [args[0] for args, _ in self.sent("set_joint_position_target")]

    def names(self) -> list[str]:
        with self.lock:
            return [call for call, _, _ in self.calls]


def fr3_like_model(tmp_path: Path) -> RobotModel:
    """A stand-in for the FR3's URDF with its joint names, scaled speed limits and Hand."""
    lower = [-2.9, -1.8, -2.9, -3.07, -2.87, 0.44, -3.05]
    upper = [2.9, 1.8, 2.9, -0.12, 2.87, 4.62, 3.05]
    links = [f'<link name="fr3_link{i}"/>' for i in range(8)]
    joints = [
        f'<joint name="arm/fr3_joint{i}" type="revolute"><parent link="fr3_link{i - 1}"/>'
        f'<child link="fr3_link{i}"/><limit lower="{lower[i - 1]}" upper="{upper[i - 1]}" '
        f'velocity="{FR3_VELOCITY_MAX[i - 1] * 0.25}" effort="87"/></joint>'
        for i in range(1, 8)
    ]
    finger = '<limit lower="0" upper="0.04" velocity="0.2" effort="100"/>'
    links += ['<link name="fr3_leftfinger"/>', '<link name="fr3_rightfinger"/>']
    joints += [
        f'<joint name="arm/{FR3_FINGER_JOINT}" type="prismatic"><parent link="fr3_link7"/>'
        f'<child link="fr3_leftfinger"/>{finger}</joint>',
        '<joint name="fr3_finger_joint2" type="prismatic"><parent link="fr3_link7"/>'
        f'<child link="fr3_rightfinger"/>{finger}<mimic joint="arm/{FR3_FINGER_JOINT}"/></joint>',
    ]
    path = tmp_path / "fr3.urdf"
    path.write_text(f'<robot name="fr3">{"".join(links)}{"".join(joints)}</robot>')
    return RobotModel.from_file(path)


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


class Rig:
    """One ``FrankaConnection`` on a fake pylibfranka, with an in-process transport on every port."""

    def __init__(self, module: FrankaConnection, franka: FakeLibfranka) -> None:
        self.module = module
        self.franka = franka
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def move(self, **positions: float) -> None:
        """Send targets on ``position_command``, e.g. ``move(fr3_joint1=0.02)``."""
        names = [f"arm/{joint}" for joint in positions]
        self.ports["position_command"].publish(
            JointState(name=names, position=list(positions.values()))
        )

    def readings(self) -> list[JointState]:
        return list(self.ports["joint_state"].sent)


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build a ``FrankaConnection`` on a fake pylibfranka, start it unless told
    not to, and stop every one at the end of the test."""
    made: list[Rig] = []

    def build(franka: FakeLibfranka | None = None, start: bool = True, **config: Any) -> Rig:
        fake = franka or FakeLibfranka()
        monkeypatch.setattr(connection, "_import_libfranka", lambda: fake)
        config.setdefault("ip", IP)
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        config.setdefault("model", fr3_like_model(tmp_path))
        # The fake pylibfranka lives in this process; on hardware the Hand gets its own.
        config.setdefault("hand_process", False)
        made.append(Rig(FrankaConnection(**config), fake))
        if start:
            made[-1].module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def test_nobody_at_the_address_fails_start_naming_fci(rig: Callable[..., Rig]) -> None:
    franka = FakeLibfranka()
    franka.unreachable = True
    r = rig(franka, start=False)
    with pytest.raises(ConnectionError, match=f"no FR3 answers over FCI at {IP}"):
        r.module.start()


def test_start_sets_collision_limits_and_starts_no_control_session(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    assert r.franka.names()[:3] == ["Robot", "set_collision_behavior", "Gripper"]
    # No real-time kernel required by default.
    ((args, _),) = r.franka.sent("Robot")
    assert args == (IP, FakeLibfranka.RealtimeConfig.kIgnore)
    # Starting a session sends all-zero positions, so none starts before a command.
    assert "configure" not in r.franka.names()
    assert r.franka.targets() == []


def test_the_first_command_starts_a_session_holding_the_arm_where_it_is(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(fr3_joint1=0.02, fr3_joint7=0.8)
    ((_, config),) = r.franka.sent("configure")
    assert config["maximum_joint_velocities"] == pytest.approx([v * 0.25 for v in FR3_VELOCITY_MAX])
    hold, target = r.franka.targets()
    assert hold == pytest.approx(REST)
    # A joint left out stays where it is.
    assert target == pytest.approx([0.02, *REST[1:6], 0.8])
    r.move(fr3_joint1=0.03)
    assert len(r.franka.sent("configure")) == 1


def test_a_target_past_the_jump_guard_is_refused_and_nothing_reaches_the_arm(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(fr3_joint3=0.4)
    assert r.franka.targets() == []
    assert "configure" not in r.franka.names()
    assert "arm/fr3_joint3 told 0.400 rad" in str(r.module.status().last_error)


def test_a_joint_leaving_its_band_stops_the_robot(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(fr3_joint1=0.02)
    # Joint 1 should be between 0.0 and 0.02 rad; it reads 0.2.
    r.franka.q = [0.2, *REST[1:]]
    wait_until(lambda: "drift guard" in str(r.module.status().last_error), timeout=2.0)
    assert "arm/fr3_joint1 at 0.200 rad" in str(r.module.status().last_error)
    assert "cancel_motion" in r.franka.names()
    r.move(fr3_joint1=0.2)
    assert len(r.franka.targets()) == 2


def test_a_reflex_stops_commands_until_errors_are_cleared(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(fr3_joint1=0.05)
    r.franka.mode, r.franka.errors = Mode.Reflex, ["joint_reflex"]
    wait_until(lambda: "Reflex" in str(r.module.status().last_error), timeout=2.0)
    assert "joint_reflex" in str(r.module.status().last_error)
    assert "cancel_motion" in r.franka.names()
    r.move(fr3_joint1=0.05)
    assert len(r.franka.targets()) == 2

    r.module.clear_errors()
    assert r.franka.names()[-1] == "automatic_error_recovery"
    r.move(fr3_joint1=0.05)
    assert len(r.franka.sent("configure")) == 2
    assert r.franka.targets()[-2:] == [pytest.approx(REST), pytest.approx([0.05, *REST[1:]])]


def test_an_arm_in_reflex_at_start_takes_no_command_until_its_errors_are_cleared(
    rig: Callable[..., Rig],
) -> None:
    franka = FakeLibfranka()
    franka.mode = Mode.Reflex
    r = rig(franka)
    r.move(fr3_joint1=0.05)
    assert "configure" not in franka.names()
    assert franka.targets() == []
    r.module.clear_errors()
    r.move(fr3_joint1=0.05)
    assert len(franka.targets()) == 2


def test_a_refused_hold_faults_the_arm_and_cancels_the_session(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.franka.refuse_target = True
    r.move(fr3_joint1=0.05)
    assert "refused its hold target" in str(r.module.status().last_error)
    assert "cancel_motion" in r.franka.names()
    r.franka.refuse_target = False
    r.move(fr3_joint1=0.05)
    assert len(r.franka.targets()) == 1
    r.module.clear_errors()
    r.move(fr3_joint1=0.05)
    assert len(r.franka.targets()) == 3


def test_closing_the_hand_grasps_and_opening_moves_each_sent_once(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.franka.width = 0.08
    for _ in range(3):
        r.move(**{FR3_FINGER_JOINT: 0.0})
    wait_until(lambda: bool(r.franka.sent("grasp")), timeout=2.0)
    ((args, kwargs),) = r.franka.sent("grasp")
    assert args[0] == 0.0 and args[2] == 20.0
    assert kwargs == {"epsilon_inner": 0.08, "epsilon_outer": 0.08}

    r.move(**{FR3_FINGER_JOINT: 0.04})
    wait_until(lambda: bool(r.franka.sent("move")), timeout=2.0)
    # One finger at 0.04 m is the Hand 0.08 m open.
    assert r.franka.sent("move")[0][0][0] == pytest.approx(0.08)
    assert len(r.franka.sent("grasp")) == 1


def test_joint_state_names_each_joint_and_the_hand_in_its_own_message(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    wait_until(
        lambda: any(m.name == [f"arm/{FR3_FINGER_JOINT}"] for m in r.readings()), timeout=2.0
    )
    arm = next(m for m in r.readings() if m.name == fr3_joint_names("arm"))
    assert arm.position == pytest.approx(REST)
    assert arm.velocity == pytest.approx([0.01 * (i + 1) for i in range(7)])
    assert arm.effort == pytest.approx([0.1 * (i + 1) for i in range(7)])
    hand = next(m for m in r.readings() if m.name == [f"arm/{FR3_FINGER_JOINT}"])
    assert hand.position == pytest.approx([0.04])


def test_the_deadman_holds_the_arm_where_it_is(rig: Callable[..., Rig]) -> None:
    r = rig(deadman_timeout_s=0.05)
    r.move(fr3_joint1=0.02)
    wait_until(lambda: r.module.status().deadman_fired, timeout=2.0)
    targets = [args[0] for args, _ in r.franka.sent("set_joint_position_target")]
    assert targets[-1] == pytest.approx(REST)


def test_stopping_cancels_the_session_and_never_finishes_it_with_zeros(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(fr3_joint1=0.05)
    r.module.stop()
    assert "cancel_motion" in r.franka.names()
    assert "stop_control" not in r.franka.names()
    assert not r.module.status().connected


def test_only_position_commands_and_valid_settings_are_accepted(tmp_path: Path) -> None:
    model = fr3_like_model(tmp_path)
    with pytest.raises(ValidationError, match="positions only"):
        FrankaConfig(
            ip=IP,
            model=model,
            joint_commands={"arm/fr3_joint1": (Interface.VELOCITY,)},
        )
    with pytest.raises(ValidationError, match="speed_fraction"):
        FrankaConfig(ip=IP, model=model, speed_fraction=1.5)
    with pytest.raises(ValidationError):
        FrankaConfig(ip="not-an-ip", model=model)


def test_the_default_commands_are_the_seven_joints_and_the_hand(tmp_path: Path) -> None:
    config = FrankaConfig(ip=IP, model=fr3_like_model(tmp_path))
    assert list(config.joint_commands) == [*fr3_joint_names("arm"), f"arm/{FR3_FINGER_JOINT}"]
    without_hand = FrankaConfig(ip=IP, model=fr3_like_model(tmp_path), hand=False)
    assert list(without_hand.joint_commands) == fr3_joint_names("arm")


def test_reading_errors_show_in_status_without_stopping_the_module(
    rig: Callable[..., Rig],
) -> None:
    r = rig()

    def broken() -> Any:
        raise RuntimeError("UDP receive: Timeout")

    r.module._robot.read_once = broken
    wait_until(lambda: "reading failed" in str(r.module.status().last_error), timeout=2.0)
    wait_until(lambda: time.monotonic() - r.module._measured_at > 0.1, timeout=2.0)
    with pytest.raises(FrankaError, match="last read"):
        r.module.write({"arm/fr3_joint1/position": 0.01})


def test_a_target_where_the_arm_already_is_starts_no_session(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(fr3_joint1=0.005)
    assert "configure" not in r.franka.names()
    assert r.franka.targets() == []


def test_a_session_the_robot_ends_at_its_target_is_not_a_fault(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(fr3_joint1=0.05)
    r.franka.q = [0.05, *REST[1:]]
    r.franka.session_ended = "Unexpected reply to a Move command"
    wait_until(lambda: "cancel_motion" in r.franka.names(), timeout=2.0)
    assert r.module.status().last_error is None
    # Holding there starts nothing; the next new target starts a fresh session.
    r.franka.session_ended = None
    r.move(fr3_joint1=0.05)
    assert len(r.franka.sent("configure")) == 1
    r.move(fr3_joint1=0.1)
    assert len(r.franka.sent("configure")) == 2
    assert r.franka.targets()[-2:] == [
        pytest.approx([0.05, *REST[1:]]),
        pytest.approx([0.1, *REST[1:]]),
    ]


def test_a_session_ended_away_from_its_target_faults(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(fr3_joint1=0.05)
    r.franka.session_ended = "Unexpected reply to a Move command"
    wait_until(
        lambda: "ended away from its target" in str(r.module.status().last_error), timeout=2.0
    )


def test_a_session_ended_short_of_its_target_is_restarted_by_the_next_target(
    rig: Callable[..., Rig],
) -> None:
    r = rig(hold_on_short_end=True)
    r.move(fr3_joint1=0.05)
    r.franka.session_ended = "Unexpected reply to a Move command"
    wait_until(lambda: "cancel_motion" in r.franka.names(), timeout=2.0)
    assert r.module.status().last_error is None
    r.franka.session_ended = None
    r.move(fr3_joint1=0.05)  # the coordinator keeps sending it: a fresh session finishes the move
    assert len(r.franka.sent("configure")) == 2
    assert r.franka.targets()[-1] == pytest.approx([0.05, *REST[1:]])


def test_a_session_ending_short_again_and_again_faults(rig: Callable[..., Rig]) -> None:
    r = rig(hold_on_short_end=True)
    r.franka.session_ended = "Unexpected reply to a Move command"
    for ended in range(1, 4):
        r.move(fr3_joint1=0.05)
        wait_until(lambda n=ended: r.franka.names().count("cancel_motion") >= n, timeout=2.0)
        assert r.module.status().last_error is None
    r.move(fr3_joint1=0.05)
    wait_until(
        lambda: "ended away from its target" in str(r.module.status().last_error), timeout=2.0
    )


def test_an_arm_standing_still_short_of_its_target_faults(
    rig: Callable[..., Rig], monkeypatch: pytest.MonkeyPatch
) -> None:
    moving = robot_state

    def still(q: list[float], mode: Mode = Mode.Idle, errors: list[str] | None = None) -> Any:
        state = moving(q, mode, errors)
        state.dq = [0.0] * 7
        return state

    monkeypatch.setitem(globals(), "robot_state", still)
    r = rig()
    r.move(fr3_joint1=0.05)
    wait_until(lambda: "not following" in str(r.module.status().last_error), timeout=2.0)
    assert "cancel_motion" in r.franka.names()


def test_holding_a_homed_hand_open_moves_it_and_never_grasps(rig: Callable[..., Rig]) -> None:
    r = rig()
    # A Hand just homed in Desk reads a little past 80 mm.
    r.franka.width = 0.0801
    r.move(**{FR3_FINGER_JOINT: 0.04})
    wait_until(lambda: bool(r.franka.sent("move")), timeout=2.0)
    assert r.franka.sent("grasp") == []


def test_a_dropped_hand_link_is_reopened_and_the_target_resent(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.franka.width = 0.03
    r.franka.hand_down = True  # every Hand call fails like a dropped link
    r.move(**{FR3_FINGER_JOINT: 0.04})
    wait_until(lambda: "Net Exception" in str(r.module.status().last_error), timeout=2.0)
    r.franka.hand_down = False
    wait_until(lambda: bool(r.franka.sent("move")), timeout=3.0)
    assert r.franka.names().count("Gripper") >= 2


def test_a_trajectory_may_run_ahead_of_the_arm_but_not_jump(rig: Callable[..., Rig]) -> None:
    r = rig()
    # The arm stays at REST while targets advance 0.05 rad a step: each is close
    # to the one before, though they end up 0.3 rad ahead of the arm.
    for k in range(1, 7):
        r.move(fr3_joint2=REST[1] + 0.05 * k)
    assert len(r.franka.targets()) == 7  # the hold, then all six
    assert r.module.status().last_error is None
    # A jump from the previous target is refused.
    r.move(fr3_joint2=REST[1] + 0.6)
    assert "from the previous target" in str(r.module.status().last_error)
    # Refused, and the arm is told to hold where it is instead.
    assert len(r.franka.targets()) == 8
    assert r.franka.targets()[-1] == pytest.approx(REST)


def test_a_target_far_from_the_arm_is_refused_even_in_small_steps(rig: Callable[..., Rig]) -> None:
    r = rig()
    k = 0
    while "from where it is" not in str(r.module.status().last_error) and k < 40:
        k += 1
        r.move(fr3_joint2=REST[1] + 0.05 * k)
    # 1.0 s at 25% of 2.62 rad/s is 0.655 rad: the 14th step (0.70 rad) is refused.
    assert k == 14


def test_the_tool_contact_thresholds_are_set_from_the_config(rig: Callable[..., Rig]) -> None:
    r = rig()
    ((args, _),) = r.franka.sent("set_collision_behavior")
    assert args[2:] == ([20.0] * 6, [40.0] * 6)
    r2 = rig(contact_force=(15.0, 30.0))
    ((args2, _),) = r2.franka.sent("set_collision_behavior")
    assert args2[2:] == ([15.0] * 6, [30.0] * 6)
