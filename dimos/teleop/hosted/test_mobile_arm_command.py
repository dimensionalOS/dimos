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

"""Base drive and torso jog on the hosted mobile arm command plane."""

from __future__ import annotations

from collections.abc import Iterator
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.hosted.mobile_arm_command import MobileArmCommandModule
from dimos.teleop.quest.quest_types import Hand

_PORTS = (
    "left_controller_output",
    "right_controller_output",
    "teleop_buttons",
    "left_gripper_command",
    "right_gripper_command",
    "cmd_ack",
    "robot_state",
    "ee_twist_command",
    "gripper_command",
    "twist_command",
    "joint_command",
    "coordinator",
)


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> Iterator[MobileArmCommandModule]:
    def _fake_init(self: Any, **kwargs: Any) -> None:
        self.config = SimpleNamespace(
            control_loop_hz=50.0,
            cmd_stale_after_sec=0.5,
            enable_ui_scaling=False,
            input_timeout_s=1.0,
            base_linear_speed=0.3,
            base_angular_speed=0.4,
            boost_multiplier=2.0,
            base_deadzone=0.15,
            torso_deadzone=0.25,
            torso_speed=0.25,
            torso_fold_drops=[0.0, 0.2, 0.4],
            torso_fold_joints={"t1": [0.0, 0.8, 1.2], "t2": [0.0, -1.6, -2.4]},
            trust_operator_clock=False,
            engage_on_grip=True,
            recover_pose={"a/j1": 0.5, "a/j2": -1.0},
            recover_hz=5.0,
            require_deadman_to_drive=True,
            torso_requires_stick_click=True,
            grip_engage_threshold=0.5,
        )

    monkeypatch.setattr(Module, "__init__", _fake_init)
    module = MobileArmCommandModule()
    for port in _PORTS:
        setattr(module, port, MagicMock())
    module._cmd.start()
    yield module
    module._cmd.stop()


def _joy(
    frame_id: str,
    x: float = 0.0,
    y: float = 0.0,
    primary: bool = False,
    grip: float = 0.0,
    click: bool = False,
    secondary: bool = False,
) -> bytes:
    # axes: thumbstick x, thumbstick y, trigger, grip.
    # buttons: trigger, squeeze, touchpad, thumbstick, primary, secondary, menu.
    return Joy(
        frame_id=frame_id,
        axes=[x, y, 0.0, grip],
        buttons=[0, int(grip > 0.5), 0, int(click), int(primary), int(secondary), 0],
    ).lcm_encode()


def _pose_bytes(frame_id: str) -> bytes:
    return PoseStamped(ts=time.time(), frame_id=frame_id).lcm_encode()


def _tick(module: MobileArmCommandModule, *frames: bytes) -> None:
    """Deliver Joy frames, then run the control-loop tick that publishes.

    Base twist and torso target are published from _publish_button_state, at
    loop rate, because the operator page sends Joy in bursts rather than every
    frame.
    """
    for frame in frames:
        module._on_joy_bytes(frame)
    with module._lock:
        left = module._controllers.get(Hand.LEFT)
        right = module._controllers.get(Hand.RIGHT)
    module._publish_button_state(left, right)


def _last_twist(module: MobileArmCommandModule) -> Any:
    return module.twist_command.publish.call_args[0][0]


def _torso_drop(module: MobileArmCommandModule) -> float:
    """Recover the commanded drop from the joint goal, through the table."""
    goal = module.joint_command.publish.call_args[0][0]
    t1 = dict(zip(goal.name, goal.position, strict=True))["t1"]
    drops = module.config.torso_fold_drops
    values = module.config.torso_fold_joints["t1"]
    return float(np.interp(t1, values, drops))


def test_thumbsticks_drive_the_base(module: MobileArmCommandModule) -> None:
    """Right stick translates, left stick X yaws, both past the deadzone."""
    _tick(module, _joy("left", x=0.5, y=-1.0, grip=1.0), _joy("right", x=-1.0, grip=1.0))

    twist = _last_twist(module)
    assert twist.linear.x == pytest.approx(0.3)  # stick forward is negative y
    assert twist.linear.y == pytest.approx(-0.15)
    assert twist.angular.z == pytest.approx(0.4)


def test_holding_b_boosts_the_base(module: MobileArmCommandModule) -> None:
    """B on the right controller scales both base speeds while held, then lets go."""
    _tick(module, _joy("left", y=-1.0, grip=1.0), _joy("right", x=-1.0, secondary=True, grip=1.0))

    twist = _last_twist(module)
    assert twist.linear.x == pytest.approx(0.6)
    assert twist.angular.z == pytest.approx(0.8)

    _tick(module, _joy("left", y=-1.0, grip=1.0), _joy("right", x=-1.0, grip=1.0))

    twist = _last_twist(module)
    assert twist.linear.x == pytest.approx(0.3)
    assert twist.angular.z == pytest.approx(0.4)


def test_boost_is_still_behind_the_deadman(module: MobileArmCommandModule) -> None:
    """B without the grips is not a way to drive."""
    _tick(module, _joy("left", y=-1.0), _joy("right", secondary=True))

    twist = _last_twist(module)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_stick_drift_inside_the_deadzone_does_not_move_the_base(
    module: MobileArmCommandModule,
) -> None:
    _tick(module, _joy("left", x=0.1, y=0.1, grip=1.0), _joy("right", x=0.1, grip=1.0))

    twist = _last_twist(module)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_right_stick_y_jogs_the_torso_between_the_table_ends(
    module: MobileArmCommandModule,
) -> None:
    """A pure height axis: pull back lowers to the bottom of the table,
    push forward returns to full extension, and neither runs past."""
    # Nothing is published while the jog is not moving: streaming it would
    # restart the trajectory endlessly and fight hold-to-recover.
    _tick(module, _joy("right", y=0.0, click=True))
    module.joint_command.publish.assert_not_called()

    for _ in range(60):
        module._last_jog_t = time.monotonic() - 0.1
        _tick(module, _joy("right", y=1.0, click=True))
    assert _torso_drop(module) == pytest.approx(0.4)

    for _ in range(60):
        module._last_jog_t = time.monotonic() - 0.1
        _tick(module, _joy("right", y=-1.0, click=True))
    assert _torso_drop(module) == pytest.approx(0.0)


def test_torso_height_survives_releasing_the_deadman(
    module: MobileArmCommandModule,
) -> None:
    """Height is absolute, so letting go stops the torso where it is rather
    than dropping the robot back to full extension."""
    module._last_jog_t = time.monotonic() - 0.5
    _tick(module, _joy("left", grip=1.0), _joy("right", y=1.0, grip=1.0, click=True))
    lowered = _torso_drop(module)
    assert lowered > 0.0

    _tick(module, _joy("right", y=0.0, grip=0.0, click=True))
    assert _torso_drop(module) == pytest.approx(lowered)


def test_estop_stops_the_base_and_freezes_the_torso(module: MobileArmCommandModule) -> None:
    module._last_jog_t = time.monotonic() - 0.5
    _tick(module, _joy("right", y=1.0, click=True))
    held = _torso_drop(module)
    assert held > 0.0

    module._handle_estop(nonce="n1")
    module._last_jog_t = time.monotonic() - 0.5
    _tick(module, _joy("left", x=1.0, y=-1.0), _joy("right", y=1.0, click=True))

    twist = _last_twist(module)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert _torso_drop(module) == pytest.approx(held)


def test_side_grip_engages_and_frees_the_thumbstick(module: MobileArmCommandModule) -> None:
    """Holding a face button parks the thumb off the stick, which the operator
    needs for driving, so the grip is the engage control."""
    for side in ("left", "right"):
        module._on_pose_bytes(_pose_bytes(side))
    _tick(module, _joy("left", grip=1.0), _joy("right", grip=1.0))
    with module._lock:
        module._handle_engage()
    assert module._is_engaged[Hand.LEFT] and module._is_engaged[Hand.RIGHT]

    # The task's deadman reads the primary bits, so the grip must appear there.
    buttons = module.teleop_buttons.publish.call_args[0][0]
    assert buttons.left_primary and buttons.right_primary

    # The face button alone must not engage once the grip owns it.
    _tick(module, _joy("left", primary=True), _joy("right", primary=True))
    with module._lock:
        module._handle_engage()
    assert not module._is_engaged[Hand.LEFT]


def test_holding_a_walks_the_arms_back_to_the_configured_pose(
    module: MobileArmCommandModule,
) -> None:
    """A is free now that the grip engages, so it recovers posture on hold."""
    _tick(module, _joy("right", primary=True))
    goal = module.joint_command.publish.call_args[0][0]
    assert dict(zip(goal.name, goal.position, strict=True)) == {"a/j1": 0.5, "a/j2": -1.0}

    # Held, it re-issues rather than spamming every 50 Hz tick.
    before = module.joint_command.publish.call_count
    _tick(module, _joy("right", primary=True))
    assert module.joint_command.publish.call_count == before

    # Released, it stops.
    _tick(module, _joy("right", primary=False))
    module._last_recover_t = 0.0
    _tick(module, _joy("right", primary=False))
    assert module.joint_command.publish.call_count == before


def test_estop_blocks_recovery(module: MobileArmCommandModule) -> None:
    module._handle_estop(nonce="n1")
    _tick(module, _joy("right", primary=True))
    module.joint_command.publish.assert_not_called()


def test_driving_needs_the_same_grips_that_engage(module: MobileArmCommandModule) -> None:
    """A live chassis under a disengaged operator is the thing to avoid."""
    _tick(module, _joy("left", y=-1.0), _joy("right"))
    assert _last_twist(module).linear.x == pytest.approx(0.0)

    _tick(module, _joy("left", y=-1.0, grip=1.0), _joy("right", grip=1.0))
    assert _last_twist(module).linear.x == pytest.approx(0.3)


def test_torso_jog_needs_the_right_stick_clicked(module: MobileArmCommandModule) -> None:
    """Right-Y also yaws nothing by accident: the axis is armed by the click."""
    module._last_jog_t = time.monotonic() - 0.5
    _tick(module, _joy("right", y=1.0))
    module.joint_command.publish.assert_not_called()

    module._last_jog_t = time.monotonic() - 0.5
    _tick(module, _joy("right", y=1.0, click=True))
    assert _torso_drop(module) > 0.0


def test_clicking_the_right_stick_suppresses_yaw(module: MobileArmCommandModule) -> None:
    """The stick cannot steer and place height at the same time."""
    _tick(module, _joy("left", grip=1.0), _joy("right", x=1.0, grip=1.0))
    assert _last_twist(module).angular.z == pytest.approx(-0.4)

    _tick(module, _joy("left", grip=1.0), _joy("right", x=1.0, grip=1.0, click=True))
    assert _last_twist(module).angular.z == pytest.approx(0.0)
