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

"""Deterministic unit tests for the R1 Lite quest teleop module and blueprints.

No headset, no ROS, no threads: the module is constructed with rpc disabled
and the event loop factory stubbed, and Out streams are replaced per test.
"""

from __future__ import annotations

import time
import types
from typing import Any

import pytest

import dimos.core.module as module_mod
from dimos.control.coordinator import ControlCoordinator
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import (
    R1LITE_LEFT_ARM_JOINTS,
    R1LITE_RIGHT_ARM_JOINTS,
    r1lite_quest_teleop,
    r1lite_quest_teleop_sim,
)
from dimos.robot.galaxea.r1lite.connection import R1LITE_UPPER_BODY_JOINTS
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.quest.quest_extensions import R1LiteQuestTeleopModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState, ThumbstickState


class _NoRpc(RPCSpec):
    """Module.__init__ treats a ValueError from the rpc factory as rpc disabled."""

    def __init__(self, **kw: Any) -> None:
        raise ValueError("rpc disabled for unit tests")


@pytest.fixture(autouse=True)
def _no_background_threads(monkeypatch: Any) -> None:
    monkeypatch.setattr(module_mod, "get_loop", lambda: (types.SimpleNamespace(), None))


class _FakeOut:
    def __init__(self) -> None:
        self.msgs: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.msgs.append(msg)


def _module(**config_kwargs: Any) -> R1LiteQuestTeleopModule:
    m = R1LiteQuestTeleopModule(rpc_transport=_NoRpc, **config_kwargs)
    for stream in (
        "cmd_vel",
        "gripper_left_command",
        "gripper_right_command",
        "teleop_buttons",
        "left_controller_output",
        "right_controller_output",
    ):
        setattr(m, stream, _FakeOut())
    return m


def _controller(
    *,
    is_left: bool = True,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    stick_press: bool = False,
    trigger: float = 0.0,
    primary: bool = False,
) -> QuestControllerState:
    return QuestControllerState(
        is_left=is_left,
        trigger=trigger,
        primary=primary,
        thumbstick_press=stick_press,
        thumbstick=ThumbstickState(x=stick_x, y=stick_y),
    )


def _mark_fresh(m: R1LiteQuestTeleopModule, *hands: Hand) -> float:
    now = time.monotonic()
    for hand in hands:
        m._joy_rx_ts[hand] = now
    return now


def _joy_bytes(
    frame_id: str,
    *,
    axes: list[float] | None = None,
    buttons: list[int] | None = None,
) -> bytes:
    return Joy(
        frame_id=frame_id,
        axes=axes if axes is not None else [0.0, 0.0, 0.0, 0.0],
        buttons=buttons if buttons is not None else [0] * 7,
    ).lcm_encode()


def test_left_stick_maps_to_linear_velocity() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT)
    twist = m._chassis_twist(_controller(stick_y=-1.0, stick_x=0.5), None, now)
    assert twist.linear.x == pytest.approx(0.2)
    assert twist.linear.y == pytest.approx(-0.1)
    assert twist.angular.z == 0.0


def test_right_stick_maps_to_yaw() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.RIGHT)
    twist = m._chassis_twist(None, _controller(is_left=False, stick_x=1.0), now)
    assert twist.linear.x == 0.0
    assert twist.angular.z == pytest.approx(-0.4)


def test_deadzone_suppresses_stick_drift() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT, Hand.RIGHT)
    twist = m._chassis_twist(
        _controller(stick_x=0.05, stick_y=-0.09),
        _controller(is_left=False, stick_x=0.09),
        now,
    )
    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0


def test_stale_controller_contributes_zero() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.RIGHT)
    m._joy_rx_ts[Hand.LEFT] = now - 10.0
    twist = m._chassis_twist(
        _controller(stick_y=-1.0),
        _controller(is_left=False, stick_x=1.0),
        now,
    )
    assert twist.linear.x == 0.0
    assert twist.angular.z == pytest.approx(-0.4)


def test_missing_controllers_give_zero_twist() -> None:
    m = _module()
    twist = m._chassis_twist(None, None, time.monotonic())
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_any_thumbstick_press_zeros_twist() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT, Hand.RIGHT)
    twist = m._chassis_twist(
        _controller(stick_y=-1.0),
        _controller(is_left=False, stick_x=1.0, stick_press=True),
        now,
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_gripper_command_maps_trigger_to_percent() -> None:
    m = _module()
    assert m._gripper_command(0.0).position[0] == pytest.approx(100.0)
    assert m._gripper_command(0.5).position[0] == pytest.approx(50.0)
    assert m._gripper_command(1.0).position[0] == pytest.approx(0.0)
    assert m._gripper_command(2.0).position[0] == pytest.approx(0.0)


def test_publish_tick_always_streams_twist_and_buttons() -> None:
    m = _module()
    m._publish_button_state(None, None)
    assert len(m.cmd_vel.msgs) == 1
    assert len(m.teleop_buttons.msgs) == 1
    assert m.gripper_left_command.msgs == []
    assert m.gripper_right_command.msgs == []


def test_gripper_streams_only_while_engaged_and_fresh() -> None:
    m = _module()
    left = _controller(trigger=1.0, primary=True)
    _mark_fresh(m, Hand.LEFT)

    m._publish_button_state(left, None)
    assert m.gripper_left_command.msgs == []

    m._is_engaged[Hand.LEFT] = True
    m._publish_button_state(left, None)
    assert len(m.gripper_left_command.msgs) == 1
    assert m.gripper_left_command.msgs[0].position[0] == pytest.approx(0.0)
    assert m.gripper_right_command.msgs == []

    m._joy_rx_ts[Hand.LEFT] = time.monotonic() - 10.0
    m._publish_button_state(left, None)
    assert len(m.gripper_left_command.msgs) == 1


def test_buttons_carry_packed_analog_triggers() -> None:
    m = _module()
    m._publish_button_state(
        _controller(trigger=1.0),
        _controller(is_left=False, trigger=0.5),
    )
    buttons = m.teleop_buttons.msgs[0]
    assert buttons.left_trigger_analog == pytest.approx(1.0)
    assert buttons.right_trigger_analog == pytest.approx(0.5, abs=0.01)


def test_on_joy_bytes_stores_state_and_stamps_receive_time() -> None:
    m = _module()
    m._on_joy_bytes(_joy_bytes("left", axes=[0.3, -0.7, 0.9, 0.0], buttons=[0, 0, 0, 0, 1, 0, 0]))
    left = m._controllers[Hand.LEFT]
    assert left is not None
    assert left.thumbstick.x == pytest.approx(0.3)
    assert left.thumbstick.y == pytest.approx(-0.7)
    assert left.trigger == pytest.approx(0.9)
    assert left.primary is True
    assert m._joy_rx_ts[Hand.LEFT] > 0.0
    assert m._joy_rx_ts[Hand.RIGHT] == 0.0


def test_on_joy_bytes_rejects_malformed_without_stamping() -> None:
    m = _module()
    m._on_joy_bytes(_joy_bytes("left", axes=[0.0, 0.0], buttons=[0] * 7))
    assert m._controllers[Hand.LEFT] is None
    assert m._joy_rx_ts[Hand.LEFT] == 0.0


def test_pose_frame_id_routes_to_task_name() -> None:
    m = _module(task_names={"left": "teleop_left_arm", "right": "teleop_right_arm"})
    m._publish_msg(Hand.LEFT, PoseStamped())
    m._publish_msg(Hand.RIGHT, PoseStamped())
    assert m.left_controller_output.msgs[0].frame_id == "teleop_left_arm"
    assert m.right_controller_output.msgs[0].frame_id == "teleop_right_arm"


def _coordinator_tasks(blueprint: Any) -> list[Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator
    )["tasks"]


@pytest.mark.parametrize(
    "blueprint",
    [
        pytest.param(r1lite_quest_teleop, id="hardware"),
        pytest.param(r1lite_quest_teleop_sim, id="sim"),
    ],
)
def test_quest_blueprint_task_set(blueprint: Any) -> None:
    tasks = {t.name: t for t in _coordinator_tasks(blueprint)}
    assert set(tasks) == {"servo_r1lite", "vel_chassis", "teleop_left_arm", "teleop_right_arm"}

    left = tasks["teleop_left_arm"]
    right = tasks["teleop_right_arm"]
    assert left.type == right.type == "teleop_ik"
    assert left.priority == right.priority == 20
    assert left.joint_names == R1LITE_LEFT_ARM_JOINTS
    assert right.joint_names == R1LITE_RIGHT_ARM_JOINTS
    assert left.params["hand"] == "left"
    assert right.params["hand"] == "right"

    servo = tasks["servo_r1lite"]
    assert servo.priority == 10
    assert servo.joint_names == R1LITE_UPPER_BODY_JOINTS


def test_arm_slices_match_connection_layout() -> None:
    assert R1LITE_LEFT_ARM_JOINTS == [f"r1lite/left_arm_joint{i}" for i in range(1, 7)]
    assert R1LITE_RIGHT_ARM_JOINTS == [f"r1lite/right_arm_joint{i}" for i in range(1, 7)]


def test_production_coordinator_tasks_unchanged() -> None:
    tasks = _coordinator_tasks(r1lite_coordinator)
    assert [(t.name, t.type, t.priority) for t in tasks] == [
        ("servo_r1lite", "servo", 10),
        ("vel_chassis", "velocity", 10),
    ]
