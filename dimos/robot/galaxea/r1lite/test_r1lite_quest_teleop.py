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

from dimos.control.coordinator import ControlCoordinator
import dimos.core.module as module_mod
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import (
    R1LITE_LEFT_ARM_JOINTS,
    R1LITE_RIGHT_ARM_JOINTS,
    r1lite_quest_teleop,
    r1lite_quest_teleop_sim,
)
from dimos.robot.galaxea.r1lite.config import (
    R1LITE_LEFT_ARM_MODEL,
    R1LITE_RIGHT_ARM_MODEL,
)
from dimos.robot.galaxea.r1lite.connection import R1LITE_UPPER_BODY_JOINTS
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
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)[
        "tasks"
    ]


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


def test_ik_uses_per_side_captured_models() -> None:
    tasks = {t.name: t for t in _coordinator_tasks(r1lite_quest_teleop)}
    left_model = tasks["teleop_left_arm"].params["model_path"]
    right_model = tasks["teleop_right_arm"].params["model_path"]
    assert left_model == R1LITE_LEFT_ARM_MODEL
    assert right_model == R1LITE_RIGHT_ARM_MODEL
    assert left_model.exists()
    assert right_model.exists()


@pytest.mark.parametrize("model", [R1LITE_LEFT_ARM_MODEL, R1LITE_RIGHT_ARM_MODEL])
def test_arm_model_is_the_a1x_chain(model: Any) -> None:
    # The arms are A1X units; the A1Z flange model once shipped here had wrong
    # link lengths and limits and produced wrong motion on hardware. Pin the
    # captured chain: 6 revolute joints, the A1X axis pattern, the 300 mm
    # forearm link A1Z does not have, and the A1X joint1 limits.
    import xml.etree.ElementTree as ET

    root = ET.parse(model).getroot()
    joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
    assert len(joints) == 6
    axes = [j.find("axis").get("xyz") for j in joints]
    assert axes == ["0 0 1", "0 1 0", "0 1 0", "0 1 0", "0 0 1", "1 0 0"]
    j3_x = float(joints[2].find("origin").get("xyz").split()[0])
    assert j3_x == pytest.approx(-0.3)
    limit1 = joints[0].find("limit")
    assert float(limit1.get("upper")) == pytest.approx(2.8798)


def test_production_coordinator_tasks_unchanged() -> None:
    tasks = _coordinator_tasks(r1lite_coordinator)
    assert [(t.name, t.type, t.priority) for t in tasks] == [
        ("servo_r1lite", "servo", 10),
        ("vel_chassis", "velocity", 10),
    ]


def test_ik_tasks_configure_bounded_stepping() -> None:
    for blueprint in (r1lite_quest_teleop, r1lite_quest_teleop_sim):
        tasks = {t.name: t for t in _coordinator_tasks(blueprint)}
        for name in ("teleop_left_arm", "teleop_right_arm"):
            assert tasks[name].params["max_joint_delta_deg"] == 45.0
            assert tasks[name].params["max_step_deg_per_tick"] == 0.5
            assert tasks[name].params["max_target_offset_m"] == 0.02
            assert tasks[name].params["max_target_rot_deg"] == 15.0


def test_teleop_chases_through_folded_home_and_teleports() -> None:
    # Regression for the hardware sessions: the arms home folded, where small
    # cartesian targets need large joint motion, and pose-stream gaps teleport
    # the target far from the arm. A plain per-tick delta gate wedges tracking
    # permanently in both cases. With the chase window recentered on the EE
    # each tick, bounded steps, and the 45 degree backstop, tracking must
    # reach a far teleported target from folded with zero rejections.
    import numpy as np
    import pinocchio

    from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK

    ik = PinocchioIK.from_model_path(R1LITE_LEFT_ARM_MODEL, ee_joint_id=6)
    q0 = np.zeros(6)
    ee0 = ik.forward_kinematics(q0)
    target = pinocchio.SE3(ee0.rotation, ee0.translation + np.array([0.25, 0.05, 0.15]))

    q_sol, _, _ = ik.solve(target, q0)
    assert np.rad2deg(np.max(np.abs(q_sol - q0))) > 45.0  # the wedge, documented

    hard = np.deg2rad(45.0)
    step = np.deg2rad(0.5)
    win_t, win_r = 0.02, np.deg2rad(15.0)
    q = q0
    ticks = 0
    while np.linalg.norm(ik.forward_kinematics(q).translation - target.translation) >= 0.005:
        ticks += 1
        assert ticks <= 600, "teleported target not reached in 600 ticks"
        ee = ik.forward_kinematics(q)
        off = target.translation - ee.translation
        dist = np.linalg.norm(off)
        pos = target.translation if dist <= win_t else ee.translation + off * (win_t / dist)
        w = pinocchio.log3(ee.rotation.T @ target.rotation)
        angle = np.linalg.norm(w)
        rot = (
            target.rotation if angle <= win_r else ee.rotation @ pinocchio.exp3(w * (win_r / angle))
        )
        q_sol, _, _ = ik.solve(pinocchio.SE3(rot, pos), q)
        assert np.max(np.abs(q_sol - q)) < hard  # zero rejections along the chase
        q = q + np.clip(q_sol - q, -step, step)
    assert ticks < 500


def test_create_task_plumbs_step_limits() -> None:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.tasks.teleop_task.teleop_task import create_task

    cfg = TaskConfig(
        name="teleop_left_arm",
        type="teleop_ik",
        joint_names=R1LITE_LEFT_ARM_JOINTS,
        priority=20,
        params={
            "model_path": R1LITE_LEFT_ARM_MODEL,
            "ee_joint_id": 6,
            "hand": "left",
            "max_joint_delta_deg": 45.0,
            "max_step_deg_per_tick": 0.5,
            "max_target_offset_m": 0.02,
            "max_target_rot_deg": 15.0,
        },
    )
    task = create_task(cfg, hardware=None)
    assert task._config.max_joint_delta_deg == 45.0
    assert task._config.max_step_deg_per_tick == 0.5
    assert task._config.max_target_offset_m == 0.02
    assert task._config.max_target_rot_deg == 15.0


def test_web_server_binds_all_interfaces(monkeypatch: Any) -> None:
    # A loopback bind serves nothing off the robot; the headset is another
    # machine. Guard the listen_host wiring end to end without starting uvicorn.
    import dimos.teleop.quest.quest_teleop_module as qtm

    class _FakeServer:
        def __init__(self, port: int) -> None:
            self.port = port
            self.host = "127.0.0.1"
            self.app = types.SimpleNamespace(
                get=lambda *a, **k: (lambda fn: fn),
                websocket=lambda *a, **k: (lambda fn: fn),
                mount=lambda *a, **k: None,
            )

        def run(self, **kw: Any) -> None:
            pass

        def shutdown(self) -> None:
            pass

    monkeypatch.setattr(qtm, "RobotWebInterface", _FakeServer)
    m = _module()
    assert m.config.listen_host == "0.0.0.0"
    m._start_server()
    try:
        assert m._web_server.host == "0.0.0.0"
        assert m._web_server.port == 8443
    finally:
        m._stop_server()
