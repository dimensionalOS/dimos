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


def test_operator_yaw_180_flips_planar_deltas() -> None:
    m = _module(operator_yaw_deg=180.0)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.10, 0.05, 0.20])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx(-0.10)
    assert out.position.y == pytest.approx(-0.05)
    assert out.position.z == pytest.approx(0.20)


def test_local_rotation_uses_hand_frame_delta() -> None:
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    m = _module(local_rotation=True)
    m._is_engaged[Hand.LEFT] = True
    initial = PoseStamped(orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 1.0)))
    current = PoseStamped(
        orientation=initial.orientation * Quaternion.from_euler(Vector3(0.5, 0.0, 0.0))
    )
    m._initial_poses[Hand.LEFT] = initial
    m._current_poses[Hand.LEFT] = current
    out = m._get_output_pose(Hand.LEFT)
    expected = initial.orientation.inverse() * current.orientation
    got = out.orientation
    assert (
        abs(
            abs(
                sum(
                    a * b
                    for a, b in zip(
                        (got.x, got.y, got.z, got.w),
                        (expected.x, expected.y, expected.z, expected.w),
                        strict=False,
                    )
                )
            )
            - 1.0
        )
        < 1e-6
    )


def test_absolute_orientation_publishes_current_hand_attitude() -> None:
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    m = _module(absolute_orientation=True)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped(
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 1.0))
    )
    current = PoseStamped(orientation=Quaternion.from_euler(Vector3(0.5, 0.2, -0.3)))
    m._current_poses[Hand.LEFT] = current
    out = m._get_output_pose(Hand.LEFT)
    assert out.orientation == current.orientation


def _rotation_task(**config_overrides: Any) -> Any:
    from dimos.control.tasks.teleop_task.teleop_task import TeleopIKTask, TeleopIKTaskConfig

    defaults: dict[str, Any] = {
        "joint_names": R1LITE_LEFT_ARM_JOINTS,
        "model_path": R1LITE_LEFT_ARM_MODEL,
        "ee_joint_id": 6,
        "hand": "left",
        "solver": "dls",
    }
    defaults.update(config_overrides)
    return TeleopIKTask("rotation_test", TeleopIKTaskConfig(**defaults))


def test_absolute_rotation_does_not_ratchet_across_engages() -> None:
    # Delta modes re-anchor on the EE each engage, so orientation error left
    # at release accumulates over a session. Absolute mode captures the
    # hand-to-EE alignment once: after a drifted re-engage the same hand
    # attitude still maps to the original EE attitude, and a hand rotation
    # about a world axis rotates the target about that same axis.
    import numpy as np
    import pinocchio

    task = _rotation_task(rotation_frame="absolute")
    ee0 = pinocchio.exp3(np.array([0.0, 0.3, 0.1]))
    hand0 = pinocchio.exp3(np.array([0.2, 0.0, -0.4]))
    task._initial_ee_pose = pinocchio.SE3(ee0, np.zeros(3))

    first = task._target_rotation(hand0)
    assert np.allclose(first, ee0)

    spin = pinocchio.exp3(np.array([0.0, 0.0, 0.5]))
    assert np.allclose(task._target_rotation(spin @ hand0), spin @ ee0)

    drifted = pinocchio.exp3(np.array([1.2, -0.5, 0.9]))
    task._initial_ee_pose = pinocchio.SE3(drifted, np.zeros(3))
    assert np.allclose(task._target_rotation(hand0), ee0)


def test_rotation_deadband_holds_small_and_shortens_large() -> None:
    import numpy as np
    import pinocchio

    task = _rotation_task(rotation_frame="absolute", rotation_deadband_deg=4.0)
    ref = np.eye(3)
    assert np.allclose(task._apply_rotation_deadband(ref), ref)

    tremor = pinocchio.exp3(np.array([np.deg2rad(2.0), 0.0, 0.0]))
    assert np.allclose(task._apply_rotation_deadband(tremor), ref)

    turn = pinocchio.exp3(np.array([np.deg2rad(30.0), 0.0, 0.0]))
    out = task._apply_rotation_deadband(turn)
    moved = np.rad2deg(np.linalg.norm(pinocchio.log3(out)))
    assert moved == pytest.approx(26.0, abs=0.1)


def test_recorder_writes_replayable_frames(tmp_path: Any) -> None:
    # The recorder stores raw websocket frames verbatim; the replay script
    # must recover the exact bytes and relative timing.
    import base64
    import json

    record = tmp_path / "session.jsonl"
    m = _module(record_path=str(record))
    frames = [b"\x01\x02fingerprint-a", b"\x03\x04fingerprint-b"]
    for frame in frames:
        m._record_frame(frame)
    m._close_recorder()

    entries = [json.loads(line) for line in record.read_text().splitlines()]
    assert [base64.b64decode(e["data"]) for e in entries] == frames
    assert entries[1]["t"] >= entries[0]["t"]

    import sys

    sys.path.insert(0, "scripts/r1lite_test")
    try:
        from replay_quest_stream import load_frames
    finally:
        sys.path.pop(0)
    loaded = load_frames(record, 0.0, float("inf"))
    assert [data for _, data in loaded] == frames
    assert loaded[0][0] == 0.0


def test_recorder_off_by_default() -> None:
    m = _module()
    m._record_frame(b"frame")
    m._close_recorder()
    assert m._record_file is None
    assert m._record_count == 0


def test_hardware_blueprint_records_sessions() -> None:
    kwargs = next(
        atom.kwargs
        for atom in r1lite_quest_teleop.blueprints
        if atom.module is R1LiteQuestTeleopModule
    )
    assert kwargs["record_path"].startswith("logs/quest_record_")


def test_position_deadband_zeroes_small_deltas() -> None:
    m = _module(position_deadband_m=0.02)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.01, 0.01, 0.0])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == 0.0
    assert out.position.y == 0.0
    assert out.position.z == 0.0


def test_position_deadband_is_soft_and_applies_before_gain() -> None:
    # Soft: motion just past the threshold produces a small output, not a jump
    # of threshold size. Before gain: the band is in physical hand units.
    m = _module(position_deadband_m=0.02, motion_gain=2.0)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.05, 0.0, 0.0])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx((0.05 - 0.02) * 2.0)
    assert out.position.y == 0.0


def test_motion_gain_scales_position_delta_only() -> None:
    m = _module(motion_gain=1.3)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    moved = PoseStamped(position=[0.10, 0.0, 0.20])
    m._current_poses[Hand.LEFT] = moved
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx(0.13)
    assert out.position.z == pytest.approx(0.26)


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


def test_teleop_connection_raises_tracking_speed() -> None:
    from dimos.robot.galaxea.r1lite.connection import R1LiteConnection

    kwargs = next(
        atom.kwargs for atom in r1lite_quest_teleop.blueprints if atom.module is R1LiteConnection
    )
    assert kwargs["tracking_speed"] == 1.25


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
            assert tasks[name].params["max_step_deg_per_tick"] == 1.5
            assert tasks[name].params["max_target_offset_m"] == 0.08
            assert tasks[name].params["max_target_rot_deg"] == 20.0
            assert tasks[name].params["solver"] == "pink"
            assert tasks[name].params["rotation_frame"] == "local"
            assert tasks[name].params["orientation_weight"] == 1.0
            assert tasks[name].params["posture_weight"] == 0.05
            assert tasks[name].params["tool_offset_m"] == (0.17, 0.0, 0.0)


def test_module_orientation_output_pairs_with_task_rotation_frame() -> None:
    # The module publishes the orientation delta in the hand's own frame and
    # the task composes it in the gripper frame; mismatched pairing garbles
    # wrist rotation. Pin both blueprints to the paired configuration. The
    # absolute mode was hardware-tried and reverted (infeasible attitude
    # demands on a non-spherical wrist); delta local is the deliberate choice.
    for blueprint in (r1lite_quest_teleop, r1lite_quest_teleop_sim):
        kwargs = next(
            atom.kwargs for atom in blueprint.blueprints if atom.module is R1LiteQuestTeleopModule
        )
        assert kwargs["local_rotation"] is True
        assert "absolute_orientation" not in kwargs
        tasks = {t.name: t for t in _coordinator_tasks(blueprint)}
        for name in ("teleop_left_arm", "teleop_right_arm"):
            assert tasks[name].params["rotation_frame"] == "local"
            assert tasks[name].params["rotation_deadband_deg"] == 4.0


def test_hardware_blueprint_sets_position_deadband() -> None:
    kwargs = next(
        atom.kwargs
        for atom in r1lite_quest_teleop.blueprints
        if atom.module is R1LiteQuestTeleopModule
    )
    assert kwargs["position_deadband_m"] == 0.02


def test_teleop_task_controls_the_tool_point() -> None:
    # The gripper's grasp center sits well past the last joint; controlling
    # the joint-6 origin with orientation loose lets wrist swings sweep the
    # physical tool sideways. The task must anchor, window and solve for the
    # offset tool point.
    import numpy as np
    import pinocchio

    from dimos.control.tasks.teleop_task.teleop_task import TeleopIKTask, TeleopIKTaskConfig
    from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK

    offset = (0.17, 0.0, 0.0)
    task = TeleopIKTask(
        "tool_point_test",
        TeleopIKTaskConfig(
            joint_names=R1LITE_LEFT_ARM_JOINTS,
            model_path=R1LITE_LEFT_ARM_MODEL,
            ee_joint_id=6,
            hand="left",
            solver="dls",
            tool_offset_m=offset,
        ),
    )
    ik = PinocchioIK.from_model_path(R1LITE_LEFT_ARM_MODEL, ee_joint_id=6)
    q = np.array([0.3, -0.5, 0.4, 0.2, -0.3, 0.1])
    wrist = ik.forward_kinematics(q)
    tool = task._tool_fk(q)
    expected = wrist * pinocchio.SE3(np.eye(3), np.array(offset))
    assert np.allclose(tool.translation, expected.translation)
    assert np.allclose(tool.rotation, wrist.rotation)
    assert np.linalg.norm(tool.translation - wrist.translation) == pytest.approx(0.17)


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
            "joint_limit_margin_deg": 2.0,
        },
    )
    task = create_task(cfg, hardware=None)
    assert task._config.max_joint_delta_deg == 45.0
    assert task._config.max_step_deg_per_tick == 0.5
    assert task._config.max_target_offset_m == 0.02
    assert task._config.max_target_rot_deg == 15.0
    assert task._config.joint_limit_margin_deg == 2.0


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
