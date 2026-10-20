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

"""Adapter-boundary position feedback; no hardware or worker processes."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock, patch

import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import (
    ConnectedHardware,
    ConnectedTwistBase,
    ConnectedWholeBody,
)
from dimos.control.task import JointCommandOutput, ResourceClaim
from dimos.control.tick_loop import TickLoop
from dimos.hardware.drive_trains.spec import TwistBaseAdapter
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter
from dimos.hardware.whole_body.spec import MotorState, WholeBodyAdapter
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

JOINTS = ["arm/joint2", "arm/joint1", "arm/gripper"]


def hardware(whole_body: bool = False) -> tuple[ConnectedHardware, MagicMock]:
    adapter = MagicMock(spec=WholeBodyAdapter if whole_body else ManipulatorAdapter)
    if whole_body:
        adapter.has_motor_states.return_value = True
        adapter.read_motor_states.return_value = [MotorState(q=q) for q in [0.1, 0.2, 0.3]]
        adapter.write_motor_commands.return_value = True
    else:
        adapter.read_joint_positions.return_value = [0.1, 0.2, 0.3]
        adapter.read_joint_velocities.return_value = [8.0] * 3
        adapter.read_joint_efforts.return_value = [9.0] * 3
        adapter.set_control_mode.return_value = True
        adapter.write_joint_positions.return_value = True
    component = HardwareComponent(
        hardware_id="arm",
        joints=JOINTS,
        hardware_type=HardwareType.WHOLE_BODY if whole_body else HardwareType.MANIPULATOR,
    )
    wrapper = ConnectedWholeBody if whole_body else ConnectedHardware
    return wrapper(adapter, component), adapter


def loop(hardware_by_id: dict[str, ConnectedHardware], received: list[JointState]) -> TickLoop:
    lock = threading.Lock()

    def receive(msg: JointState) -> None:
        assert not lock.locked()
        received.append(msg)

    return TickLoop(
        tick_rate=100,
        hardware=hardware_by_id,
        hardware_lock=lock,
        tasks={},
        task_lock=threading.Lock(),
        joint_to_hardware={name: "arm" for name in JOINTS},
        publish_applied_position_callback=receive,
        frame_id="robot",
    )


@pytest.mark.parametrize("whole_body", [False, True])
@pytest.mark.parametrize("mode", [ControlMode.POSITION, ControlMode.SERVO_POSITION])
def test_full_dispatch_payload_in_adapter_order_with_partial_updates(whole_body, mode):
    hw, adapter = hardware(whole_body)
    received = []
    tick = loop({"arm": hw}, received)
    tick._write_all_hardware({"arm": ({"arm/gripper": 0.6, "arm/joint1": 1.2}, mode)}, 123.0)
    tick._write_all_hardware({"arm": ({"arm/joint2": -0.4}, mode)}, 124.0)

    assert [(msg.name, msg.position, msg.ts, msg.frame_id) for msg in received] == [
        (JOINTS, [0.1, 1.2, 0.6], 123.0, "robot"),
        (JOINTS, [-0.4, 1.2, 0.6], 124.0, "robot"),
    ]
    assert received[0].velocity == []
    assert received[0].effort == []
    if whole_body:
        assert [
            [cmd.q for cmd in call.args[0]] for call in adapter.write_motor_commands.call_args_list
        ] == [
            [0.1, 1.2, 0.6],
            [-0.4, 1.2, 0.6],
        ]
    else:
        assert [call.args[0] for call in adapter.write_joint_positions.call_args_list] == [
            [0.1, 1.2, 0.6],
            [-0.4, 1.2, 0.6],
        ]


@pytest.mark.parametrize("whole_body", [False, True])
@pytest.mark.parametrize("failure", ["reject", "raise", "not_ready", "mode_switch"])
def test_failed_dispatch_does_not_publish_stale_payload(whole_body, failure):
    hw, adapter = hardware(whole_body)
    received = []
    tick = loop({"arm": hw}, received)
    tick._write_all_hardware({"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)}, 123.0)
    received.clear()
    write = adapter.write_motor_commands if whole_body else adapter.write_joint_positions
    if failure == "reject":
        write.return_value = False
    elif failure == "raise":
        write.side_effect = RuntimeError("backend failed")
    elif failure == "not_ready":
        hw.ready_for_control = lambda: False
    elif failure == "mode_switch" and not whole_body:
        adapter.set_control_mode.return_value = False
        # Whole-body wrappers do not switch modes. Unsupported velocity fails.
    mode = (
        ControlMode.VELOCITY
        if whole_body and failure == "mode_switch"
        else ControlMode.SERVO_POSITION
    )
    tick._write_all_hardware({"arm": ({"arm/joint1": 2.4}, mode)}, 124.0)
    assert received == []


def test_velocity_and_empty_ticks_do_not_publish_positions():
    hw, adapter = hardware()
    received = []
    tick = loop({"arm": hw}, received)
    tick._write_all_hardware({}, 123.0)
    tick._write_all_hardware({"missing": ({"missing/joint": 1.0}, ControlMode.POSITION)}, 123.0)
    tick._write_all_hardware({"arm": ({"arm/joint1": 1.0}, ControlMode.VELOCITY)}, 124.0)
    assert adapter.write_joint_velocities.call_args.args[0] == [0.0, 1.0, 0.0]
    assert received == []


def test_twist_base_never_labels_velocity_as_position():
    adapter = MagicMock(spec=TwistBaseAdapter)
    adapter.write_velocities.return_value = True
    hw = ConnectedTwistBase(
        adapter,
        HardwareComponent(
            hardware_id="base",
            hardware_type=HardwareType.BASE,
            joints=["base/vx"],
        ),
    )
    received = []
    loop({"base": hw}, received)._write_all_hardware(
        {"base": ({"base/vx": 0.5}, ControlMode.POSITION)},
        123.0,
    )
    assert adapter.write_velocities.call_args.args[0] == [0.5]
    assert received == []


def test_tick_uses_winning_task_output_and_same_state_timestamp():
    hw, adapter = hardware()
    received = []
    measured = []
    tick = loop({"arm": hw}, received)
    tick._publish_callback = measured.append
    task = MagicMock()
    task.name = "winner"
    task.is_active.return_value = True
    task.claim.return_value = ResourceClaim(joints=frozenset(JOINTS), priority=20)
    task.compute.return_value = JointCommandOutput(
        joint_names=["arm/joint1"],
        positions=[1.5],
        mode=ControlMode.SERVO_POSITION,
    )
    loser = MagicMock()
    loser.name = "loser"
    loser.is_active.return_value = True
    loser.claim.return_value = ResourceClaim(joints=frozenset(JOINTS), priority=10)
    loser.compute.return_value = JointCommandOutput(joint_names=["arm/joint1"], positions=[99.0])
    tick._tasks = {"loser": loser, "winner": task}
    with patch("dimos.control.tick_loop.time.time", return_value=456.0):
        tick._tick()
    assert received[0].ts == measured[0].ts == 456.0
    assert measured[0].position == [0.1, 0.2, 0.3]
    assert received[0].position == [0.1, 1.5, 0.3]
    assert adapter.write_joint_positions.call_args.args[0] == received[0].position


def test_partial_backend_failure_keeps_only_successful_hardware():
    good, _ = hardware()
    bad, adapter = hardware()
    adapter.write_joint_positions.return_value = False
    received = []
    tick = loop({"arm": good, "failed": bad}, received)
    tick._write_all_hardware(
        {
            "failed": ({"arm/joint1": 9.0}, ControlMode.POSITION),
            "arm": ({"arm/joint1": 1.2}, ControlMode.POSITION),
        },
        123.0,
    )
    assert len(received) == 1
    assert received[0].name == JOINTS
    assert received[0].position == [0.1, 1.2, 0.3]


def test_coordinator_wires_output_to_tick_loop():
    coordinator = ControlCoordinator(publish_joint_state=False)
    with patch("dimos.control.coordinator.TickLoop") as tick_cls:
        try:
            coordinator.start()
            assert (
                tick_cls.call_args.kwargs["publish_applied_position_callback"]
                == coordinator.applied_joint_position_command.publish
            )
        finally:
            coordinator.stop()


def test_command_subscriber_failure_preserves_measured_outputs():
    hw, adapter = hardware()
    coordinator = ControlCoordinator()
    measured = MagicMock()
    per_robot = MagicMock()
    transforms = MagicMock()
    task = MagicMock()
    task.name = "servo"
    task.is_active.return_value = True
    task.claim.return_value = ResourceClaim(joints=frozenset(JOINTS))
    task.compute.return_value = JointCommandOutput(
        joint_names=["arm/joint1"],
        positions=[1.5],
        mode=ControlMode.SERVO_POSITION,
    )
    task.measured_frame_poses.return_value = {"tool": PoseStamped(position=(0.1, 0.2, 0.3))}

    def fail(msg: JointState) -> None:
        assert msg.position == [0.1, 1.5, 0.3]
        raise RuntimeError("recording subscriber failed")

    unsubscribe = coordinator.applied_joint_position_command.subscribe(fail)
    tick = TickLoop(
        tick_rate=100,
        hardware={"arm": hw},
        hardware_lock=threading.Lock(),
        tasks={"servo": task},
        task_lock=threading.Lock(),
        joint_to_hardware={name: "arm" for name in JOINTS},
        publish_callback=measured,
        publish_robot_callback=per_robot,
        publish_tf_callback=transforms,
        publish_applied_position_callback=coordinator.applied_joint_position_command.publish,
    )
    try:
        with patch("dimos.control.tick_loop.logger.exception") as log_error:
            with patch("dimos.control.tick_loop.time.time", return_value=456.0):
                tick._tick()
        assert adapter.write_joint_positions.call_args.args[0] == [0.1, 1.5, 0.3]
        measured.assert_called_once()
        assert measured.call_args.args[0].position == [0.1, 0.2, 0.3]
        assert measured.call_args.args[0].ts == 456.0
        per_robot.assert_called_once()
        assert per_robot.call_args.args[0] == "arm"
        assert per_robot.call_args.args[1].position == [0.1, 0.2, 0.3]
        transforms.assert_called_once()
        assert transforms.call_args.args[0].transforms[0].child_frame_id == "tool"
        log_error.assert_called_once()
    finally:
        unsubscribe()
        coordinator.stop()
