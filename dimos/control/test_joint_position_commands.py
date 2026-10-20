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

"""Complete intended targets without changing adapter dispatch."""

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
        publish_command_callback=receive,
        frame_id="robot",
    )


@pytest.mark.parametrize("whole_body", [False, True])
@pytest.mark.parametrize("mode", [ControlMode.POSITION, ControlMode.SERVO_POSITION])
def test_partial_targets_reuse_existing_hold_values(whole_body, mode):
    hw, adapter = hardware(whole_body)
    received = []
    tick = loop({"arm": hw}, received)
    write(tick, {"arm": ({"arm/gripper": 600.0, "arm/joint1": 1.2}, mode)}, 123.0)
    if whole_body:
        adapter.read_motor_states.return_value = [MotorState(q=9.0)] * 3
    else:
        adapter.read_joint_positions.return_value = [9.0, 9.0, 9.0]
    write(tick, {"arm": ({"arm/joint2": -0.4}, mode)}, 124.0)

    assert [(msg.name, msg.position, msg.ts, msg.frame_id) for msg in received] == [
        (JOINTS, [0.1, 1.2, 600.0], 123.0, "robot"),
        (JOINTS, [-0.4, 1.2, 600.0], 124.0, "robot"),
    ]
    assert received[0].velocity == []
    assert received[0].effort == []
    if whole_body:
        payloads = [
            [cmd.q for cmd in call.args[0]] for call in adapter.write_motor_commands.call_args_list
        ]
    else:
        payloads = [call.args[0] for call in adapter.write_joint_positions.call_args_list]
    assert payloads == [[0.1, 1.2, 600.0], [-0.4, 1.2, 600.0]]


def write(tick, commands, timestamp=123.0):
    with patch("dimos.control.tick_loop.time.time", return_value=timestamp):
        tick._write_all_hardware(commands)


@pytest.mark.parametrize("whole_body", [False, True])
@pytest.mark.parametrize("failure", ["reject", "raise"])
def test_failed_write_retains_intended_targets_without_claiming_execution(whole_body, failure):
    hw, adapter = hardware(whole_body)
    send = adapter.write_motor_commands if whole_body else adapter.write_joint_positions
    if failure == "reject":
        send.return_value = False
    else:
        send.side_effect = RuntimeError("backend failed")
    received = []
    tick = loop({"arm": hw}, received)
    write(tick, {"arm": ({"arm/joint1": 2.4}, ControlMode.SERVO_POSITION)})
    assert len(received) == 1
    assert received[0].position == [0.1, 2.4, 0.3]
    assert received[0].ts == 123.0
    assert [state.position for state in hw.read_state().values()] == [0.1, 0.2, 0.3]
    send.assert_called_once()


def test_failed_mode_switch_still_exposes_the_existing_intended_target():
    hw, adapter = hardware()
    adapter.set_control_mode.return_value = False
    received = []
    write(loop({"arm": hw}, received), {"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)})
    assert received[0].position == [0.1, 1.2, 0.3]
    adapter.write_joint_positions.assert_not_called()


def test_complete_snapshot_waits_for_initialization_then_uses_fixed_hardware_order():
    left, left_adapter = hardware()
    right_adapter = MagicMock(spec=ManipulatorAdapter)
    right_adapter.read_joint_positions.return_value = [0.4]
    right_adapter.set_control_mode.return_value = True
    right_adapter.write_joint_positions.return_value = True
    right = ConnectedHardware(
        right_adapter,
        HardwareComponent(
            hardware_id="right",
            hardware_type=HardwareType.MANIPULATOR,
            joints=["right/gripper"],
        ),
    )
    received = []
    tick = loop({"arm": left, "right": right}, received)
    write(tick, {})
    write(tick, {"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)})
    assert received == []
    right_adapter.read_joint_positions.assert_not_called()
    write(tick, {"right": ({"right/gripper": 0.7}, ControlMode.POSITION)})
    write(
        tick,
        {
            "right": ({"right/gripper": 0.8}, ControlMode.POSITION),
            "arm": ({"arm/joint2": -0.4}, ControlMode.POSITION),
        },
    )
    write(tick, {})
    assert [msg.name for msg in received] == [[*JOINTS, "right/gripper"]] * 3
    assert [msg.position for msg in received] == [
        [0.1, 1.2, 0.3, 0.7],
        [-0.4, 1.2, 0.3, 0.8],
        [-0.4, 1.2, 0.3, 0.8],
    ]
    assert left_adapter.write_joint_positions.call_count == 2
    assert right_adapter.write_joint_positions.call_count == 2


@pytest.mark.parametrize("whole_body", [False, True])
def test_unready_hardware_does_not_invent_or_update_hold_targets(whole_body):
    hw, adapter = hardware(whole_body)
    received = []
    tick = loop({"arm": hw}, received)
    with patch.object(hw, "ready_for_control", return_value=False):
        write(tick, {"arm": ({"arm/joint1": 9.0}, ControlMode.POSITION)})
    assert received == []
    assert hw.position_targets is None
    write(tick, {"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)})
    with patch.object(hw, "ready_for_control", return_value=False):
        write(tick, {"arm": ({"arm/joint1": 9.0}, ControlMode.POSITION)})
    assert [msg.position for msg in received] == [[0.1, 1.2, 0.3]] * 2
    send = adapter.write_motor_commands if whole_body else adapter.write_joint_positions
    send.assert_called_once()


@pytest.mark.parametrize("mode", [ControlMode.VELOCITY, ControlMode.TORQUE])
def test_other_modes_cannot_publish_old_position_targets(mode):
    hw, _ = hardware()
    received = []
    tick = loop({"arm": hw}, received)
    write(tick, {"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)})
    received.clear()
    write(tick, {"arm": ({"arm/joint1": 0.5}, mode)})
    write(tick, {})
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
    write(loop({"base": hw}, received), {"base": ({"base/vx": 0.5}, ControlMode.POSITION)})
    assert adapter.write_velocities.call_args.args[0] == [0.5]
    assert received == []


def test_initialization_failure_does_not_publish_a_partial_vector():
    hw, adapter = hardware()
    adapter.read_joint_positions.return_value = []
    received = []
    with patch("dimos.control.hardware_interface.time.sleep"):
        write(loop({"arm": hw}, received), {"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)})
    assert received == []
    adapter.write_joint_positions.assert_not_called()


@pytest.mark.parametrize("subscriber_raises", [False, True])
def test_tick_publishes_winning_targets_at_write_boundary_and_isolates_subscribers(
    subscriber_raises,
):
    hw, adapter = hardware()
    coordinator = ControlCoordinator()
    received = []
    measured = MagicMock()
    per_robot = MagicMock()
    transforms = MagicMock()
    winner = MagicMock()
    winner.name = "servo"
    winner.is_active.return_value = True
    winner.claim.return_value = ResourceClaim(joints=frozenset(JOINTS), priority=20)
    winner.compute.return_value = JointCommandOutput(
        joint_names=["arm/joint1"],
        positions=[1.5],
        mode=ControlMode.SERVO_POSITION,
    )
    winner.measured_frame_poses.return_value = {"tool": PoseStamped(position=(0.1, 0.2, 0.3))}
    loser = MagicMock()
    loser.name = "loser"
    loser.is_active.return_value = True
    loser.claim.return_value = ResourceClaim(joints=frozenset(JOINTS), priority=10)
    loser.compute.return_value = JointCommandOutput(joint_names=["arm/joint1"], positions=[99.0])
    loser.measured_frame_poses.return_value = {}

    def receive(msg: JointState) -> None:
        assert adapter.write_joint_positions.call_count == 1
        assert not tick._hardware_lock.locked()
        received.append(msg)
        if subscriber_raises:
            raise RuntimeError("recording subscriber failed")

    unsubscribe = coordinator.coordinator_joint_position_command.subscribe(receive)
    tick = TickLoop(
        tick_rate=100,
        hardware={"arm": hw},
        hardware_lock=threading.Lock(),
        tasks={"loser": loser, "servo": winner},
        task_lock=threading.Lock(),
        joint_to_hardware={name: "arm" for name in JOINTS},
        publish_callback=measured,
        publish_robot_callback=per_robot,
        publish_tf_callback=transforms,
        publish_command_callback=coordinator.coordinator_joint_position_command.publish,
    )
    try:
        with patch("dimos.control.tick_loop.time.time", side_effect=[100.0, 200.0]):
            with patch("dimos.control.tick_loop.logger.exception") as log_error:
                tick._tick()
        assert (
            received[0].position
            == adapter.write_joint_positions.call_args.args[0]
            == [0.1, 1.5, 0.3]
        )
        assert received[0].ts == 200.0
        assert measured.call_args.args[0].position == [0.1, 0.2, 0.3]
        assert measured.call_args.args[0].ts == 100.0
        measured.assert_called_once()
        per_robot.assert_called_once()
        assert per_robot.call_args.args[0] == "arm"
        transforms.assert_called_once()
        assert transforms.call_args.args[0].transforms[0].child_frame_id == "tool"
        assert log_error.call_count == int(subscriber_raises)
    finally:
        unsubscribe()
        coordinator.stop()


def test_coordinator_wires_command_output_separately_from_measured_state():
    coordinator = ControlCoordinator()
    with patch("dimos.control.coordinator.TickLoop") as tick_cls:
        try:
            coordinator.start()
            assert (
                tick_cls.call_args.kwargs["publish_command_callback"]
                == coordinator.coordinator_joint_position_command.publish
            )
            assert (
                tick_cls.call_args.kwargs["publish_callback"]
                == coordinator.coordinator_joint_state.publish
            )
        finally:
            coordinator.stop()
