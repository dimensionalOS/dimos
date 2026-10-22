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

"""Record the position-command subset without changing dispatch."""

from contextlib import ExitStack
import threading
from unittest.mock import MagicMock, patch

import pytest

from dimos.control.components import HardwareComponent, HardwareType, JointState as JointReading
from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ControlMode
from dimos.messages.sensor_msgs.msg import JointState as CdrJointState
from dimos.msgs.sensor_msgs.JointState import JointState


def make_loop(callback):
    hardware = MagicMock(spec=ConnectedHardware)
    hardware.joint_names = ["arm/joint2", "arm/joint1", "arm/gripper"]
    hardware.component = HardwareComponent(
        hardware_id="arm", hardware_type=HardwareType.MANIPULATOR
    )
    hardware.ready_for_control.return_value = True
    hardware.write_command.return_value = True
    hardware.read_state.return_value = {
        "arm/joint1": JointReading(position=-99.0, velocity=0.0, effort=0.0)
    }
    return TickLoop(
        tick_rate=100,
        hardware={"arm": hardware},
        hardware_lock=threading.Lock(),
        tasks={},
        task_lock=threading.Lock(),
        joint_to_hardware={},
        publish_command_callback=callback,
        frame_id="robot",
    ), hardware


@pytest.mark.parametrize("mode", [ControlMode.POSITION, ControlMode.SERVO_POSITION])
@pytest.mark.parametrize("result", [True, False, RuntimeError("write failed")])
def test_intended_subset_matches_dispatch_even_on_failure(mode, result):
    messages = []
    tick, hardware = make_loop(messages.append)
    values = {"arm/gripper": 600.0, "arm/joint1": 1.2}
    with patch("dimos.control.tick_loop.time") as clock:
        clock.time.return_value = 123.25

        def dispatch(*args):
            clock.time.assert_not_called()
            if isinstance(result, Exception):
                raise result
            return result

        hardware.write_command.side_effect = dispatch
        tick._write_all_hardware({"arm": (values, mode)})
        clock.time.assert_called_once_with()
    hardware.write_command.assert_called_once_with(values, mode)
    (message,) = messages
    assert message.name == ["arm/joint1", "arm/gripper"]
    assert message.position == [1.2, 600.0]
    assert message.velocity == message.effort == []
    assert (message.ts, message.frame_id) == (123.25, "robot")
    decoded = JointState.lcm_decode(message.lcm_encode())
    assert (decoded.name, decoded.position, decoded.ts) == (message.name, message.position, 123.25)


@pytest.mark.parametrize(
    ("mode", "ready", "kind", "values"),
    [
        (ControlMode.VELOCITY, True, HardwareType.MANIPULATOR, {"arm/joint1": 1.0}),
        (ControlMode.TORQUE, True, HardwareType.MANIPULATOR, {"arm/joint1": 1.0}),
        (ControlMode.POSITION, False, HardwareType.MANIPULATOR, {"arm/joint1": 1.0}),
        (ControlMode.POSITION, True, HardwareType.BASE, {"arm/joint1": 1.0}),
        (ControlMode.POSITION, True, HardwareType.MANIPULATOR, {}),
    ],
)
def test_non_position_unready_and_empty_updates_do_not_publish(mode, ready, kind, values):
    messages = []
    tick, hardware = make_loop(messages.append)
    hardware.ready_for_control.return_value = ready
    hardware.component.hardware_type = kind
    tick._write_all_hardware({})
    tick._write_all_hardware({"arm": (values, mode)})
    assert messages == []
    assert hardware.write_command.call_count == int(ready)


def test_subscriber_failure_does_not_suppress_measured_state():
    commands = []
    measured = []

    def fail(message):
        commands.append(message)
        raise RuntimeError("subscriber failed")

    tick, hardware = make_loop(fail)
    tick._publish_callback = measured.append
    with patch.object(
        tick,
        "_route_to_hardware",
        return_value={"arm": ({"arm/joint1": 1.2}, ControlMode.POSITION)},
    ):
        with patch("dimos.control.tick_loop.time.time", side_effect=[100.0, 200.0]):
            with patch("dimos.control.tick_loop.logger.exception") as log_error:
                tick._tick()
    assert commands[0].position == [1.2]
    assert commands[0].ts == 200.0
    assert measured[0].position == [-99.0]
    assert measured[0].ts == 100.0
    hardware.write_command.assert_called_once()
    log_error.assert_called_once()


def test_coordinator_wires_joint_state_command_output(mocker):
    coordinator = ControlCoordinator()
    tick_cls = mocker.patch("dimos.control.coordinator.TickLoop")
    publish = mocker.patch.object(coordinator.coordinator_joint_command, "publish")
    with ExitStack() as cleanup:
        cleanup.callback(coordinator.stop)
        coordinator.start()
        assert coordinator.coordinator_joint_command.type is CdrJointState
        callback = tick_cls.call_args.kwargs["publish_command_callback"]
        callback(JointState(ts=12.5, frame_id="arm", name=["joint1"], position=[1.2]))
        command = publish.call_args.args[0]
        assert type(command) is CdrJointState
        assert command.name == ["joint1"]
        assert command.position.tolist() == [1.2]
        assert (command.header.stamp.sec, command.header.stamp.nanosec) == (12, 500000000)
        assert command.header.frame_id == "arm"
