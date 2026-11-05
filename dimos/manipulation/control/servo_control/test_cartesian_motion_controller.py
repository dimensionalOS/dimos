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

from dimos_lcm.control_msgs import JointCommand
import pytest

from dimos.manipulation.control.servo_control.cartesian_motion_controller import (
    CartesianMotionController,
)
from dimos.msgs.sensor_msgs.JointState import JointState


@pytest.fixture
def controller(mocker):
    module = CartesianMotionController()
    mocker.patch(
        "dimos.manipulation.control.servo_control.cartesian_motion_controller.time.time",
        return_value=100.25,
    )
    mocker.patch.object(module._stop_event, "is_set", side_effect=[False, True])
    mocker.patch.object(module, "_call_fk", return_value=(0, [0, 0, 0, 0, 0, 0]))
    mocker.patch.object(module, "_call_ik", return_value=(0, [0.1, 0.2, 0.3]))
    mocker.patch.object(module.current_pose, "publish")
    mocker.patch.object(module.cartesian_velocity, "publish")
    module.set_target_pose([0.01, 0, 0], [0, 0, 0, 1])
    yield module
    module.stop()


@pytest.mark.parametrize("names", [[], [""], ["elbow", "elbow", "wrist"], ["elbow"]])
def test_cartesian_rejects_feedback_without_unique_names(controller, mocker, names):
    published = mocker.patch.object(controller.joint_position_command, "publish")
    controller._on_joint_state(JointState(name=names, position=[0, 0, 0]))
    controller._control_loop()
    published.assert_not_called()


def test_cartesian_preserves_feedback_joint_order_in_ros_command(controller, mocker):
    published = mocker.patch.object(controller.joint_position_command, "publish")
    controller._on_joint_state(JointState(name=["wrist", "elbow", "shoulder"], position=[0, 0, 0]))
    controller._control_loop()
    published.assert_called_once()
    command = JointCommand.lcm_decode(published.call_args.args[0].lcm_encode())
    assert command.interface_name == "position"
    assert list(command.joint_names) == ["wrist", "elbow", "shoulder"]
    assert list(command.values) == [0.1, 0.2, 0.3]
    assert (command.header.stamp.sec, command.header.stamp.nsec) == (100, 250000000)
