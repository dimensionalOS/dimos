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

from dimos.manipulation.control.trajectory_controller.joint_trajectory_controller import (
    JointTrajectoryController,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@pytest.fixture
def controller():
    module = JointTrajectoryController()
    yield module
    module.stop()


@pytest.mark.parametrize("names", [[], [""], ["elbow", "elbow"], ["elbow"]])
def test_trajectory_rejects_missing_duplicate_or_mismatched_names(controller, names):
    trajectory = JointTrajectory(
        joint_names=names, points=[TrajectoryPoint(time_from_start=1, positions=[0.1, 0.2])]
    )
    assert not controller.execute_trajectory(trajectory)


def test_trajectory_publishes_named_ros_position_command(controller, mocker):
    mocker.patch(
        "dimos.manipulation.control.trajectory_controller.joint_trajectory_controller.time.time",
        return_value=100.25,
    )
    mocker.patch(
        "dimos.manipulation.control.trajectory_controller.joint_trajectory_controller.time.sleep"
    )
    mocker.patch.object(controller._stop_event, "is_set", side_effect=[False, True])
    published = mocker.patch.object(controller.joint_position_command, "publish")
    trajectory = JointTrajectory(
        joint_names=["wrist", "elbow"],
        points=[TrajectoryPoint(time_from_start=1, positions=[0.1, 0.2])],
    )
    assert controller.execute_trajectory(trajectory)
    controller._execution_loop()
    published.assert_called_once()
    command = JointCommand.lcm_decode(published.call_args.args[0].lcm_encode())
    assert command.interface_name == "position"
    assert list(command.joint_names) == ["wrist", "elbow"]
    assert list(command.values) == [0.1, 0.2]
    assert (command.header.stamp.sec, command.header.stamp.nsec) == (100, 250000000)
