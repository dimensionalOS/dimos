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


from pathlib import Path

import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection


def test_deployed_connection_limits_follow_joint_names_not_model_order(mocker):
    deployed = DualOpenYamConnection.models()
    names = deployed.arm_joints
    bounds = {name: (-float(i + 1), float(i + 2)) for i, name in enumerate(names)}
    xml = (
        '<robot name="test"><link name="base"/>'
        + "".join(
            f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
            f'<parent link="base"/><child link="{name}_link"/>'
            f'<limit lower="{bounds[name][0]}" upper="{bounds[name][1]}"/></joint>'
            for name in reversed(names)
        )
        + "".join(
            f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
            f'<parent link="base"/><child link="{name}_link"/>'
            '<limit lower="0" upper="1"/></joint>'
            for name in deployed.gripper_joints
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    source = ConnectionSource("arm", ["position_command"], ["joint_state"], deployed.control, 1)
    source.on_message(
        "joint_state",
        JointState(name=list(deployed.joints), position=[0.0] * len(deployed.joints)),
    )
    # Every name is tested against its own distinct range, independent of XML order.
    for name in names:
        source.command({name: bounds[name][1]}, ControlMode.POSITION)
        with pytest.raises(ValueError, match="outside"):
            source.command({name: bounds[name][1] + 0.1}, ControlMode.POSITION)
