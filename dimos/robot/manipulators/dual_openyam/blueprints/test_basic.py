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

import numpy as np
import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.hardware.manipulators.spec import ControlMode
from dimos.message_utils import time as message_time
from dimos.messages.sensor_msgs.msg import JointState
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_openyam.config import (
    dual_openyam_control_model,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection


def test_deployed_connection_limits_follow_joint_names_not_model_order(mocker):
    deployed = dual_openyam_model_config(robot="arm")
    names = deployed.joint_names
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
            for name in [
                f"arm/{name}" for name in DualOpenYamConnection.robot.gripper_joints.values()
            ]
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    source = ConnectionSource(
        "arm", ["position_command"], ["joint_state"], dual_openyam_control_model(robot="arm"), 1
    )
    joint_names = [
        *deployed.joint_names,
        *[f"arm/{name}" for name in DualOpenYamConnection.robot.gripper_joints.values()],
    ]
    source.on_message(
        "joint_state",
        JointState(
            header=message_time.header_now(),
            name=joint_names,
            position=np.zeros(len(joint_names), dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
    )
    # Every name is tested against its own distinct range, independent of XML order.
    for name in names:
        source.command({name: bounds[name][1]}, ControlMode.POSITION)
        with pytest.raises(ValueError, match="outside"):
            source.command({name: bounds[name][1] + 0.1}, ControlMode.POSITION)
