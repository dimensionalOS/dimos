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

"""Basic OpenArm coordinator blueprints."""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import trajectory_task
from dimos.robot.manipulators.openarm.config import (
    LEFT_CAN,
    OPENARM_ADAPTER_KWARGS,
    OPENARM_DUAL_WHOLE_BODY_JOINTS,
    RIGHT_CAN,
    openarm_hardware,
    openarm_rs_hardware,
)


def openarm_task(hw: HardwareComponent, name: str | None = None) -> TaskConfig:
    return trajectory_task(hw, name=name)


mock_left = openarm_hardware(side="left")
mock_right = openarm_hardware(side="right")

coordinator_openarm_mock = ControlCoordinator.blueprint(
    hardware=[mock_left, mock_right],
    tasks=[
        openarm_task(mock_left),
        openarm_task(mock_right),
    ],
)

left_hw = openarm_hardware(
    side="left",
    address=LEFT_CAN,
    adapter_type="openarm",
    adapter_kwargs=OPENARM_ADAPTER_KWARGS,
)
right_hw = openarm_hardware(
    side="right",
    address=RIGHT_CAN,
    adapter_type="openarm",
    adapter_kwargs=OPENARM_ADAPTER_KWARGS,
)
openarm_rs_hw = openarm_rs_hardware()

coordinator_openarm_left = ControlCoordinator.blueprint(
    hardware=[left_hw],
    tasks=[openarm_task(left_hw)],
)

coordinator_openarm_right = ControlCoordinator.blueprint(
    hardware=[right_hw],
    tasks=[openarm_task(right_hw)],
)

coordinator_openarm_bimanual = ControlCoordinator.blueprint(
    hardware=[left_hw, right_hw],
    tasks=[
        openarm_task(left_hw),
        openarm_task(right_hw),
    ],
)

coordinator_openarm_rs = ControlCoordinator.blueprint(
    hardware=[openarm_rs_hw],
    tasks=[openarm_task(openarm_rs_hw)],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

openarm_dual_whole_body = ControlCoordinator.blueprint(
    hardware=[
        HardwareComponent(
            hardware_id="openarm",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=OPENARM_DUAL_WHOLE_BODY_JOINTS,
            adapter_type="openarm_dual",
            adapter_kwargs={
                "left_address": LEFT_CAN,
                "right_address": RIGHT_CAN,
                "gravity_comp": True,
            },
        )
    ],
    tasks=[
        TaskConfig(
            name="traj_openarm",
            type="trajectory",
            joint_names=OPENARM_DUAL_WHOLE_BODY_JOINTS,
        )
    ],
)
