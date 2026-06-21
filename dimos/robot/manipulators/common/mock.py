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

"""Mock manipulator coordinator blueprints."""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig

_mock_hw = HardwareComponent(
    hardware_id="arm",
    hardware_type=HardwareType.MANIPULATOR,
    joints=make_joints("arm", 7),
    adapter_type="mock",
)

coordinator_mock = ControlCoordinator.blueprint(
    hardware=[_mock_hw],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=_mock_hw.joints,
            priority=10,
        )
    ],
)

_mock_left = HardwareComponent(
    hardware_id="left_arm",
    hardware_type=HardwareType.MANIPULATOR,
    joints=make_joints("left_arm", 7),
    adapter_type="mock",
)
_mock_right = HardwareComponent(
    hardware_id="right_arm",
    hardware_type=HardwareType.MANIPULATOR,
    joints=make_joints("right_arm", 6),
    adapter_type="mock",
)

coordinator_dual_mock = ControlCoordinator.blueprint(
    hardware=[_mock_left, _mock_right],
    tasks=[
        TaskConfig(name="traj_left", type="trajectory", joint_names=_mock_left.joints, priority=10),
        TaskConfig(
            name="traj_right", type="trajectory", joint_names=_mock_right.joints, priority=10
        ),
    ],
)
