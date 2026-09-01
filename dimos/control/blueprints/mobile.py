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

"""Mobile manipulation coordinator blueprints.

Usage:
    dimos run coordinator-mock-twist-base                # Mock holonomic base
    dimos run coordinator-mobile-manip-mock              # Mock arm + base
    dimos run coordinator-flowbase                       # FlowBase holonomic base (Portal RPC)
    dimos run coordinator-flowbase-keyboard-teleop       # FlowBase + WASD pygame teleop
"""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_joints,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_base_joints = make_twist_base_joints("base")


def _mock_twist_base(hw_id: str = "base") -> HardwareComponent:
    """Mock holonomic twist base (3-DOF: vx, vy, wz)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="mock_twist_base",
    )


def _flowbase_twist_base(
    hw_id: str = "base",
    address: str | None = None,
) -> HardwareComponent:
    """FlowBase holonomic platform via Portal RPC (3-DOF: vx, vy, wz).

    Address defaults to ``FlowBaseAdapter.DEFAULT_ADDRESS`` when ``None``.
    """
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="flowbase",
        address=address,
    )


# Mock holonomic twist base (3-DOF: vx, vy, wz)
coordinator_mock_twist_base = ControlCoordinator.blueprint(
    hardware=[_mock_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase holonomic twist base (3-DOF: vx, vy, wz) over Portal RPC
coordinator_flowbase = ControlCoordinator.blueprint(
    hardware=[_flowbase_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase + WASD pygame keyboard teleop in a single blueprint
coordinator_flowbase_keyboard_teleop = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_flowbase_twist_base()],
        tasks=[
            TaskConfig(
                name="vel_base",
                type="velocity",
                joint_names=_base_joints,
                priority=10,
            ),
        ],
    ),
    KeyboardTeleop.blueprint(),
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# Mock arm (7-DOF) + mock holonomic base (3-DOF)
_mock_arm_hw = HardwareComponent(
    hardware_id="arm",
    hardware_type=HardwareType.MANIPULATOR,
    joints=make_joints("arm", 7),
    adapter_type="mock",
)

coordinator_mobile_manip_mock = ControlCoordinator.blueprint(
    hardware=[_mock_arm_hw, _mock_twist_base()],
    tasks=[
        joint_trajectory_task(_mock_arm_hw.joints),
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])
