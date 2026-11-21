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
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.flowbase.connection import (
    FLOWBASE_BASE_JOINTS,
    FlowBaseConfig,
    FlowBaseConnection,
)
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


class FlowBaseCoordinator(ControlCoordinator):
    """Body speeds out to one FlowBase named "base", and its readings in,
    through ``FlowBaseConnection``."""

    base_velocity_command: Out[JointState]
    base_joint_state: In[JointState]


# The connection module's ports onto the coordinator's, and Twist in on cmd_vel.
# The coordinator owns cmd_vel, so the connection's own Twist input is moved off it.
_FLOWBASE_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (FlowBaseConnection, "cmd_vel", "base_cmd_vel"),
    (FlowBaseConnection, "velocity_command", "base_velocity_command"),
    (FlowBaseCoordinator, "base_joint_state", "joint_state"),
    (FlowBaseCoordinator, "twist_command", "cmd_vel"),
]


def _flowbase_coordinator() -> MetaBlueprint:
    """A FlowBase behind a coordinator, driven by Twist on ``cmd_vel``."""
    return autoconnect(
        FlowBaseCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[FlowBaseConfig()],
            tasks=[
                TaskConfig(
                    name="vel_base",
                    type="twist_velocity",
                    joint_names=list(FLOWBASE_BASE_JOINTS),
                    params={"axes": ["linear.x", "linear.y", "angular.z"]},
                    priority=10,
                ),
            ],
        ),
        FlowBaseConnection.blueprint(),
    )


# Mock holonomic twist base (3-DOF: vx, vy, wz)
coordinator_mock_twist_base = ControlCoordinator.blueprint(
    hardware=[_mock_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="twist_velocity",
            joint_names=_base_joints,
            params={"axes": ["linear.x", "linear.y", "angular.z"]},
            priority=10,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase holonomic base over Portal RPC
coordinator_flowbase = _flowbase_coordinator().remappings(_FLOWBASE_REMAPS)

# FlowBase + WASD pygame keyboard teleop. Shift boosts to 0.8 m/s, the base's top speed.
coordinator_flowbase_keyboard_teleop = autoconnect(
    _flowbase_coordinator(),
    KeyboardTeleop.blueprint(boost_multiplier=1.6),
).remappings(_FLOWBASE_REMAPS)

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
            type="twist_velocity",
            joint_names=_base_joints,
            params={"axes": ["linear.x", "linear.y", "angular.z"]},
            priority=10,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])
