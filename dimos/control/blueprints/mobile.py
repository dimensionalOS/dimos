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
    dimos run coordinator-flowbase-vis                   # FlowBase + Rerun visualization
"""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.ufactory import xarm7 as _catalog_xarm7
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.vis_module import vis_module

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
    address: str = "172.6.2.20:11323",
) -> HardwareComponent:
    """FlowBase holonomic platform via Portal RPC (3-DOF: vx, vy, wz)."""
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
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

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
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

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
).transports(
    {
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# FlowBase + Rerun + WebSocket dashboard (on-screen joystick at http://localhost:7779)
# rerun_open="both" → native Rerun GUI + dashboard served at /. Without this, / would
# redirect to the deprecated /command-center route, which returns "not built".
coordinator_flowbase_vis = autoconnect(
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
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={"rerun_open": "both"},
    ),
).transports(
    {
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        # Route the dashboard joystick's Twist to the coordinator's /cmd_vel topic.
        ("tele_cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Mock arm (7-DOF) + mock holonomic base (3-DOF)
_mock_arm_cfg = _catalog_xarm7(name="arm")

coordinator_mobile_manip_mock = ControlCoordinator.blueprint(
    hardware=[_mock_arm_cfg.to_hardware_component(), _mock_twist_base()],
    tasks=[
        _mock_arm_cfg.to_task_config(task_name="traj_arm"),
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)


__all__ = [
    "coordinator_flowbase",
    "coordinator_flowbase_keyboard_teleop",
    "coordinator_flowbase_vis",
    "coordinator_mobile_manip_mock",
    "coordinator_mock_twist_base",
]
