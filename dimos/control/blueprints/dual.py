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

"""Dual-arm coordinator blueprints with trajectory control.

Usage:
    dimos run coordinator-dual-mock      # Mock 7+6 DOF arms
    dimos run coordinator-dual-xarm      # XArm7 left + XArm6 right
    dimos run coordinator-piper-xarm     # XArm6 + Piper
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.piper import piper as _catalog_piper
from dimos.robot.catalog.ufactory import xarm6 as _catalog_xarm6, xarm7 as _catalog_xarm7

# Dual mock arms (7-DOF left, 6-DOF right)
_mock_left = _catalog_xarm7(name="left_arm")
_mock_right = _catalog_xarm6(name="right_arm", add_gripper=False)

coordinator_dual_mock = ControlCoordinator.blueprint(
    hardware=[_mock_left.to_hardware_component(), _mock_right.to_hardware_component()],
    tasks=[
        _mock_left.to_task_config(task_name="traj_left"),
        _mock_right.to_task_config(task_name="traj_right"),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Dual XArm (XArm7 left, XArm6 right)
_xarm7_left = _catalog_xarm7(name="left_arm", adapter_type="xarm", address=global_config.xarm7_ip)
_xarm6_right = _catalog_xarm6(
    name="right_arm", adapter_type="xarm", address=global_config.xarm6_ip, add_gripper=False
)

coordinator_dual_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm7_left.to_hardware_component(), _xarm6_right.to_hardware_component()],
    tasks=[
        _xarm7_left.to_task_config(task_name="traj_left"),
        _xarm6_right.to_task_config(task_name="traj_right"),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Dual arm (XArm6 + Piper)
_xarm6_dual = _catalog_xarm6(
    name="xarm_arm", adapter_type="xarm", address=global_config.xarm6_ip, add_gripper=False
)
_piper_dual = _catalog_piper(
    name="piper_arm", adapter_type="piper", address=global_config.can_port or "can0"
)

coordinator_piper_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm6_dual.to_hardware_component(), _piper_dual.to_hardware_component()],
    tasks=[
        _xarm6_dual.to_task_config(task_name="traj_xarm"),
        _piper_dual.to_task_config(task_name="traj_piper"),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


__all__ = [
    "coordinator_dual_mock",
    "coordinator_dual_xarm",
    "coordinator_piper_xarm",
]
