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

"""Single-arm coordinator blueprints with trajectory control.

Usage:
    dimos run coordinator-mock           # Mock 7-DOF arm
    dimos run coordinator-xarm7          # XArm7 real hardware
    dimos run coordinator-xarm6          # XArm6 real hardware
    dimos run coordinator-piper          # Piper arm (CAN bus)
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.piper import piper as _catalog_piper
from dimos.robot.catalog.ufactory import xarm6 as _catalog_xarm6, xarm7 as _catalog_xarm7

# Minimal blueprint (no hardware, no tasks)
coordinator_basic = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Mock 7-DOF arm (for testing)
_mock_cfg = _catalog_xarm7(name="arm")

coordinator_mock = ControlCoordinator.blueprint(
    hardware=[_mock_cfg.to_hardware_component()],
    tasks=[_mock_cfg.to_task_config()],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm7 real hardware
_xarm7_cfg = _catalog_xarm7(name="arm", adapter_type="xarm", address=global_config.xarm7_ip)

coordinator_xarm7 = ControlCoordinator.blueprint(
    hardware=[_xarm7_cfg.to_hardware_component()],
    tasks=[_xarm7_cfg.to_task_config()],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm6 real hardware
_xarm6_cfg = _catalog_xarm6(
    name="arm", adapter_type="xarm", address=global_config.xarm6_ip, add_gripper=False
)

coordinator_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_cfg.to_hardware_component()],
    tasks=[_xarm6_cfg.to_task_config(task_name="traj_xarm")],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Piper arm (6-DOF, CAN bus)
_piper_cfg = _catalog_piper(
    name="arm", adapter_type="piper", address=global_config.can_port or "can0"
)

coordinator_piper = ControlCoordinator.blueprint(
    hardware=[_piper_cfg.to_hardware_component()],
    tasks=[_piper_cfg.to_task_config(task_name="traj_piper")],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


__all__ = [
    "coordinator_basic",
    "coordinator_mock",
    "coordinator_piper",
    "coordinator_xarm6",
    "coordinator_xarm7",
]
