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

from dimos.control.blueprints._hardware import manipulator, mock_arm
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.global_config import global_config

# Dual mock arms (7-DOF left, 6-DOF right)
_mock_left = mock_arm("left_arm", 7)
_mock_right = mock_arm("right_arm", 6)

coordinator_dual_mock = ControlCoordinator.blueprint(
    hardware=[_mock_left, _mock_right],
    tasks=[
        TaskConfig(name="traj_left", type="trajectory", joint_names=_mock_left.joints, priority=10),
        TaskConfig(
            name="traj_right", type="trajectory", joint_names=_mock_right.joints, priority=10
        ),
    ],
)

# Dual XArm (XArm7 left, XArm6 right)
_xarm7_left = manipulator(
    "left_arm",
    7,
    adapter_type="xarm",
    address=global_config.xarm7_ip,
)
_xarm6_right = manipulator(
    "right_arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
)

coordinator_dual_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm7_left, _xarm6_right],
    tasks=[
        TaskConfig(
            name="traj_left", type="trajectory", joint_names=_xarm7_left.joints, priority=10
        ),
        TaskConfig(
            name="traj_right", type="trajectory", joint_names=_xarm6_right.joints, priority=10
        ),
    ],
)

# Dual arm (XArm6 + Piper)
_xarm6_dual = manipulator(
    "xarm_arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
)
_piper_dual = manipulator(
    "piper_arm",
    6,
    adapter_type="piper",
    address=global_config.can_port or "can0",
    gripper=True,
)

coordinator_piper_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm6_dual, _piper_dual],
    tasks=[
        TaskConfig(
            name="traj_xarm", type="trajectory", joint_names=_xarm6_dual.joints, priority=10
        ),
        TaskConfig(
            name="traj_piper", type="trajectory", joint_names=_piper_dual.joints, priority=10
        ),
    ],
)


__all__ = [
    "coordinator_dual_mock",
    "coordinator_dual_xarm",
    "coordinator_piper_xarm",
]
