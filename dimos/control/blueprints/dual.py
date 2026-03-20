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

from dimos.control.blueprints._hardware import mock_arm, piper, xarm6, xarm7
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState

# Dual mock arms (7-DOF left, 6-DOF right)
coordinator_dual_mock = control_coordinator(
    hardware=[mock_arm("left_arm", 7), mock_arm("right_arm", 6)],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=[f"left_arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=[f"right_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Dual XArm (XArm7 left, XArm6 right)
coordinator_dual_xarm = control_coordinator(
    hardware=[xarm7("left_arm"), xarm6("right_arm")],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=[f"left_arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=[f"right_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Dual arm (XArm6 + Piper)
coordinator_piper_xarm = control_coordinator(
    hardware=[xarm6("xarm_arm"), piper("piper_arm")],
    tasks=[
        TaskConfig(
            name="traj_xarm",
            type="trajectory",
            joint_names=[f"xarm_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
        TaskConfig(
            name="traj_piper",
            type="trajectory",
            joint_names=[f"piper_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
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
