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

from dimos.control.blueprints._hardware import mock_arm, piper, xarm6, xarm7
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState

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
coordinator_mock = ControlCoordinator.blueprint(
    hardware=[mock_arm()],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm7 real hardware
coordinator_xarm7 = ControlCoordinator.blueprint(
    hardware=[xarm7()],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm6 real hardware
coordinator_xarm6 = ControlCoordinator.blueprint(
    hardware=[xarm6()],
    tasks=[
        TaskConfig(
            name="traj_xarm",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Piper arm (6-DOF, CAN bus)
coordinator_piper = ControlCoordinator.blueprint(
    hardware=[piper()],
    tasks=[
        TaskConfig(
            name="traj_piper",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
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
