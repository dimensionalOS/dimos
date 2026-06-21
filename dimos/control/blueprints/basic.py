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

Robot-specific coordinator blueprints are owned by their robot packages and
re-exported here for compatibility.

Usage:
    dimos run coordinator-mock                    # Mock 7-DOF arm
    dimos run coordinator-xarm7                   # XArm7 real
    dimos --simulation run coordinator-xarm7      # XArm7 in MuJoCo
    dimos run coordinator-xarm6                   # XArm6 real
    dimos --simulation run coordinator-xarm6      # XArm6 in MuJoCo
    dimos run coordinator-piper                   # Piper real (CAN)
    dimos --simulation run coordinator-piper      # Piper in MuJoCo
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.robot.manipulators.piper.blueprints.basic import coordinator_piper
from dimos.robot.manipulators.xarm.blueprints.basic import (
    coordinator_xarm6,
    coordinator_xarm7,
)

# Minimal blueprint (no hardware, no tasks)
coordinator_basic = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
)

# Mock 7-DOF arm (for testing)
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

__all__ = [
    "coordinator_basic",
    "coordinator_mock",
    "coordinator_piper",
    "coordinator_xarm6",
    "coordinator_xarm7",
]
