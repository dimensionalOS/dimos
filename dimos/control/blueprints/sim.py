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

"""Simulation coordinator blueprints (MuJoCo).

Usage:
    dimos run coordinator-sim-xarm7
"""

from __future__ import annotations

from dimos.control.blueprints._hardware import sim_xarm7
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState

# Simulated XArm7 with trajectory task (MuJoCo viewer visible)
coordinator_sim_xarm7 = control_coordinator(
    hardware=[sim_xarm7(headless=False)],
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


__all__ = [
    "coordinator_sim_xarm7",
]
