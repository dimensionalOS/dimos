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
    dimos run coordinator-sim-xarm7              # Trajectory control only
    dimos run coordinator-sim-teleop-xarm7       # Coordinator + teleop_ik (no quest module)
"""

from __future__ import annotations

from dimos.control.blueprints._hardware import XARM7_MODEL_PATH, sim_xarm7
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons

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

# Simulated XArm7 with Quest TeleopIK (coordinator side only)
coordinator_sim_teleop_xarm7 = control_coordinator(
    hardware=[sim_xarm7(headless=False)],
    tasks=[
        TaskConfig(
            name="teleop_xarm",
            type="teleop_ik",
            joint_names=[f"arm_joint{i + 1}" for i in range(7)],
            priority=10,
            model_path=XARM7_MODEL_PATH,
            ee_joint_id=7,
            hand="right",
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


__all__ = [
    "coordinator_sim_teleop_xarm7",
    "coordinator_sim_xarm7",
]
