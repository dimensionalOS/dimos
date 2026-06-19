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

Each arm blueprint switches between real hardware and MuJoCo via `--simulation`.

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

from dimos.control.blueprints._hardware import (
    PIPER_SIM_PATH,
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    mock_arm,
    piper,
    xarm6,
    xarm7,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

_is_sim = global_config.simulation


def _mujoco_if_sim(sim_path: str, dof: int) -> tuple[Blueprint, ...]:
    if not _is_sim:
        return ()
    return (MujocoSimModule.blueprint(address=sim_path, headless=False, dof=dof),)


# Minimal blueprint (no hardware, no tasks)
coordinator_basic = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
)

# Mock 7-DOF arm (for testing)
_mock_hw = mock_arm("arm", 7)

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

# XArm7 (real, or MuJoCo with --simulation)
_xarm7_hw = xarm7("arm")

coordinator_xarm7 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm7_hw],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=_xarm7_hw.joints,
                priority=10,
            )
        ],
    ),
    *_mujoco_if_sim(str(XARM7_SIM_PATH), len(_xarm7_hw.joints)),
)

# XArm6 (real, or MuJoCo with --simulation)
_xarm6_hw = xarm6("arm", gripper=True)

coordinator_xarm6 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_hw],
        tasks=[
            TaskConfig(
                name="traj_xarm",
                type="trajectory",
                joint_names=_xarm6_hw.joints,
                priority=10,
            )
        ],
    ),
    *_mujoco_if_sim(str(XARM6_SIM_PATH), len(_xarm6_hw.joints)),
)

# Piper 6-DOF (CAN bus, or MuJoCo with --simulation)
_piper_hw = piper("arm")

coordinator_piper = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_piper_hw],
        tasks=[
            TaskConfig(
                name="traj_piper",
                type="trajectory",
                joint_names=_piper_hw.joints,
                priority=10,
            )
        ],
    ),
    *_mujoco_if_sim(str(PIPER_SIM_PATH), len(_piper_hw.joints)),
)
