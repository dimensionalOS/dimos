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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.piper import PIPER_SIM_PATH, piper as _catalog_piper
from dimos.robot.catalog.ufactory import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    xarm6 as _catalog_xarm6,
    xarm7 as _catalog_xarm7,
)
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

# XArm7 (real, or MuJoCo with --simulation)
_xarm7_cfg = _catalog_xarm7(
    name="arm",
    adapter_type="sim_mujoco" if _is_sim else "xarm",
    address=str(XARM7_SIM_PATH) if _is_sim else global_config.xarm7_ip,
)

coordinator_xarm7 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm7_cfg.to_hardware_component()],
        tasks=[_xarm7_cfg.to_task_config()],
    ),
    *_mujoco_if_sim(str(XARM7_SIM_PATH), _xarm7_cfg.dof),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm6 (real, or MuJoCo with --simulation)
_xarm6_cfg = _catalog_xarm6(
    name="arm",
    adapter_type="sim_mujoco" if _is_sim else "xarm",
    address=str(XARM6_SIM_PATH) if _is_sim else global_config.xarm6_ip,
)

coordinator_xarm6 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_cfg.to_hardware_component()],
        tasks=[_xarm6_cfg.to_task_config(task_name="traj_xarm")],
    ),
    *_mujoco_if_sim(str(XARM6_SIM_PATH), _xarm6_cfg.dof),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Piper 6-DOF (CAN bus, or MuJoCo with --simulation)
_piper_cfg = _catalog_piper(
    name="arm",
    adapter_type="sim_mujoco" if _is_sim else "piper",
    address=str(PIPER_SIM_PATH) if _is_sim else (global_config.can_port or "can0"),
)

coordinator_piper = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_piper_cfg.to_hardware_component()],
        tasks=[_piper_cfg.to_task_config(task_name="traj_piper")],
    ),
    *_mujoco_if_sim(str(PIPER_SIM_PATH), _piper_cfg.dof),
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
