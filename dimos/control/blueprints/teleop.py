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

"""Advanced control coordinator blueprints: servo, velocity, cartesian IK, and teleop IK.

Teleop blueprints switch between MuJoCo and real hardware via `--simulation`.

Usage:
    dimos run coordinator-teleop-xarm6              # Real XArm6 (TeleopIK)
    dimos --simulation run coordinator-teleop-xarm6 # XArm6 in MuJoCo sim
    dimos run coordinator-teleop-xarm7              # Real XArm7
    dimos --simulation run coordinator-teleop-xarm7 # XArm7 in MuJoCo sim
    dimos run coordinator-teleop-piper              # Real Piper
    dimos --simulation run coordinator-teleop-piper # Piper in MuJoCo sim
    dimos run coordinator-servo-xarm6               # Servo streaming (real-only)
    dimos run coordinator-velocity-xarm6            # Velocity streaming (real-only)
    dimos run coordinator-combined-xarm6            # Servo + velocity (real-only)
    dimos run coordinator-cartesian-ik-mock         # Cartesian IK (mock)
    dimos run coordinator-cartesian-ik-piper        # Cartesian IK (Piper, real-only)
    dimos run coordinator-teleop-dual               # TeleopIK dual arm (real-only)
"""

from __future__ import annotations

from dimos.control.blueprints._hardware import (
    PIPER_FK_MODEL,
    PIPER_SIM_PATH,
    XARM6_FK_MODEL,
    XARM6_SIM_PATH,
    XARM7_FK_MODEL,
    XARM7_SIM_PATH,
    manipulator,
    mock_arm,
    piper,
    xarm6,
    xarm7,
)
from dimos.control.components import make_gripper_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

_is_sim = global_config.simulation


def _mujoco_if_sim(sim_path: str, dof: int) -> tuple[Blueprint, ...]:
    if not _is_sim:
        return ()
    return (MujocoSimModule.blueprint(address=sim_path, headless=False, dof=dof),)


_xarm6_hw = manipulator(
    "arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
    gripper=True,
)
_piper_hw = manipulator(
    "arm",
    6,
    adapter_type="piper",
    address=global_config.can_port or "can0",
    gripper=True,
)

_xarm7_teleop_hw = xarm7("arm", gripper=True)
_xarm6_teleop_hw = xarm6("arm", gripper=True)
_piper_teleop_hw = piper("arm")

# XArm6 servo - streaming position control
coordinator_servo_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_hw],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=_xarm6_hw.joints,
            priority=10,
        ),
    ],
)

# XArm6 velocity control - streaming velocity for joystick
coordinator_velocity_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_hw],
    tasks=[
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_hw.joints,
            priority=10,
        ),
    ],
)

# XArm6 combined (servo + velocity tasks)
coordinator_combined_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_hw],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=_xarm6_hw.joints,
            priority=10,
        ),
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_hw.joints,
            priority=10,
        ),
    ],
)

# Mock 6-DOF arm with CartesianIK
_mock_6dof_hw = mock_arm("arm", 6)

coordinator_cartesian_ik_mock = ControlCoordinator.blueprint(
    hardware=[_mock_6dof_hw],
    tasks=[
        TaskConfig(
            name="cartesian_ik_arm",
            type="cartesian_ik",
            joint_names=_mock_6dof_hw.joints,
            priority=10,
            params={"model_path": PIPER_FK_MODEL, "ee_joint_id": 6},
        ),
    ],
)

# Piper arm with CartesianIK
coordinator_cartesian_ik_piper = ControlCoordinator.blueprint(
    hardware=[_piper_hw],
    tasks=[
        TaskConfig(
            name="cartesian_ik_arm",
            type="cartesian_ik",
            joint_names=_piper_hw.joints,
            priority=10,
            params={"model_path": PIPER_FK_MODEL, "ee_joint_id": 6},
        ),
    ],
)

# XArm7 with TeleopIK (real, or MuJoCo with --simulation)
coordinator_teleop_xarm7 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm7_teleop_hw],
        tasks=[
            TaskConfig(
                name="teleop_xarm",
                type="teleop_ik",
                joint_names=_xarm7_teleop_hw.joints,
                priority=10,
                params={
                    "model_path": XARM7_FK_MODEL,
                    "ee_joint_id": 7,
                    "hand": "right",
                    "gripper_joint": make_gripper_joints("arm")[0],
                    "gripper_open_pos": 0.85,
                    "gripper_closed_pos": 0.0,
                },
            ),
        ],
    ),
    *_mujoco_if_sim(str(XARM7_SIM_PATH), len(_xarm7_teleop_hw.joints)),
)

# XArm6 with TeleopIK (real, or MuJoCo with --simulation)
coordinator_teleop_xarm6 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_teleop_hw],
        tasks=[
            TaskConfig(
                name="teleop_xarm",
                type="teleop_ik",
                joint_names=_xarm6_teleop_hw.joints,
                priority=10,
                params={
                    "model_path": XARM6_FK_MODEL,
                    "ee_joint_id": 6,
                    "hand": "right",
                    "gripper_joint": make_gripper_joints("arm")[0],
                    "gripper_open_pos": 0.85,
                    "gripper_closed_pos": 0.0,
                },
            ),
        ],
    ),
    *_mujoco_if_sim(str(XARM6_SIM_PATH), len(_xarm6_teleop_hw.joints)),
)

# Piper with TeleopIK (real, or MuJoCo with --simulation)
coordinator_teleop_piper = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_piper_teleop_hw],
        tasks=[
            TaskConfig(
                name="teleop_piper",
                type="teleop_ik",
                joint_names=_piper_teleop_hw.joints,
                priority=10,
                params={
                    "model_path": PIPER_FK_MODEL,
                    "ee_joint_id": 6,
                    "hand": "left",
                    "gripper_joint": make_gripper_joints("arm")[0],
                    "gripper_open_pos": 0.0,
                    "gripper_closed_pos": 0.035,
                },
            ),
        ],
    ),
    *_mujoco_if_sim(str(PIPER_SIM_PATH), len(_piper_teleop_hw.joints)),
)

# Dual arm teleop: XArm6 + Piper with TeleopIK (real-only)
_xarm6_dual_hw = manipulator(
    "xarm_arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
    gripper=True,
)
_piper_dual_hw = manipulator(
    "piper_arm",
    6,
    adapter_type="piper",
    address=global_config.can_port,
    gripper=True,
)

coordinator_teleop_dual = ControlCoordinator.blueprint(
    hardware=[_xarm6_dual_hw, _piper_dual_hw],
    tasks=[
        TaskConfig(
            name="teleop_xarm",
            type="teleop_ik",
            joint_names=_xarm6_dual_hw.joints,
            priority=10,
            params={"model_path": XARM6_FK_MODEL, "ee_joint_id": 6, "hand": "left"},
        ),
        TaskConfig(
            name="teleop_piper",
            type="teleop_ik",
            joint_names=_piper_dual_hw.joints,
            priority=10,
            params={"model_path": PIPER_FK_MODEL, "ee_joint_id": 6, "hand": "right"},
        ),
    ],
)


__all__ = [
    "coordinator_cartesian_ik_mock",
    "coordinator_cartesian_ik_piper",
    "coordinator_combined_xarm6",
    "coordinator_servo_xarm6",
    "coordinator_teleop_dual",
    "coordinator_teleop_piper",
    "coordinator_teleop_xarm6",
    "coordinator_teleop_xarm7",
    "coordinator_velocity_xarm6",
]
