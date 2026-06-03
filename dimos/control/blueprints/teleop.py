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

from dimos.control.components import make_gripper_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.piper import PIPER_FK_MODEL, PIPER_SIM_PATH, piper as _catalog_piper
from dimos.robot.catalog.ufactory import (
    XARM6_FK_MODEL,
    XARM6_SIM_PATH,
    XARM7_FK_MODEL,
    XARM7_SIM_PATH,
    xarm6 as _catalog_xarm6,
    xarm7 as _catalog_xarm7,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.quest.quest_types import Buttons

_is_sim = global_config.simulation


def _mujoco_if_sim(sim_path: str, dof: int) -> tuple[Blueprint, ...]:
    if not _is_sim:
        return ()
    return (MujocoSimModule.blueprint(address=sim_path, headless=False, dof=dof),)


_xarm6_cfg = _catalog_xarm6(name="arm", adapter_type="xarm", address=global_config.xarm6_ip)
_piper_cfg = _catalog_piper(
    name="arm", adapter_type="piper", address=global_config.can_port or "can0"
)

_xarm7_teleop_cfg = _catalog_xarm7(
    name="arm",
    adapter_type="sim_mujoco" if _is_sim else "xarm",
    address=str(XARM7_SIM_PATH) if _is_sim else global_config.xarm7_ip,
    add_gripper=True,
)
_xarm6_teleop_cfg = _catalog_xarm6(
    name="arm",
    adapter_type="sim_mujoco" if _is_sim else "xarm",
    address=str(XARM6_SIM_PATH) if _is_sim else global_config.xarm6_ip,
)
_piper_teleop_cfg = _catalog_piper(
    name="arm",
    adapter_type="sim_mujoco" if _is_sim else "piper",
    address=str(PIPER_SIM_PATH) if _is_sim else (global_config.can_port or "can0"),
)

# XArm6 servo - streaming position control
coordinator_servo_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_cfg.to_hardware_component()],
    tasks=[
        _xarm6_cfg.to_task_config(task_type="servo", task_name="servo_arm"),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/teleop/joint_command", JointState),
    }
)

# XArm6 velocity control - streaming velocity for joystick
coordinator_velocity_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_cfg.to_hardware_component()],
    tasks=[
        _xarm6_cfg.to_task_config(task_type="velocity", task_name="velocity_arm"),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/joystick/joint_command", JointState),
    }
)

# XArm6 combined (servo + velocity tasks)
coordinator_combined_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_cfg.to_hardware_component()],
    tasks=[
        _xarm6_cfg.to_task_config(task_type="servo", task_name="servo_arm"),
        _xarm6_cfg.to_task_config(task_type="velocity", task_name="velocity_arm"),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/control/joint_command", JointState),
    }
)

# Mock 6-DOF arm with CartesianIK
_mock_6dof_cfg = _catalog_piper(name="arm")

coordinator_cartesian_ik_mock = ControlCoordinator.blueprint(
    hardware=[_mock_6dof_cfg.to_hardware_component()],
    tasks=[
        _mock_6dof_cfg.to_task_config(
            task_type="cartesian_ik",
            task_name="cartesian_ik_arm",
            model_path=PIPER_FK_MODEL,
            ee_joint_id=_mock_6dof_cfg.dof,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
    }
)

# Piper arm with CartesianIK
coordinator_cartesian_ik_piper = ControlCoordinator.blueprint(
    hardware=[_piper_cfg.to_hardware_component()],
    tasks=[
        _piper_cfg.to_task_config(
            task_type="cartesian_ik",
            task_name="cartesian_ik_arm",
            model_path=PIPER_FK_MODEL,
            ee_joint_id=_piper_cfg.dof,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
    }
)

# XArm7 with TeleopIK (real, or MuJoCo with --simulation)
coordinator_teleop_xarm7 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm7_teleop_cfg.to_hardware_component()],
        tasks=[
            _xarm7_teleop_cfg.to_task_config(
                task_type="teleop_ik",
                task_name="teleop_xarm",
                model_path=XARM7_FK_MODEL,
                ee_joint_id=_xarm7_teleop_cfg.dof,
                hand="right",
                gripper_joint=make_gripper_joints("arm")[0],
                gripper_open_pos=0.85,
                gripper_closed_pos=0.0,
            ),
        ],
    ),
    *_mujoco_if_sim(str(XARM7_SIM_PATH), _xarm7_teleop_cfg.dof),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# XArm6 with TeleopIK (real, or MuJoCo with --simulation)
coordinator_teleop_xarm6 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_teleop_cfg.to_hardware_component()],
        tasks=[
            _xarm6_teleop_cfg.to_task_config(
                task_type="teleop_ik",
                task_name="teleop_xarm",
                model_path=XARM6_FK_MODEL,
                ee_joint_id=_xarm6_teleop_cfg.dof,
                hand="right",
                gripper_joint=make_gripper_joints("arm")[0],
                gripper_open_pos=0.85,
                gripper_closed_pos=0.0,
            ),
        ],
    ),
    *_mujoco_if_sim(str(XARM6_SIM_PATH), _xarm6_teleop_cfg.dof),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# Piper with TeleopIK (real, or MuJoCo with --simulation)
coordinator_teleop_piper = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_piper_teleop_cfg.to_hardware_component()],
        tasks=[
            _piper_teleop_cfg.to_task_config(
                task_type="teleop_ik",
                task_name="teleop_piper",
                model_path=PIPER_FK_MODEL,
                ee_joint_id=_piper_teleop_cfg.dof,
                hand="left",
                gripper_joint=make_gripper_joints("arm")[0],
                gripper_open_pos=0.0,
                gripper_closed_pos=0.035,
            ),
        ],
    ),
    *_mujoco_if_sim(str(PIPER_SIM_PATH), _piper_teleop_cfg.dof),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# Dual arm teleop: XArm6 + Piper with TeleopIK (real-only)
_xarm6_dual_cfg = _catalog_xarm6(
    name="xarm_arm", adapter_type="xarm", address=global_config.xarm6_ip
)
_piper_dual_cfg = _catalog_piper(
    name="piper_arm", adapter_type="piper", address=global_config.can_port
)

coordinator_teleop_dual = ControlCoordinator.blueprint(
    hardware=[_xarm6_dual_cfg.to_hardware_component(), _piper_dual_cfg.to_hardware_component()],
    tasks=[
        _xarm6_dual_cfg.to_task_config(
            task_type="teleop_ik",
            task_name="teleop_xarm",
            model_path=XARM6_FK_MODEL,
            ee_joint_id=_xarm6_dual_cfg.dof,
            hand="left",
        ),
        _piper_dual_cfg.to_task_config(
            task_type="teleop_ik",
            task_name="teleop_piper",
            model_path=PIPER_FK_MODEL,
            ee_joint_id=_piper_dual_cfg.dof,
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
