# Copyright 2026 Dimensional Inc.
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

"""Galaxea A1Z teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.robot.manipulators.a1z.config import (
    A1Z_DOF,
    A1Z_FK_MODEL,
    make_a1z_hardware,
)
from dimos.robot.manipulators.common.blueprints import eef_twist_task
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def _build_a1z_keyboard_components(simulation: str) -> tuple[Blueprint, ...]:
    """Build A1Z teleop components without consulting mutable config."""
    is_mujoco = simulation == "mujoco"
    if is_mujoco:
        from dimos.robot.manipulators.a1z.simulation import A1Z_SCENE_PATH, A1Z_SIM_HOME

        hardware = make_a1z_hardware(
            "arm",
            adapter_type="sim_mujoco",
            address=str(A1Z_SCENE_PATH),
            home_joints=list(A1Z_SIM_HOME),
        )
    else:
        hardware = make_a1z_hardware("arm", adapter_type="mock", address=None)
    if not is_mujoco:
        # make_a1z_hardware also honors the process-wide simulation setting;
        # teleop's explicit mode must win for non-MuJoCo values.
        hardware.adapter_type = "mock"
        hardware.address = None
        hardware.adapter_kwargs = {}
    tasks = [eef_twist_task(hardware, model_path=A1Z_FK_MODEL, ee_joint_id=A1Z_DOF)]
    modules: list[Blueprint] = [
        KeyboardTeleopModule.blueprint(linear_speed=0.05, angular_speed=0.5)
    ]
    if is_mujoco:
        from dimos.robot.manipulators.a1z.simulation import (
            A1Z_SCENE_PATH,
            A1Z_SIM_HOME,
            _A1ZMujocoSimModule,
        )

        tasks.append(
            TaskConfig(
                name="servo_gripper",
                type="servo",
                joint_names=["arm/gripper"],
                priority=20,
                params={"timeout": 0.0, "default_positions": [0.0]},
            )
        )
        modules.append(
            _A1ZMujocoSimModule.blueprint(
                address=str(A1Z_SCENE_PATH),
                headless=False,
                dof=A1Z_DOF,
                reset_joint_positions=list(A1Z_SIM_HOME),
                fps=30,
                camera_name="wrist_camera",
                gripper_control_mapping="identity",
            )
        )
    modules.append(ControlCoordinator.blueprint(hardware=[hardware], tasks=tasks))
    return tuple(modules)


keyboard_teleop_a1z = autoconnect(*_build_a1z_keyboard_components(global_config.simulation))
