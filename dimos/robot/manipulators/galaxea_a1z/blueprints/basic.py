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

"""Basic Galaxea A1Z coordinator, planner, and teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.a1z.config import (
    A1Z_DOF,
    A1Z_FK_MODEL,
    A1Z_G1Z_MODEL_PATH,
    make_a1z_model_config,
)
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    planner,
    trajectory_task,
)
from dimos.robot.manipulators.galaxea_a1z.config import galaxea_a1z_hardware
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# Gripper commands remain disabled, but the physical distal G1Z mass must be
# present in the SDK dynamics model or joints 2-4 sag under the missing load.
_A1Z_DYNAMICS_URDF = str(A1Z_G1Z_MODEL_PATH)
_a1z_hw = galaxea_a1z_hardware("arm", gripper=False, dynamics_urdf_path=_A1Z_DYNAMICS_URDF)

coordinator_galaxea_a1z = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_a1z_hw],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=_a1z_hw.joints,
                priority=10,
            )
        ],
    ),
)

# Planner and Viser use the physical G1Z model, including its fixed gripper
# geometry and TCP. Gripper commands stay disabled until the vendor SDK supports
# that hardware branch.
_planner_hw = galaxea_a1z_hardware("arm", gripper=False, dynamics_urdf_path=_A1Z_DYNAMICS_URDF)

galaxea_a1z_planner_coordinator = autoconnect(
    planner(
        robots=[
            make_a1z_model_config(
                name="arm",
                has_gripper=True,
                enable_gripper_control=False,
            )
        ]
    ),
    coordinator(
        hardware=[_planner_hw],
        tasks=[trajectory_task(_planner_hw)],
    ),
)

# Keyboard teleop on real hardware (eef twist task from the FK model).
_teleop_hw = galaxea_a1z_hardware("arm", gripper=False, dynamics_urdf_path=_A1Z_DYNAMICS_URDF)

keyboard_teleop_galaxea_a1z = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_teleop_hw],
        tasks=[eef_twist_task(_teleop_hw, model_path=A1Z_FK_MODEL, ee_joint_id=A1Z_DOF)],
    ),
    ManipulationModule.blueprint(
        robots=[
            make_a1z_model_config(
                name="arm",
                has_gripper=True,
                enable_gripper_control=False,
            )
        ],
        visualization={"backend": "viser"},
    ),
)
