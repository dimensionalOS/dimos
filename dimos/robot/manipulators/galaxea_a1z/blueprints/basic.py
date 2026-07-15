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

# Arm-only stable configuration: the a1z SDK 'gripper' branch ships a G1Z
# gravity model that mismatches this unit's mounting (pushes the arm during
# zero-force startup; soft e-stop cannot catch the arm with comp disabled).
# Until the model/mount is calibrated, run the SDK *main* branch with
# gripper=False. Gripper code support remains in the adapter.
_a1z_hw = galaxea_a1z_hardware("arm", gripper=False)

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

# Planner (ManipulationModule) on real hardware. Arm-only model (A1Z_Flange)
# to stay consistent with the gripper=False hardware configuration above.
_planner_hw = galaxea_a1z_hardware("arm", gripper=False)

galaxea_a1z_planner_coordinator = autoconnect(
    planner(robots=[make_a1z_model_config(name="arm", has_gripper=False)]),
    coordinator(
        hardware=[_planner_hw],
        tasks=[trajectory_task(_planner_hw)],
    ),
)

# Keyboard teleop on real hardware (eef twist task from the FK model).
_teleop_hw = galaxea_a1z_hardware("arm", gripper=False)

keyboard_teleop_galaxea_a1z = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_teleop_hw],
        tasks=[eef_twist_task(_teleop_hw, model_path=A1Z_FK_MODEL, ee_joint_id=A1Z_DOF)],
    ),
    ManipulationModule.blueprint(
        robots=[make_a1z_model_config(name="arm", has_gripper=False)],
        visualization={"backend": "viser"},
    ),
)
