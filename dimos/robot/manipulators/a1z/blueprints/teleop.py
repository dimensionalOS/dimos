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

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.a1z.config import (
    a1z_hardware,
    make_a1z_model_config,
)
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    teleop_ik_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_a1z_keyboard_hw = a1z_hardware("arm")
_a1z_model = make_a1z_model_config()

keyboard_teleop_a1z = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_a1z_keyboard_hw],
        tasks=[
            eef_twist_task(
                _a1z_keyboard_hw,
                robot_model=_a1z_model,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            trajectory_task(_a1z_keyboard_hw, priority=20),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_a1z_model],
        visualization={"backend": "viser"},
    ),
)


_a1z_webxr_hw = a1z_hardware("arm")
_a1z_webxr_model = make_a1z_model_config()

coordinator_teleop_a1z = autoconnect(
    TeleopControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_a1z_webxr_hw],
        tasks=[
            teleop_ik_task(
                _a1z_webxr_hw,
                name="teleop_a1z",
                robot_model=_a1z_webxr_model,
                bindings=[
                    {
                        "hand": "left",
                        "target_frame": _a1z_webxr_model.end_effector_link,
                    }
                ],
                priority=20,
                params={"max_joint_velocity_rad_s": 2.0},
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            trajectory_task(_a1z_webxr_hw),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_a1z_webxr_model],
        visualization={"backend": "viser"},
    ),
)
