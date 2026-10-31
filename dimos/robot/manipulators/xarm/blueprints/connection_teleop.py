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

"""Keyboard teleop blueprints using the optional xArm connection."""

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.xarm.config import (
    XARM_ARMS,
    XArmType,
    make_xarm_model_config,
    xarm_model,
)
from dimos.robot.manipulators.xarm.connection import XArmConnection
from dimos.robot.manipulators.xarm.twist_coordinator import XArmTwistCoordinator
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def _keyboard_teleop(arm: XArmType) -> Blueprint:
    """Jog an arm through XArmConnection; set --xarmconnection.ip at run time."""
    dof, robot_type = XARM_ARMS[arm]
    ik_model = make_xarm_model_config(dof, robot_type=robot_type, robot="arm", add_gripper=False)
    return autoconnect(
        KeyboardTeleopModule.blueprint(),
        XArmTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=xarm_model(arm, robot="arm"),
            tasks=[
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(ik_model.joint_names),
                    priority=10,
                    params={
                        "robot_model": ik_model,
                        "target_frame": f"link{dof}",
                        "timeout": 0.3,
                    },
                ),
            ],
        ),
        XArmConnection.blueprint(arm=arm),
    )


_ARM_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (XArmConnection, "position_command", "arm_position_command"),
    (XArmConnection, "velocity_command", "arm_velocity_command"),
    (XArmTwistCoordinator, "arm_joint_state", "joint_state"),
]


xarm7_keyboard_teleop = _keyboard_teleop("xarm7").remappings(_ARM_REMAPS)
xarm6_keyboard_teleop = _keyboard_teleop("xarm6").remappings(_ARM_REMAPS)
