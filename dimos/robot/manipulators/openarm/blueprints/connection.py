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

"""OpenArm trajectories and normalized grippers through a connection module."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.openarm.config import openarm_bimanual_model_config
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.urdf_manager import UrdfManager

_MODEL = openarm_bimanual_model_config(robot="arm")
_GRIPPERS = [
    (f"arm_{name}", f"arm/{name}", f"{name}_command")
    for name in OpenArmConnection.robot.gripper_joint_names
]


def openarm_connection_control_blueprint() -> Blueprint:
    """Trajectories and normalized grippers with the robot model served to clients."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=OpenArmConnection.control_model("arm"),
            tasks=[
                joint_trajectory_task([*_MODEL.joint_names, *(joint for _, joint, _ in _GRIPPERS)]),
                *[
                    TaskConfig(
                        name=name,
                        type="gripper",
                        joint_names=[joint],
                        priority=20,
                        stream_bind={"gripper_command": stream},
                    )
                    for name, joint, stream in _GRIPPERS
                ],
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenArmConnection),
        UrdfManager.blueprint(model=OpenArmConnection.control_model("arm")),
    ).remappings(arm_remaps(OpenArmConnection, DamiaoCoordinator))


openarm_connection_control = openarm_connection_control_blueprint().remappings([])
