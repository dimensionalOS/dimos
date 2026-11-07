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

"""OpenArm trajectories and grippers through a connection module."""

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
# Each jaw's task name, its finger joint, its command stream and its open end.
_GRIPPERS = [
    (f"arm_{gripper.name}", f"arm/{gripper.joint}", f"{gripper.name}_command", gripper.open_at)
    for gripper in OpenArmConnection.robot.grippers
]


def openarm_connection_control_blueprint() -> Blueprint:
    """Trajectories and the grippers, with the robot model served to clients."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=_MODEL.model,
            tasks=[
                joint_trajectory_task(
                    [*_MODEL.joint_names, *(joint for _, joint, _, _ in _GRIPPERS)]
                ),
                *[
                    TaskConfig(
                        name=name,
                        type="gripper",
                        joint_names=[joint],
                        priority=20,
                        params={"open_at": open_at},
                        stream_bind={"gripper_command": stream},
                    )
                    for name, joint, stream, open_at in _GRIPPERS
                ],
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenArmConnection),
        UrdfManager.blueprint(model=_MODEL.model),
    ).remappings(arm_remaps(OpenArmConnection, DamiaoCoordinator))


openarm_connection_control = openarm_connection_control_blueprint().remappings([])
