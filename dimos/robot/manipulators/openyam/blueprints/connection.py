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

"""OpenYAM trajectory and gripper control through its connection."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.openyam.config import make_openyam_model_config
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.coordinator import OpenYamCoordinator
from dimos.robot.manipulators.openyam.joints import OPENYAM_GRIPPER_JOINT
from dimos.robot.urdf_manager import UrdfManager

_MODEL = make_openyam_model_config(robot="arm")
_GRIPPER_JOINTS = [OPENYAM_GRIPPER_JOINT]


def openyam_connection_control_blueprint() -> Blueprint:
    """Trajectories and the gripper, with the robot model served to clients."""
    return autoconnect(
        OpenYamCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(OpenYamConnection)],
            tasks=[
                joint_trajectory_task([*_MODEL.joint_names, *_GRIPPER_JOINTS]),
                # joint7 is open at its lower end.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=_GRIPPER_JOINTS,
                    priority=20,
                    params={"open_at": "lower"},
                ),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenYamConnection),
        UrdfManager.blueprint(model=_MODEL.model),
    ).remappings(arm_remaps(OpenYamConnection, OpenYamCoordinator))


openyam_connection_control = openyam_connection_control_blueprint().remappings([])
