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

"""Metal trajectory and gripper control over connection streams."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.makermods_metal.config import make_metal_model_config
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.robot.usd_manager import UsdManager

_MODEL = make_metal_model_config(robot="arm")


def makermods_metal_connection_control_blueprint() -> Blueprint:
    """Trajectories and the gripper, with the robot model served to clients."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=_MODEL.model,
            tasks=[
                joint_trajectory_task([*_MODEL.joint_names, "arm/joint7"]),
                # joint7 is open at its lower end.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/joint7"],
                    priority=20,
                    params={"open_at": "lower"},
                ),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(MetalConnection),
        UsdManager.blueprint(model=_MODEL.model),
    ).remappings(arm_remaps(MetalConnection, DamiaoCoordinator))


makermods_metal_connection_control = makermods_metal_connection_control_blueprint().remappings([])
