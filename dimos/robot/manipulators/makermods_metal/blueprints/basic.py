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

"""The main Metal coordinator entry, using its direct connection."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.makermods_metal.config import make_metal_model_config
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection

_MODEL = make_metal_model_config(robot="arm")


def metal_control() -> Blueprint:
    """Trajectories and the Metal jaw over the connection."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(MetalConnection)],
            tasks=[
                joint_trajectory_task(list(_MODEL.joint_names)),
                # joint7 is open at its lower end.
                TaskConfig(
                    name="metal_gripper",
                    type="gripper",
                    joint_names=["arm/joint7"],
                    priority=20,
                    params={"open_at": "lower"},
                ),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(MetalConnection),
    ).remappings(arm_remaps(MetalConnection, DamiaoCoordinator))


coordinator_makermods_metal = metal_control().remappings([])
