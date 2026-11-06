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

"""OpenArm trajectory and planner compositions."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.openarm.config import openarm_bimanual_model_config
from dimos.robot.manipulators.openarm.connection import OpenArmConnection

_MODEL = openarm_bimanual_model_config(robot="arm")


def _trajectory_task() -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(_MODEL.joint_names),
        priority=10,
        params={"start_position_tolerance": 0.05},
    )


def openarm_control(tasks: list[TaskConfig] | None = None) -> Blueprint:
    """A coordinator named ControlCoordinator over the OpenArm connection."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=OpenArmConnection.control_model("arm"),
            tasks=list(tasks if tasks is not None else [_trajectory_task()]),
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenArmConnection),
    ).remappings(arm_remaps(OpenArmConnection, DamiaoCoordinator))


coordinator_openarm = openarm_control().remappings([])

openarm_planner_coordinator = autoconnect(
    planner(model=_MODEL),
    openarm_control(),
)
