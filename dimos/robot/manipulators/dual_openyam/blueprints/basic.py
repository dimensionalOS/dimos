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

"""Dual OpenYAM coordinator and planner through one connection."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_control_model,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection

_dual_openyam = dual_openyam_model_config(robot="arm")

# Planning group to the gripper task that moves its jaw.
DUAL_OPENYAM_GRIPPER_TASKS = {
    f"{side}_manipulator": f"{side}_arm_gripper" for side in DUAL_OPENYAM_SIDES
}


def dual_openyam_trajectory_task(*, priority: int = 20) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(_dual_openyam.joint_names),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


def _gripper_tasks() -> list[TaskConfig]:
    return [
        TaskConfig(
            name=DUAL_OPENYAM_GRIPPER_TASKS[f"{side}_manipulator"],
            type="gripper",
            joint_names=[f"arm/{side}_gripper"],
            priority=20,
        )
        for side in DUAL_OPENYAM_SIDES
    ]


def dual_openyam_control() -> Blueprint:
    """Trajectories and both jaws over the two-bus connection."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=dual_openyam_control_model(robot="arm"),
            tasks=[dual_openyam_trajectory_task(), *_gripper_tasks()],
            zero_stiffness_joints=[],
        ),
        arm_connection(DualOpenYamConnection),
    ).remappings(arm_remaps(DualOpenYamConnection, DamiaoCoordinator))


coordinator_dual_openyam = dual_openyam_control().remappings([])
dual_openyam_planner_coordinator = autoconnect(
    planner(model=_dual_openyam, gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS),
    dual_openyam_control(),
)
