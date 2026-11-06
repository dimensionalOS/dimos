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


"""OpenYAM coordinator and planning through its connection module."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.damiao.damiao_blueprint import connect_damiao
from dimos.robot.manipulators.openyam.connection import OpenYamConnection

_openyam = OpenYamConnection.planning_model("arm")


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return joint_trajectory_task([*_openyam.joint_names, "arm/gripper"], priority=priority)


def _gripper_task() -> TaskConfig:
    return TaskConfig(name="arm_gripper", type="gripper", joint_names=["arm/gripper"], priority=20)


coordinator_openyam = connect_damiao(
    OpenYamConnection, tasks=[_trajectory_task(), _gripper_task()]
).remappings([])
openyam_planner_coordinator = autoconnect(
    planner(model=_openyam, gripper_tasks={"manipulator": "arm_gripper"}),
    connect_damiao(OpenYamConnection, tasks=[_trajectory_task(), _gripper_task()]),
)
