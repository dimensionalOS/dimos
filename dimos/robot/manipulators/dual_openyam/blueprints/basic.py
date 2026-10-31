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
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.common.damiao_blueprint import connect_damiao
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection

_dual_openyam = DualOpenYamConnection.models()


def dual_openyam_trajectory_task(*, priority: int = 20) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(_dual_openyam.arm_joints),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


coordinator_dual_openyam = connect_damiao(
    DualOpenYamConnection, tasks=[dual_openyam_trajectory_task()]
).remappings([])
dual_openyam_planner_coordinator = autoconnect(
    planner(model=_dual_openyam.planning),
    connect_damiao(DualOpenYamConnection, tasks=[dual_openyam_trajectory_task()]),
)
