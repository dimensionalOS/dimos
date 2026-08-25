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

"""Basic OpenYAM coordinator blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator, planner
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_HARDWARE_ID,
    OPENYAM_JOINTS,
    make_openyam_model_config,
    openyam_hardware,
)


def _trajectory_task() -> TaskConfig:
    return joint_trajectory_task(OPENYAM_JOINTS)


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name=f"{OPENYAM_HARDWARE_ID}_gripper",
        type="gripper",
        joint_names=[OPENYAM_GRIPPER_JOINT],
        priority=20,
    )


_openyam_planner_hw = openyam_hardware()

openyam_planner_coordinator = autoconnect(
    planner(robots=[make_openyam_model_config(name="arm")]),
    coordinator(
        hardware=[_openyam_planner_hw],
        tasks=[_trajectory_task(), _gripper_task()],
    ),
)

_openyam_hw = openyam_hardware()

coordinator_openyam = ControlCoordinator.blueprint(
    hardware=[_openyam_hw],
    tasks=[_trajectory_task(), _gripper_task()],
)
