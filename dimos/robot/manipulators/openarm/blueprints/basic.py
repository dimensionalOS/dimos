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

"""Basic OpenArm coordinator blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    openarm_bimanual_model_config,
    openarm_hardware,
)


def _trajectory_task() -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(OPENARM_ARM_JOINTS),
        priority=10,
        params={"start_position_tolerance": 0.05},
    )


class _OpenArmCoordinatorConfig(ControlCoordinatorConfig):
    """OpenArm deployment configuration requiring an explicit bus pair."""

    left_can_port: str | None = None
    right_can_port: str | None = None


class _OpenArmCoordinator(ControlCoordinator):
    """Select mock or explicitly addressed dual-CAN OpenArm hardware."""

    config: _OpenArmCoordinatorConfig

    def _setup_from_config(self) -> None:
        self.config.hardware = [
            openarm_hardware(
                left_can_port=self.config.left_can_port,
                right_can_port=self.config.right_can_port,
            )
        ]
        super()._setup_from_config()


openarm_planner_coordinator = autoconnect(
    planner(model=openarm_bimanual_model_config()),
    _OpenArmCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[_trajectory_task()],
    ),
)

coordinator_openarm = _OpenArmCoordinator.blueprint(
    instance_name="ControlCoordinator",
    tasks=[_trajectory_task()],
)
