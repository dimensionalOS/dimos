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

"""Dual OpenYAM coordinator and planning blueprints."""

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ARM_JOINTS,
    dual_openyam_hardware,
    dual_openyam_model_config,
)


def dual_openyam_trajectory_task(*, priority: int = 20) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(DUAL_OPENYAM_ARM_JOINTS),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


class DualOpenYamCoordinatorConfig(ControlCoordinatorConfig):
    """Dual OpenYAM deployment configuration."""

    left_can_port: str | None = None
    right_can_port: str | None = None


class DualOpenYamCoordinator(TeleopControlCoordinator):
    """Select mock or explicit dual-CAN hardware during coordinator setup."""

    config: DualOpenYamCoordinatorConfig

    def _setup_from_config(self) -> None:
        self.config.hardware = [
            dual_openyam_hardware(
                left_can_port=self.config.left_can_port,
                right_can_port=self.config.right_can_port,
            )
        ]
        super()._setup_from_config()


coordinator_dual_openyam = DualOpenYamCoordinator.blueprint(
    tasks=[dual_openyam_trajectory_task()],
)

dual_openyam_planner_coordinator = autoconnect(
    planner(robots=[dual_openyam_model_config()]),
    DualOpenYamCoordinator.blueprint(
        tasks=[dual_openyam_trajectory_task()],
    ),
)
