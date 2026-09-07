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

from pathlib import Path

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS,
    DUAL_OPENYAM_SIDES,
    dual_openyam_hardware,
    dual_openyam_model_config,
    dual_openyam_sim_hardware,
)


def dual_openyam_gripper_task(side: str, *, priority: int = 20) -> TaskConfig:
    """Gripper task named to match the group's ``gripper_hardware_id``."""
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    index = DUAL_OPENYAM_SIDES.index(side)
    return TaskConfig(
        name=f"{side}_arm_gripper",
        type="gripper",
        joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[index]],
        priority=priority,
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
    sim_scene_path: str | Path | None = None


class DualOpenYamCoordinator(TeleopControlCoordinator):
    """Select mock or explicit dual-CAN hardware during coordinator setup."""

    config: DualOpenYamCoordinatorConfig

    def _setup_from_config(self) -> None:
        if self.config.sim_scene_path is not None:
            if self.config.left_can_port is not None or self.config.right_can_port is not None:
                raise ValueError("A simulation scene cannot be combined with CAN ports")
            self.config.hardware = [dual_openyam_sim_hardware(self.config.sim_scene_path)]
            super()._setup_from_config()
            return
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
    planner(model=dual_openyam_model_config()),
    DualOpenYamCoordinator.blueprint(
        tasks=[
            dual_openyam_trajectory_task(),
            dual_openyam_gripper_task("left"),
            dual_openyam_gripper_task("right"),
        ],
    ),
)
