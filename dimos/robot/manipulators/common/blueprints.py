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

"""Small blueprint helpers shared by manipulator stacks."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common.topics import (
    CARTESIAN_IK_TASK_NAME,
    COORDINATOR_FRAME_ID,
    DEFAULT_TRAJECTORY_TASK_NAME,
    trajectory_task_name,
)


def trajectory_task(
    hardware: HardwareComponent,
    *,
    name: str | None = None,
    priority: int = 10,
) -> TaskConfig:
    return TaskConfig(
        name=name or trajectory_task_name(hardware.hardware_id),
        type="trajectory",
        joint_names=hardware.joints,
        priority=priority,
    )


def cartesian_ik_task(
    hardware: HardwareComponent,
    *,
    model_path: Path,
    ee_joint_id: int,
    name: str = CARTESIAN_IK_TASK_NAME,
    priority: int = 10,
) -> TaskConfig:
    return TaskConfig(
        name=name,
        type="cartesian_ik",
        joint_names=hardware.joints,
        priority=priority,
        params={"model_path": model_path, "ee_joint_id": ee_joint_id},
    )


def teleop_ik_task(
    hardware: HardwareComponent,
    *,
    model_path: Path,
    ee_joint_id: int,
    hand: str,
    name: str,
    priority: int = 10,
    params: dict[str, Any] | None = None,
) -> TaskConfig:
    task_params: dict[str, Any] = {
        "model_path": model_path,
        "ee_joint_id": ee_joint_id,
        "hand": hand,
    }
    if params:
        task_params.update(params)
    return TaskConfig(
        name=name,
        type="teleop_ik",
        joint_names=hardware.joints,
        priority=priority,
        params=task_params,
    )


def coordinator(
    *,
    hardware: Sequence[HardwareComponent] = (),
    tasks: Sequence[TaskConfig] = (),
    tick_rate: float = 100.0,
    publish_joint_state: bool = True,
    joint_state_frame_id: str = COORDINATOR_FRAME_ID,
) -> Blueprint:
    return ControlCoordinator.blueprint(
        tick_rate=tick_rate,
        publish_joint_state=publish_joint_state,
        joint_state_frame_id=joint_state_frame_id,
        hardware=list(hardware),
        tasks=list(tasks),
    )


def planner(
    *,
    robots: Sequence[RobotModelConfig],
    planning_timeout: float = 10.0,
    visualization: dict[str, Any] | None = None,
    **kwargs: Any,
) -> Blueprint:
    return ManipulationModule.blueprint(
        robots=list(robots),
        planning_timeout=planning_timeout,
        visualization=visualization or {"backend": "meshcat"},
        **kwargs,
    )


def default_trajectory_task_name(hardware_id: str) -> str:
    if hardware_id == "arm":
        return DEFAULT_TRAJECTORY_TASK_NAME
    return trajectory_task_name(hardware_id)
