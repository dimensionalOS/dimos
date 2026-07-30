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
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
)
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common.topics import (
    CARTESIAN_IK_TASK_NAME,
    COORDINATOR_FRAME_ID,
    EEF_TWIST_TASK_NAME,
)


def trajectory_task(
    hardware: HardwareComponent,
    *additional_hardware: HardwareComponent,
    priority: int = 10,
    start_position_tolerance: float = 0.05,
) -> TaskConfig:
    hardware_components = (hardware, *additional_hardware)
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=[
            joint_name for component in hardware_components for joint_name in component.joints
        ],
        priority=priority,
        params={"start_position_tolerance": start_position_tolerance},
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


def eef_twist_task(
    hardware: HardwareComponent,
    *,
    model_path: Path,
    ee_joint_id: int,
    name: str = EEF_TWIST_TASK_NAME,
    priority: int = 10,
    params: dict[str, Any] | None = None,
) -> TaskConfig:
    task_params: dict[str, Any] = {"model_path": model_path, "ee_joint_id": ee_joint_id}
    if params:
        task_params.update(params)
    return TaskConfig(
        name=name,
        type="eef_twist",
        joint_names=hardware.joints,
        priority=priority,
        params=task_params,
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
    module_kwargs: dict[str, Any] = {
        "robots": list(robots),
        "planning_timeout": planning_timeout,
        **kwargs,
    }
    if visualization is not None:
        module_kwargs["visualization"] = visualization
    return ManipulationModule.blueprint(**module_kwargs)
