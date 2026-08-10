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
from typing import Any, TypedDict

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common.topics import (
    CARTESIAN_IK_TASK_NAME,
    COORDINATOR_FRAME_ID,
    DEFAULT_TRAJECTORY_TASK_NAME,
    EEF_TWIST_TASK_NAME,
    trajectory_task_name,
)


class PinkControlIKOverrides(TypedDict, total=False):
    """Pink tuning values that may be overridden by a manipulator blueprint."""

    solver: str
    max_velocity: float
    lm_damping: float
    task_gain: float
    position_cost: float
    orientation_cost: float
    posture_cost: float
    joint_centering_cost: float
    damping_cost: float
    position_limit_margin: float
    seed_limit_tolerance: float
    reference_q: list[float] | None
    qpsolver_options: dict[str, float]


class GripperTaskOverrides(TypedDict, total=False):
    """Optional gripper fields shared by teleop and EEF-twist tasks."""

    gripper_joint: str
    gripper_open_pos: float
    gripper_closed_pos: float


def trajectory_task(
    hardware: HardwareComponent,
    *additional_hardware: HardwareComponent,
    name: str | None = None,
    priority: int = 10,
    start_position_tolerance: float = 0.05,
) -> TaskConfig:
    hardware_components = (hardware, *additional_hardware)
    return TaskConfig(
        name=name
        or (
            trajectory_task_name(hardware.hardware_id)
            if not additional_hardware
            else DEFAULT_TRAJECTORY_TASK_NAME
        ),
        type="trajectory",
        joint_names=[
            joint_name for component in hardware_components for joint_name in component.joints
        ],
        priority=priority,
        params={"start_position_tolerance": start_position_tolerance},
    )


def _resolve_control_ik(
    hardware: HardwareComponent,
    robot_model: RobotModelConfig,
    control_ik: PinkControlIKOverrides | None,
) -> dict[str, Any]:
    if len(hardware.joints) != len(robot_model.joint_names):
        raise ValueError("hardware and RobotModelConfig must have the same joint count")
    payload = dict(control_ik or {})
    payload["robot_model"] = robot_model
    return payload


def cartesian_ik_task(
    hardware: HardwareComponent,
    *,
    name: str = CARTESIAN_IK_TASK_NAME,
    priority: int = 10,
    timeout: float = 0.5,
    max_joint_delta_deg: float = 15.0,
    max_tracking_error_deg: float = 10.0,
    min_dt: float = 1e-4,
    max_dt: float = 0.05,
    control_ik: PinkControlIKOverrides | None = None,
    robot_model: RobotModelConfig,
) -> TaskConfig:
    resolved_control_ik = _resolve_control_ik(hardware, robot_model, control_ik)
    return TaskConfig(
        name=name,
        type="cartesian_ik",
        joint_names=hardware.joints,
        priority=priority,
        params={
            "control_ik": resolved_control_ik,
            "timeout": timeout,
            "max_joint_delta_deg": max_joint_delta_deg,
            "max_tracking_error_deg": max_tracking_error_deg,
            "min_dt": min_dt,
            "max_dt": max_dt,
        },
    )


def eef_twist_task(
    hardware: HardwareComponent,
    *,
    name: str = EEF_TWIST_TASK_NAME,
    priority: int = 10,
    timeout: float = 0.3,
    max_joint_delta_deg: float = 15.0,
    max_tracking_error_deg: float = 10.0,
    min_dt: float = 1e-4,
    max_dt: float = 0.05,
    control_ik: PinkControlIKOverrides | None = None,
    robot_model: RobotModelConfig,
    params: GripperTaskOverrides | None = None,
) -> TaskConfig:
    resolved_control_ik = _resolve_control_ik(hardware, robot_model, control_ik)
    task_params: dict[str, Any] = {
        "control_ik": resolved_control_ik,
        "timeout": timeout,
        "max_joint_delta_deg": max_joint_delta_deg,
        "max_tracking_error_deg": max_tracking_error_deg,
        "min_dt": min_dt,
        "max_dt": max_dt,
    }
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
    hand: str,
    name: str,
    robot_model: RobotModelConfig,
    priority: int = 10,
    timeout: float = 0.5,
    max_joint_delta_deg: float = 5.0,
    max_tracking_error_deg: float = 10.0,
    min_dt: float = 1e-4,
    max_dt: float = 0.05,
    control_ik: PinkControlIKOverrides | None = None,
    params: GripperTaskOverrides | None = None,
) -> TaskConfig:
    resolved_control_ik = _resolve_control_ik(hardware, robot_model, control_ik)
    task_params: dict[str, Any] = {
        "control_ik": resolved_control_ik,
        "hand": hand,
        "timeout": timeout,
        "max_joint_delta_deg": max_joint_delta_deg,
        "max_tracking_error_deg": max_tracking_error_deg,
        "min_dt": min_dt,
        "max_dt": max_dt,
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
    cls: type[ControlCoordinator] = ControlCoordinator,
    instance_name: str | None = None,
    publish_robot_joint_states: bool = False,
) -> Blueprint:
    """*cls* is the subclass declaring the `{hardware_id}_joints` outputs; pass
    instance_name="ControlCoordinator" with it so RPC clients still find it."""
    return cls.blueprint(
        tick_rate=tick_rate,
        publish_joint_state=publish_joint_state,
        publish_robot_joint_states=publish_robot_joint_states,
        joint_state_frame_id=joint_state_frame_id,
        instance_name=instance_name,
        hardware=list(hardware),
        tasks=list(tasks),
    )


def planner(
    *,
    model: RobotModelConfig,
    planning_timeout: float = 10.0,
    visualization: dict[str, Any] | None = None,
    **kwargs: Any,
) -> Blueprint:
    module_kwargs: dict[str, Any] = {
        "model": model,
        "planning_timeout": planning_timeout,
        **kwargs,
    }
    if visualization is not None:
        module_kwargs["visualization"] = visualization
    return ManipulationModule.blueprint(**module_kwargs)


def default_trajectory_task_name(hardware_id: str) -> str:
    if hardware_id == "arm":
        return DEFAULT_TRAJECTORY_TASK_NAME
    return trajectory_task_name(hardware_id)
