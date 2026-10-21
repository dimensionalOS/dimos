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
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.roboplan_point_cloud_self_filter import (
    RoboPlanPointCloudSelfFilter,
)
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.topics import (
    CARTESIAN_IK_TASK_NAME,
    COORDINATOR_FRAME_ID,
    EEF_TWIST_TASK_NAME,
)


class TeleopBinding(TypedDict):
    """Declarative mapping from an operator hand to one model target."""

    hand: str
    target_frame: str


def trajectory_task(
    hardware: HardwareComponent,
    *additional_hardware: HardwareComponent,
    priority: int = 10,
    start_position_tolerance: float = 0.05,
) -> TaskConfig:
    hardware_components = (hardware, *additional_hardware)
    return joint_trajectory_task(
        [joint_name for component in hardware_components for joint_name in component.joints],
        priority=priority,
        start_position_tolerance=start_position_tolerance,
    )


def _model_joint_names(
    hardware: HardwareComponent,
    robot_model: RobotModelConfig,
) -> list[str]:
    joint_names = list(robot_model.joint_names)
    if not set(joint_names) <= set(hardware.joints):
        raise ValueError("hardware joints must contain RobotModelConfig joints")
    return joint_names


def cartesian_ik_task(
    hardware: HardwareComponent,
    *,
    robot_model: RobotModelConfig,
    target_frame: str,
    name: str = CARTESIAN_IK_TASK_NAME,
    priority: int = 10,
) -> TaskConfig:
    return TaskConfig(
        name=name,
        type="cartesian_ik",
        joint_names=_model_joint_names(hardware, robot_model),
        priority=priority,
        params={"robot_model": robot_model, "target_frame": target_frame},
    )


def eef_twist_task(
    hardware: HardwareComponent,
    *,
    robot_model: RobotModelConfig,
    target_frame: str,
    name: str = EEF_TWIST_TASK_NAME,
    priority: int = 10,
    timeout: float = 0.3,
    max_joint_velocity_rad_s: float = 5.0,
    max_command_tracking_error_deg: float = 10.0,
    pink: PinkKinematicsConfig | None = None,
) -> TaskConfig:
    task_params: dict[str, Any] = {
        "robot_model": robot_model,
        "target_frame": target_frame,
        "timeout": timeout,
        "max_joint_velocity_rad_s": max_joint_velocity_rad_s,
        "max_command_tracking_error_deg": max_command_tracking_error_deg,
    }
    if pink is not None:
        task_params["pink"] = pink
    return TaskConfig(
        name=name,
        type="eef_twist",
        joint_names=_model_joint_names(hardware, robot_model),
        priority=priority,
        params=task_params,
    )


def teleop_ik_task(
    hardware: HardwareComponent,
    *,
    robot_model: RobotModelConfig,
    bindings: Sequence[TeleopBinding],
    name: str,
    joint_names: Sequence[str] | None = None,
    priority: int = 10,
    solver_type: type[PinkPoseTargetSolver] = PinkPoseTargetSolver,
    params: dict[str, Any] | None = None,
) -> TaskConfig:
    task_params: dict[str, Any] = {
        "robot_model": robot_model,
        "bindings": list(bindings),
        "solver_type": solver_type,
    }
    if params:
        task_params.update(params)
    return TaskConfig(
        name=name,
        type="teleop_ik",
        joint_names=(
            list(joint_names)
            if joint_names is not None
            else _model_joint_names(hardware, robot_model)
        ),
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
    publish_frame_poses: bool = False,
) -> Blueprint:
    """*cls* is the subclass declaring the `{hardware_id}_joints` outputs; pass
    instance_name="ControlCoordinator" with it so RPC clients still find it."""
    return cls.blueprint(
        tick_rate=tick_rate,
        publish_joint_state=publish_joint_state,
        publish_robot_joint_states=publish_robot_joint_states,
        publish_frame_poses=publish_frame_poses,
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


def manipulation_mapping(
    *,
    self_filter_model: RobotModel,
    voxel_size: float,
    world_frame: str = "world",
) -> Blueprint:
    """Turn a camera's point cloud into the voxel map a ManipulationModule plans around.

    Three modules in a row: a self filter drops the robot's own returns, a
    voxel mapper collects the rest, and a box filter lets pick-and-place leave
    the object it is picking out of the map. The box filter's output is named
    ``voxel_map``, so ManipulationModule's input of that name connects by
    itself. The camera's ``pointcloud`` output must be switched on.

    Args:
        self_filter_model: The robot whose returns are dropped. Parts left out of
            this model, such as a gripper, stay in the map.
        voxel_size: Edge of one map cell, metres. Give ManipulationModule the
            same ``voxel_map_resolution``, or its octree will not line up with
            the map.
        world_frame: Fixed frame the map is built and published in.
    """
    return autoconnect(
        RoboPlanPointCloudSelfFilter.blueprint(
            model=self_filter_model,
            state_tolerance_s=0.1,
            # ManipulationModule publishes robot TF at 10Hz, so the stock 20ms
            # tolerance cannot bracket a ~92ms publish period and drops most
            # clouds. One full period admits them all, and the arm holds still
            # while scanning, so a transform a period old describes the same pose.
            tf_tolerance_s=0.1,
            tf_forward_tolerance_s=0.1,
        ),
        # Tabletop reach, not a room-scale lidar sweep.
        RayTracingVoxelMap.blueprint(
            voxel_size=voxel_size,
            world_frame=world_frame,
            max_range=2.0,
        ),
        PointCloudBoxFilter.blueprint(),
    ).remappings(
        [
            (RayTracingVoxelMap, "lidar", "filtered_pointcloud"),
            (PointCloudBoxFilter, "cloud", "global_map"),
            (PointCloudBoxFilter, "filtered_cloud", "voxel_map"),
        ]
    )
