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

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.policy_teleop_task.policy_teleop_task import TakeoverFn
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.mapping.pointclouds.riding_box_filter import RidingBox, RidingBoxFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.assets.model import RobotModel
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing


class TeleopBinding(TypedDict):
    """Declarative mapping from an operator hand to one model target."""

    hand: str
    target_frame: str


def policy_teleop_task(
    joint_names: Sequence[str],
    *,
    operator_tasks: Sequence[str],
    policy_tasks: Sequence[str],
    name: str = "policy_teleop",
    priority: int = 10,
    alpha: float = 0.5,
    takeover_fn: str | TakeoverFn | None = None,
    switch_joints: Sequence[str] = (),
    weight_rate: float = 2.0,
    source_timeout_s: float = 0.2,
) -> TaskConfig:
    """Blend registered source tasks, which must precede this entry in ``tasks``."""
    return TaskConfig(
        name=name,
        type="policy_teleop",
        joint_names=list(joint_names),
        priority=priority,
        params={
            "operator_tasks": list(operator_tasks),
            "policy_tasks": list(policy_tasks),
            "alpha": alpha,
            "takeover_fn": takeover_fn,
            "switch_joints": list(switch_joints),
            "weight_rate": weight_rate,
            "source_timeout_s": source_timeout_s,
        },
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
    workspace_radius_m: float | None = None,
    workspace_frame: str | None = None,
    riding_boxes: Sequence[RidingBox] = (),
) -> MetaBlueprint:
    """Turn a camera's point cloud into the voxel map a ManipulationModule plans around.

    Four modules in a row: a self filter drops the robot's own returns, a riding
    box filter drops what rides on it (the gripper, a held object) at each cloud's
    capture time, a voxel mapper collects the rest, and a box filter leaves out
    what pick-and-place is picking, add_surface's surfaces and what lies outside the
    workspace. The box filter's output is named ``voxel_map``, so
    ManipulationModule's input of that name connects by itself. The camera's
    ``pointcloud`` output must be switched on.

    Args:
        self_filter_model: The robot whose returns are dropped. Parts left out of
            this model, such as a gripper, stay in the map.
        voxel_size: Edge of one map cell, metres. Give ManipulationModule the
            same ``voxel_map_resolution``, or its octree will not line up with
            the map.
        world_frame: Fixed frame the map is built and published in.
        workspace_radius_m: Cells farther than this from the workspace's centre are
            left out of the map; None keeps them all.
        workspace_frame: The workspace's centre, e.g. the arm's base; None for
            ``world_frame``'s origin.
        riding_boxes: Boxes on moving frames, such as the gripper, left out of the
            camera's cloud before it is mapped.
    """
    return autoconnect(
        UrdfManagerPointClearing.blueprint(
            # ManipulationModule already publishes the arm's frames.
            publish_joint_tf=False,
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
        PointCloudBoxFilter.blueprint(
            keep_within_m=workspace_radius_m,
            keep_within_frame=workspace_frame,
        ),
        # What rides on the arm, before the mapper.
        RidingBoxFilter.blueprint(boxes=list(riding_boxes)),
    ).remappings(
        [
            (RidingBoxFilter, "cloud", "filtered_pointcloud"),
            (RidingBoxFilter, "filtered_cloud", "unridden_pointcloud"),
            (RayTracingVoxelMap, "lidar", "unridden_pointcloud"),
            (PointCloudBoxFilter, "cloud", "global_map"),
            (PointCloudBoxFilter, "filtered_cloud", "voxel_map"),
        ]
    )
