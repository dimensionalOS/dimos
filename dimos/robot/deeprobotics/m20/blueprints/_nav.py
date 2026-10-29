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

"""The M20's navigation fragment: voxel map, planners and follower over the vendor LIO."""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.viz import planner_visual_override
from dimos.navigation.global_planner.viz import render_path
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.robot.deeprobotics.m20.constants import (
    BASE_LINK_HEIGHT_M,
    BODY_LENGTH_M,
    BODY_WIDTH_M,
    PLANNING_HEIGHT_M,
)

VOXEL_SIZE_M = 0.08
WALL_CLEARANCE_M = 0.3
PLANNER_VIZ_HZ = 0.0
WORLD_FRAME = "map"

_m20_nav = autoconnect(
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE_M,
        max_range=25.0,
        ray_subsample=5,
        emit_every=1,
        global_emit_every=50,
        support_min=4,
        world_frame=WORLD_FRAME,
        worker_threads=3,
    ).remappings([(RayTracingVoxelMap, "lidar", "slam_body_points")]),
    MLSPlannerNative.blueprint(
        world_frame=WORLD_FRAME,
        base_frame="base_link",
        voxel_size=VOXEL_SIZE_M,
        robot_height=M20.height,
        start_z_offset_m=M20.base_height,
        wall_clearance_m=WALL_CLEARANCE_M,
        wall_buffer_m=0.85,
        wall_buffer_weight=100.0,
        step_threshold_m=M20.steppable,
        step_penalty_weight=4.0,
        viz_publish_hz=PLANNER_VIZ_HZ,
        worker_threads=2,
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    LocalPlannerNative.blueprint(embodiment=M20, world_frame=WORLD_FRAME),
    TrajectoryFollowerNative.blueprint(embodiment=M20),
    MovementManager.blueprint(),
)


def _robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[
                BODY_LENGTH_M * 0.5,
                BODY_WIDTH_M * 0.5,
                PLANNING_HEIGHT_M * 0.5,
            ],
            centers=[0.0, 0.0, PLANNING_HEIGHT_M * 0.5 - BASE_LINK_HEIGHT_M],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def m20_nav_static() -> dict[str, Any]:
    """Bridge static entities for navigation: the body box on base_link."""
    return {"world/robot_body": _robot_body}


def m20_nav_overrides() -> dict[str, Any]:
    """Bridge overrides for the navigation maps, paths and planner debug, with the raw clouds hidden."""
    return {
        "world/lidar": None,
        "world/slam_body_points": None,
        "world/planner_path": render_path,
        **motion_visual_override(embodiment=M20),
        **planner_visual_override(
            PLANNER_VIZ_HZ, voxel_size=VOXEL_SIZE_M, wall_clearance_m=WALL_CLEARANCE_M
        ),
    }
