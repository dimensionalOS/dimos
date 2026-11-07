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

"""The M20's navigation stack: voxel map, planners and follower over the vendor LIO."""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.native import TrajectoryFollowerNative
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME

VOXEL_SIZE_M = 0.08
WALL_CLEARANCE_M = 0.3
PLANNER_VIZ_HZ = 2.0
PLANNER_VIZ_SWEEP_CELLS = 1


def m20_nav_stack(embodiment: Embodiment = M20) -> Blueprint:
    """Voxel map, planners and follower for an M20 whose step and body limits are `embodiment`'s."""
    return autoconnect(
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=25.0,
            ray_subsample=5,
            emit_every=1,
            global_emit_every=50,
            world_frame=WORLD_FRAME,
            worker_threads=3,
        ).remappings([(RayTracingVoxelMap, "lidar", "slam_body_points")]),
        MLSPlannerNative.blueprint(
            world_frame=WORLD_FRAME,
            voxel_size=VOXEL_SIZE_M,
            robot_height=embodiment.height,
            start_z_offset_m=embodiment.base_height,
            wall_clearance_m=WALL_CLEARANCE_M,
            wall_buffer_m=0.85,
            wall_buffer_weight=100.0,
            step_threshold_m=embodiment.steppable,
            viz_publish_hz=PLANNER_VIZ_HZ,
            viz_sweep_cells=PLANNER_VIZ_SWEEP_CELLS,
            worker_threads=2,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
            ]
        ),
        LocalPlannerNative.blueprint(embodiment=embodiment, world_frame=WORLD_FRAME),
        TrajectoryFollowerNative.blueprint(embodiment=embodiment),
        MovementManager.blueprint(),
    )
