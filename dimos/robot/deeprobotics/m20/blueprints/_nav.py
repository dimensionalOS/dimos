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

"""The M20's navigation stack: voxel map, planners and follower over the M20's odometry (vendor LIO or dimSLAM)."""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.cloud_relocalizer import MID360_NAV
from dimos.mapping.relocalization.module import PremapModule
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.native import TrajectoryFollowerNative
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME

VOXEL_SIZE_M = 0.08
WALL_CLEARANCE_M = 0.07
PLANNER_VIZ_HZ = 2.0
PLANNER_VIZ_SWEEP_CELLS = 1


def m20_nav_stack(
    embodiment: Embodiment = M20,
    lidar: str = "slam_body_points",
    pre_trace_subsample: int = 5,
    ray_subsample: int = 5,
) -> MetaBlueprint:
    """Voxel map, planners and follower for an M20 whose step and body limits are `embodiment`'s; the ray tracer traces the scans on `lidar`."""
    return autoconnect(
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=25.0,
            # 1 keeps every point: a lidar that voxelizes its own scans needs no more thinning here
            pre_trace_subsample=pre_trace_subsample,
            ray_subsample=ray_subsample,
            emit_every=1,
            global_emit_every=50,
            world_frame=WORLD_FRAME,
            worker_threads=3,
        ).remappings([(RayTracingVoxelMap, "lidar", lidar)]),
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


def m20_relocalization(lidar: str = "slam_body_points") -> Blueprint:
    """Continuous relocalization against the premap, matching a live map built from the scans on `lidar` (each placed by tf at its stamp)."""
    return PremapModule.blueprint(
        world_frame=WORLD_FRAME,
        map_frame="premap",
        # at 0.2 m live voxels a right office match scores ~0.5-0.7 and a wrong one ~0.25-0.35
        relocalize=MID360_NAV.model_copy(update={"fitness_threshold": 0.65}),
        # body scans placed in map (odometry), so a world -> map jump leaves the kept live map as it was
        live_cloud="lidar",
        relocalize_once=False,
        new_map=True,
        # a standing robot keeps ~6-7k live voxels at 0.2 m, so the first match waits for 2k, under the 20k default
        min_local_points=2_000,
        # coarse enough that a robot that roams the office keeps ~10k points, not ~500k (a global search in seconds, not minutes)
        live_voxel_m=0.2,
        match_radius_m=15.0,
    ).remappings([(PremapModule, "lidar", lidar)])
