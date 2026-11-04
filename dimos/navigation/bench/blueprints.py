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

"""One benchmark case driven against the Go2's navigation stack in the simulated world."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.bench.driver import EpisodeDriver
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.robot.unitree.go2.config import voxel_size
from dimos.robot.unitree.go2.rerun import rerun_config
from dimos.simulation.go2_sim.world import ODOM_FRAME_ID, SimGo2MountTf, SimGo2World
from dimos.visualization.vis_module import vis_module

# The tuning the benchmark's baselines were scored with.
BODY_DILATE_M = -0.03
WALL_CLEARANCE_M = 0.05

go2_sim_nav_episode = autoconnect(
    RayTracingVoxelMap.blueprint(
        world_frame=ODOM_FRAME_ID,
        voxel_size=voxel_size,
        global_emit_every=50,
        viz_emit_every=5,
        viz_sweep_cells=1,
    ),
    MLSPlannerNative.blueprint(
        world_frame=ODOM_FRAME_ID,
        voxel_size=voxel_size,
        robot_height=0.4,
        surface_closing_radius=0.4,
        wall_clearance_m=WALL_CLEARANCE_M,
        wall_buffer_m=0.2,
        wall_buffer_weight=20.0,
        step_threshold_m=0.16,
        step_penalty_weight=4.0,
        viz_publish_hz=2.0,
        viz_sweep_cells=1,
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    LocalPlannerNative.blueprint(
        embodiment=GO2, body_dilate_m=BODY_DILATE_M, world_frame=ODOM_FRAME_ID
    ),
    TrajectoryFollowerNative.blueprint(embodiment=GO2),
    MovementManager.blueprint(),
    SimGo2World.blueprint(wait_for_scene=True),
    SimGo2MountTf.blueprint(),
    EpisodeDriver.blueprint(),
    # the runner records each episode through the Rerun bridge
    vis_module(global_config.viewer, rerun_config=rerun_config()),
).global_config(n_workers=10)
