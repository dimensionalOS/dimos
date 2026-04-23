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

"""Shared SmartNav module configs for G1 navigation blueprints.

Two autoconnect bundles with tuned parameters:
- _smart_nav: onboard (real hardware) — conservative speeds, detailed terrain config
- _smart_nav_sim: simulation — higher speeds, simpler terrain config

Also provides shared rerun visualization configs.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.cmd_vel_mux import CmdVelMux
from dimos.navigation.smart_nav.blueprints._rerun_helpers import (
    global_map_override,
    goal_path_override,
    path_override,
    sensor_scan_override,
    static_floor,
    static_robot,
    terrain_map_ext_override,
    terrain_map_override,
    waypoint_override,
)
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smart_nav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.g1.config import G1


# Rerun visualization
def _rerun_blueprint_3d() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


_rerun_config = {
    "blueprint": _rerun_blueprint_3d,
    "pubsubs": [LCM()],
    "min_interval_sec": 0.25,
    "visual_override": {
        "world/sensor_scan": sensor_scan_override,
        "world/terrain_map": terrain_map_override,
        "world/terrain_map_ext": terrain_map_ext_override,
        "world/global_map": global_map_override,
        "world/path": path_override,
        "world/way_point": waypoint_override,
        "world/goal_path": goal_path_override,
    },
    "static": {
        "world/floor": static_floor,
        "world/tf/robot": static_robot,
    },
}


# Onboard (real hardware)
_smart_nav = autoconnect(
    TerrainAnalysis.blueprint(
        # Input filtering
        scan_voxel_size=0.15,  # input point downsampling (m) — default 0.05, increased to reduce terrain_map density
        # Voxel grid
        terrain_voxel_size=1.0,  # grid cell size (m)
        terrain_voxel_half_width=10,  # grid radius in cells (→ 21×21)
        # Obstacle/ground classification
        obstacle_height_threshold=0.2,  # above this = hard obstacle (m)
        ground_height_threshold=0.1,  # below this = ground for cost mode (m)
        vehicle_height=G1.height_clearance,  # ignore points above this (m)
        min_relative_z=-1.5,  # height filter min relative to robot (m)
        max_relative_z=1.5,  # height filter max relative to robot (m)
        use_sorting=True,  # quantile-based ground estimation
        quantile_z=0.25,  # ground height quantile
        # Decay and clearing
        decay_time=2.0,  # point persistence (s)
        no_decay_distance=1.5,  # no-decay radius around robot (m) — default 4.0, reduced to prevent unbounded growth when stationary
        clearing_distance=8.0,  # dynamic clearing distance (m)
        clear_dynamic_obstacles=True,  # clear dynamic obstacles
        no_data_obstacle=False,  # treat unseen voxels as obstacles
        no_data_block_skip_count=0,  # skip N blocks with no data
        min_block_point_count=10,  # min points per block for classification
        # Voxel culling
        voxel_point_update_threshold=30,  # reprocess voxel after N points (default 100)
        voxel_time_update_threshold=2.0,  # cull voxel after N seconds
        # Dynamic obstacle filtering
        min_dynamic_obstacle_distance=0.14,  # min distance for dynamic obstacle detection (m)
        abs_dynamic_obstacle_relative_z_threshold=0.2,  # z threshold for dynamic obstacles (m)
        min_dynamic_obstacle_vfov=-55.0,  # min vertical FOV for dynamic obs (deg)
        max_dynamic_obstacle_vfov=10.0,  # max vertical FOV for dynamic obs (deg)
        min_dynamic_obstacle_point_count=1,  # min points for dynamic obstacle
        min_out_of_fov_point_count=20,  # min out-of-FOV points
        # Ground lift limits
        consider_drop=False,  # consider terrain drops
        limit_ground_lift=False,  # limit ground plane lift
        max_ground_lift=0.15,  # max ground lift (m)
        distance_ratio_z=0.2,  # distance-to-z ratio for filtering
    ),
    TerrainMapExt.blueprint(
        voxel_size=0.4,  # meters per voxel (coarser than local terrain)
        decay_time=8.0,  # seconds before points expire
        publish_rate=2.0,  # Hz
        max_range=40.0,  # max distance from robot to keep (m)
    ),
    LocalPlanner.blueprint(
        autonomy_mode=True,
        use_terrain_analysis=True,
        max_speed=1.0,
        autonomy_speed=1.0,
        obstacle_height_threshold=0.2,
        max_relative_z=1.5,
        min_relative_z=-1.5,
    ),
    PathFollower.blueprint(
        autonomy_mode=True,
        max_speed=1.0,
        autonomy_speed=1.0,
        max_acceleration=2.0,
        slow_down_distance_threshold=0.2,
    ),
    FarPlanner.blueprint(
        sensor_range=30.0,
        visibility_range=25.0,
    ),
    PGO.blueprint(),
    ClickToGoal.blueprint(),
    CmdVelMux.blueprint(),
).remappings(
    [
        # PathFollower cmd_vel → CmdVelMux nav input (avoid collision with mux output)
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        # Global-scale planners use PGO-corrected odometry (per CMU ICRA 2022):
        # "Loop closure adjustments are used by the high-level planners since
        # they are in charge of planning at the global scale. Modules such as
        # local planner and terrain analysis only care about the local
        # environment surrounding the vehicle and work in the odometry frame."
        (FarPlanner, "odometry", "corrected_odometry"),
        (ClickToGoal, "odometry", "corrected_odometry"),
        (TerrainAnalysis, "odometry", "corrected_odometry"),
    ]
)

# Simulation
_smart_nav_sim = autoconnect(
    TerrainAnalysis.blueprint(
        obstacle_height_threshold=0.2,
        max_relative_z=1.5,
    ),
    TerrainMapExt.blueprint(),
    LocalPlanner.blueprint(
        autonomy_mode=True,
        max_speed=2.0,
        autonomy_speed=2.0,
        obstacle_height_threshold=0.2,
        max_relative_z=1.5,
        min_relative_z=-1.0,
    ),
    PathFollower.blueprint(
        autonomy_mode=True,
        max_speed=2.0,
        autonomy_speed=2.0,
        max_acceleration=4.0,
        slow_down_distance_threshold=0.2,
    ),
    FarPlanner.blueprint(
        sensor_range=30.0,
        visibility_range=25.0,
    ),
    PGO.blueprint(),
    ClickToGoal.blueprint(),
    CmdVelMux.blueprint(),
).remappings(
    [
        # PathFollower cmd_vel → CmdVelMux nav input (avoid collision with mux output)
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        (FarPlanner, "odometry", "corrected_odometry"),
        (ClickToGoal, "odometry", "corrected_odometry"),
        (TerrainAnalysis, "odometry", "corrected_odometry"),
        (PGO, "global_map", "global_map_pgo"),
    ]
)
