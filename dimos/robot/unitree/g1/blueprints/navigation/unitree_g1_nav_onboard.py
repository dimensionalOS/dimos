#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""G1 nav onboard — FAR planner + PGO loop closure + local obstacle avoidance.

Full navigation stack on real hardware with:
- FAR visibility-graph global route planner
- PGO pose graph optimization with loop closure detection (GTSAM iSAM2)
- Local planner for reactive obstacle avoidance
- Path follower for velocity control
- FastLio2 SLAM from Livox Mid-360 lidar
- G1HighLevelDdsSdk for robot velocity commands

Odometry routing (per CMU ICRA 2022 Fig. 11):
- Local path modules (LocalPlanner, PathFollower, SensorScanGen):
  use raw odometry — they follow paths in the local odometry frame.
- Global/terrain modules (FarPlanner, ClickToGoal, TerrainAnalysis):
  use PGO corrected_odometry — they need globally consistent positions
  for terrain classification, visibility graphs, and goal coordinates.

Data flow:
    Click → ClickToGoal (corrected_odom) → goal → FarPlanner (corrected_odom)
    → way_point → LocalPlanner (raw odom) → path → PathFollower (raw odom)
    → nav_cmd_vel → CmdVelMux → cmd_vel → G1HighLevelDdsSdk

    registered_scan + odometry → PGO → corrected_odometry + global_map
"""

from __future__ import annotations

import os
from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.smartnav.blueprints._rerun_helpers import (
    goal_path_override,
    path_override,
    sensor_scan_override,
    static_floor,
    static_robot,
    terrain_map_ext_override,
    terrain_map_override,
    waypoint_override,
)
from dimos.navigation.smartnav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smartnav.modules.cmd_vel_mux import CmdVelMux
from dimos.navigation.smartnav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smartnav.modules.pgo.pgo import PGO
from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smartnav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.visualization.rerun.bridge import RerunBridgeModule, _resolve_viewer_mode
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


def _odometry_tf_override(odom: Any) -> Any:
    """Publish odometry as a TF frame so sensor_scan/path/robot can reference it.

    The z is zeroed because point clouds already have the full init_pose
    transform applied (ground at z≈0). Using the raw odom.z (= mount height)
    would double-count the vertical offset.
    """
    import rerun as rr

    tf = rr.Transform3D(
        translation=[odom.x, odom.y, 0.0],
        rotation=rr.Quaternion(
            xyzw=[
                odom.orientation.x,
                odom.orientation.y,
                odom.orientation.z,
                odom.orientation.w,
            ]
        ),
        parent_frame="tf#/map",
        child_frame="tf#/sensor",
    )
    return [
        ("tf#/sensor", tf),
    ]


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "pubsubs": [LCM()],
    "min_interval_sec": 0.25,
    "visual_override": {
        "world/odometry": _odometry_tf_override,
        "world/sensor_scan": sensor_scan_override,
        "world/terrain_map": terrain_map_override,
        "world/terrain_map_ext": terrain_map_ext_override,
        "world/path": path_override,
        "world/way_point": waypoint_override,
        "world/goal_path": goal_path_override,
    },
    "static": {
        "world/floor": static_floor,
        "world/tf/robot": static_robot,
    },
    "memory_limit": "1GB",
}

unitree_g1_nav_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            # G1 lidar mount: 1.2m height, 180° around X (upside-down mount)
            # [x, y, z, qx, qy, qz, qw] — quaternion (1,0,0,0) = 180° X rotation
            init_pose=[0.0, 0.0, 1.2, 1.0, 0.0, 0.0, 0.0],
            map_freq=1.0,
        ),
        # SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(
            # Input filtering
            scan_voxel_size=0.15,          # input point downsampling (m) — default 0.05, increased to reduce terrain_map density
            # Voxel grid
            terrain_voxel_size=1.0,        # grid cell size (m)
            terrain_voxel_half_width=10,   # grid radius in cells (→ 21×21)
            # Obstacle/ground classification
            obstacle_height_thre=0.2,      # above this = hard obstacle (m)
            ground_height_thre=0.1,        # below this = ground for cost mode (m)
            vehicle_height=1.2,            # ignore points above this (m)
            min_rel_z=-1.5,                # height filter min relative to robot (m)
            max_rel_z=1.5,                 # height filter max relative to robot (m)
            use_sorting=True,              # quantile-based ground estimation
            quantile_z=0.25,               # ground height quantile
            # Decay and clearing
            decay_time=2.0,                # point persistence (s)
            no_decay_dis=1.5,              # no-decay radius around robot (m) — default 4.0, reduced to prevent unbounded growth when stationary
            clearing_dis=8.0,              # dynamic clearing distance (m)
            clear_dy_obs=True,             # clear dynamic obstacles
            no_data_obstacle=False,        # treat unseen voxels as obstacles
            no_data_block_skip_num=0,      # skip N blocks with no data
            min_block_point_num=10,        # min points per block for classification
            # Voxel culling
            voxel_point_update_thre=30,    # reprocess voxel after N points (default 100)
            voxel_time_update_thre=2.0,    # cull voxel after N seconds
            # Dynamic obstacle filtering
            min_dy_obs_dis=0.14,           # min distance for dynamic obstacle detection (m)
            abs_dy_obs_rel_z_thre=0.2,     # z threshold for dynamic obstacles (m)
            min_dy_obs_vfov=-55.0,         # min vertical FOV for dynamic obs (deg)
            max_dy_obs_vfov=10.0,          # max vertical FOV for dynamic obs (deg)
            min_dy_obs_point_num=1,        # min points for dynamic obstacle
            min_out_of_fov_point_num=20,   # min out-of-FOV points
            # Ground lift limits
            consider_drop=False,           # consider terrain drops
            limit_ground_lift=False,       # limit ground plane lift
            max_ground_lift=0.15,          # max ground lift (m)
            dis_ratio_z=0.2,              # distance-to-z ratio for filtering
        ),
        TerrainMapExt.blueprint(
            voxel_size=0.4,      # meters per voxel (coarser than local terrain)
            decay_time=8.0,      # seconds before points expire
            publish_rate=2.0,    # Hz
            max_range=40.0,      # max distance from robot to keep (m)
        ),
        FarPlanner.blueprint(
            sensor_range=30.0,
            visibility_range=25.0,
        ),
        LocalPlanner.blueprint(
            autonomy_mode=True,
            use_terrain_analysis=True,
            max_speed=1.0,
            autonomy_speed=1.0,
            obstacle_height_thre=0.2,
            max_rel_z=1.5,
            min_rel_z=-1.5,
        ),
        PathFollower.blueprint(
            autonomy_mode=True,
            max_speed=1.0,
            autonomy_speed=1.0,
            max_accel=2.0,
            slow_dwn_dis_thre=0.2,
        ),
        PGO.blueprint(),
        ClickToGoal.blueprint(),
        CmdVelMux.blueprint(),
        G1HighLevelDdsSdk.blueprint(),
        RerunBridgeModule.blueprint(viewer_mode=_resolve_viewer_mode(), **_rerun_config),
        RerunWebSocketServer.blueprint(),
    )
    .remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
            (FastLio2, "global_map", "global_map_fastlio"),
            (PGO, "global_map", "global_map_pgo"),
            # PathFollower cmd_vel → CmdVelMux nav input (avoid name collision with mux output)
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
    .global_config(n_workers=12, robot_model="unitree_g1")
)


def main() -> None:
    unitree_g1_nav_onboard.build().loop()


__all__ = ["unitree_g1_nav_onboard"]

if __name__ == "__main__":
    main()
