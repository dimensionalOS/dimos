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

"""G1 basic nav onboard — local planner + path follower only (no FAR/PGO).

Lightweight navigation stack for real hardware: uses SmartNav C++ native
modules for terrain analysis, local planning, and path following.
FastLio2 provides SLAM from a Livox Mid-360 lidar. No global route
planner (FAR) or loop closure (PGO). For the full stack, use
unitree_g1_nav_onboard.
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
from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
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


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "pubsubs": [LCM()],
    "min_interval_sec": 0.25,
    "visual_override": {
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
}

unitree_g1_nav_basic_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            # G1 lidar mount: 1.2m height, 180° around X (upside-down mount)
            # [x, y, z, qx, qy, qz, qw] — quaternion (1,0,0,0) = 180° X rotation
            init_pose=[0.0, 0.0, 1.2, 1.0, 0.0, 0.0, 0.0],
            map_freq=1.0,  # Publish global map at 1 Hz
        ),
        SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(
            obstacle_height_thre=0.2,
            max_rel_z=1.5,
            vehicle_height=1.2,
        ),
        TerrainMapExt.blueprint(),
        LocalPlanner.blueprint(
            autonomy_mode=True,
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
            # PathFollower cmd_vel → CmdVelMux nav input (avoid name collision with mux output)
            (PathFollower, "cmd_vel", "nav_cmd_vel"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1")
)


def main() -> None:
    unitree_g1_nav_basic_onboard.build().loop()


__all__ = ["unitree_g1_nav_basic_onboard"]

if __name__ == "__main__":
    main()
