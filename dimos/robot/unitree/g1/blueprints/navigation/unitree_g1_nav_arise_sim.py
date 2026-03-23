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

"""G1 nav sim with AriseSLAM — tests SLAM in simulation.

Instead of using Unity's ground-truth odometry, this blueprint feeds
the sim's body-frame lidar + synthetic IMU into AriseSLAM, which
estimates the pose via scan-to-map matching. This lets you test and
tune SLAM without real hardware.

Data flow:
    Unity → registered_scan → SensorScanGeneration → sensor_scan (body-frame)
         → AriseSLAM.raw_points → SLAM → registered_scan + odometry
    Unity → odometry → AriseSimAdapter → synthetic imu → AriseSLAM.imu

    AriseSLAM odometry + registered_scan feed into the nav stack as usual.

Note: AriseSLAM's odometry replaces Unity's ground-truth, so navigation
accuracy depends on how well SLAM tracks. Any drift is real SLAM drift.
"""

from __future__ import annotations

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
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
from dimos.navigation.smartnav.modules.arise_sim_adapter import AriseSimAdapter
from dimos.navigation.smartnav.modules.arise_slam.arise_slam import AriseSLAM
from dimos.navigation.smartnav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smartnav.modules.cmd_vel_mux import CmdVelMux
from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smartnav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.navigation.smartnav.modules.unity_bridge.unity_bridge import UnityBridgeModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.vis_module import vis_module


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="world", name="3D"),
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            row_shares=[2, 1],
        ),
    )


_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "blueprint": _rerun_blueprint,
        "pubsubs": [LCM()],
        "min_interval_sec": 0.25,
        "visual_override": {
            "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
            "world/sensor_scan": sensor_scan_override,
            "world/terrain_map": terrain_map_override,
            "world/terrain_map_ext": terrain_map_ext_override,
            "world/path": path_override,
            "world/way_point": waypoint_override,
            "world/goal_path": goal_path_override,
        },
        "static": {
            "world/color_image": UnityBridgeModule.rerun_static_pinhole,
            "world/floor": static_floor,
            "world/tf/robot": static_robot,
        },
    },
)

unitree_g1_nav_arise_sim = autoconnect(
    # Simulator — provides ground-truth registered_scan and odometry
    UnityBridgeModule.blueprint(
        unity_binary="",
        unity_scene="home_building_1",
        vehicle_height=1.24,
    ),
    # Body-frame scan from world-frame (using Unity's ground-truth odom)
    SensorScanGeneration.blueprint(),
    # Synthetic IMU from Unity's ground-truth odom
    AriseSimAdapter.blueprint(),
    # SLAM — estimates pose from body-frame lidar + synthetic IMU
    AriseSLAM.blueprint(use_imu=True),
    # Nav stack — uses SLAM's odometry + registered_scan (NOT Unity's)
    TerrainAnalysis.blueprint(
        extra_args=["--obstacleHeightThre", "0.2", "--maxRelZ", "1.5"]
    ),
    TerrainMapExt.blueprint(),
    LocalPlanner.blueprint(
        extra_args=[
            "--autonomyMode", "true",
            "--maxSpeed", "2.0",
            "--autonomySpeed", "2.0",
            "--obstacleHeightThre", "0.2",
            "--maxRelZ", "1.5",
            "--minRelZ", "-1.0",
        ]
    ),
    PathFollower.blueprint(
        extra_args=[
            "--autonomyMode", "true",
            "--maxSpeed", "2.0",
            "--autonomySpeed", "2.0",
            "--maxAccel", "4.0",
            "--slowDwnDisThre", "0.2",
        ]
    ),
    ClickToGoal.blueprint(),
    CmdVelMux.blueprint(),
    _vis,
).remappings(
    [
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
        # SensorScanGeneration's body-frame output → AriseSLAM input
        (SensorScanGeneration, "sensor_scan", "raw_points"),
        # Prevent Unity's registered_scan/odometry from feeding directly into
        # the nav stack — AriseSLAM's outputs should be used instead.
        # Rename Unity's outputs so they don't collide with AriseSLAM's.
        (UnityBridgeModule, "registered_scan", "sim_registered_scan"),
        (UnityBridgeModule, "odometry", "sim_odometry"),
        # SensorScanGeneration needs Unity's odom + scan (renamed above)
        (SensorScanGeneration, "registered_scan", "sim_registered_scan"),
        (SensorScanGeneration, "odometry", "sim_odometry"),
        # AriseSimAdapter needs Unity's odom too
        (AriseSimAdapter, "odometry", "sim_odometry"),
    ]
).global_config(n_workers=8, robot_model="unitree_g1", simulation=True)


def main() -> None:
    unitree_g1_nav_arise_sim.build().loop()


__all__ = ["unitree_g1_nav_arise_sim"]

if __name__ == "__main__":
    main()
