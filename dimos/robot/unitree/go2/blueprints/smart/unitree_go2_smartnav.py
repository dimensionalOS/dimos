#!/usr/bin/env python3
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

"""Go2 SmartNav: native C++ navigation with PGO loop closure.

Uses the SmartNav native modules (terrain analysis, local planner,
path follower) with PGO for loop-closure-corrected odometry.
OdomAdapter bridges GO2Connection's PoseStamped odom to Odometry
for the SmartNav modules.

Data flow:
    GO2Connection.lidar → registered_scan → TerrainAnalysis + LocalPlanner + PGO
    GO2Connection.odom → raw_odom → OdomAdapter → odometry → all nav modules
    PGO.corrected_odometry → OdomAdapter → odom (corrected PoseStamped)
    TerrainAnalysis → terrain_map → TerrainMapExt → LocalPlanner
    LocalPlanner → path → PathFollower → nav_cmd_vel → CmdVelMux → cmd_vel
    ClickToGoal → way_point → LocalPlanner
    Keyboard teleop → tele_cmd_vel → CmdVelMux → cmd_vel → GO2Connection
"""

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.smartnav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smartnav.modules.cmd_vel_mux import CmdVelMux
from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smartnav.modules.odom_adapter.odom_adapter import OdomAdapter
from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smartnav.modules.pgo.pgo import PGO
from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smartnav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(voxel_size=0.1, mode="boxes")


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
            fill_mode="wireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _go2_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        ),
    )


_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "blueprint": _go2_rerun_blueprint,
        "pubsubs": [LCM()],
        "visual_override": {
            "world/camera_info": _convert_camera_info,
            "world/global_map": _convert_global_map,
            "world/navigation_costmap": _convert_navigation_costmap,
        },
        "static": {
            "world/tf/base_link": _static_base_link,
        },
    },
)

unitree_go2_smartnav = (
    autoconnect(
        GO2Connection.blueprint(),
        SensorScanGeneration.blueprint(),
        OdomAdapter.blueprint(),
        PGO.blueprint(),
        TerrainAnalysis.blueprint(obstacle_height_thre=0.2, max_rel_z=1.5),
        TerrainMapExt.blueprint(),
        LocalPlanner.blueprint(
            autonomy_mode=True,
            max_speed=1.0,
            autonomy_speed=1.0,
            obstacle_height_thre=0.2,
            max_rel_z=1.5,
            min_rel_z=-0.5,
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
        _vis,
    )
    .remappings(
        [
            # GO2Connection outputs PoseStamped odom, rename to avoid collision
            # with OdomAdapter's Odometry output
            (GO2Connection, "odom", "raw_odom"),
            (GO2Connection, "lidar", "registered_scan"),
            # PathFollower cmd_vel → CmdVelMux nav input
            (PathFollower, "cmd_vel", "nav_cmd_vel"),
            # Keyboard teleop → CmdVelMux
            (RerunWebSocketServer, "tele_cmd_vel", "tele_cmd_vel"),
            # ClickToGoal plans at global scale — needs PGO-corrected odometry
            (ClickToGoal, "odometry", "corrected_odometry"),
            (TerrainAnalysis, "odometry", "corrected_odometry"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_smartnav"]
