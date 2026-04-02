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

"""Simulation blueprint: base autonomy with Unity vehicle simulator."""

from __future__ import annotations

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.smartnav.blueprints._rerun_helpers import (
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
from dimos.navigation.smartnav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smartnav.modules.global_map.global_map import GlobalMap
from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smartnav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.simulation.unity.module import UnityBridgeModule
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


rerun_config = {
    "blueprint": _rerun_blueprint,
    "pubsubs": [LCM()],
    "min_interval_sec": 0.25,
    "visual_override": {
        "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
        "world/sensor_scan": sensor_scan_override,
        "world/terrain_map": terrain_map_override,
        "world/terrain_map_ext": terrain_map_ext_override,
        "world/global_map": global_map_override,
        "world/path": path_override,
        "world/way_point": waypoint_override,
        "world/goal_path": goal_path_override,
    },
    "static": {
        "world/color_image": UnityBridgeModule.rerun_static_pinhole,
        "world/floor": static_floor,
        "world/tf/robot": static_robot,
    },
}

simulation_blueprint = autoconnect(
    UnityBridgeModule.blueprint(
        unity_binary="",
        unity_scene="home_building_1",
    ),
    SensorScanGeneration.blueprint(),
    TerrainAnalysis.blueprint(obstacle_height_thre=0.2, max_rel_z=1.5),
    TerrainMapExt.blueprint(),
    LocalPlanner.blueprint(
        autonomy_mode=True,
        max_speed=2.0,
        autonomy_speed=2.0,
        obstacle_height_thre=0.2,
        max_rel_z=1.5,
        min_rel_z=-1.0,
    ),
    PathFollower.blueprint(
        autonomy_mode=True,
        max_speed=2.0,
        autonomy_speed=2.0,
        max_accel=4.0,
        slow_dwn_dis_thre=0.2,
    ),
    ClickToGoal.blueprint(),
    GlobalMap.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
)


def main() -> None:
    simulation_blueprint.build({"n_workers": 8}).loop()


if __name__ == "__main__":
    main()
