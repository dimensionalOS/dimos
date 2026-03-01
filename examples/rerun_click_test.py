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

"""E2E test: Rerun viewer click → Go2 navigation goal.

Usage:
    1. Start the custom rerun viewer (with LCM click support):
       ./target/release/custom_callback_viewer

    2. Run this script:
       uv run python examples/rerun_click_test.py --simulation

    3. Click on entities in the Rerun viewer → robot navigates to click position.
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PointStamped
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    _convert_camera_info,
    _convert_global_map,
    _convert_navigation_costmap,
    _static_base_link,
    _transports_base,
)
from dimos.robot.unitree.go2.connection import go2_connection
from dimos.visualization.rerun.bridge import rerun_bridge
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

CUSTOM_VIEWER_URL = "rerun+http://127.0.0.1:9877/proxy"

rerun_config = {
    "pubsubs": [LCM(autoconf=True)],
    "viewer_mode": "connect",
    "connect_url": CUSTOM_VIEWER_URL,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/global_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}

with_vis = autoconnect(_transports_base, rerun_bridge(**rerun_config))

rerun_click_test = (
    autoconnect(
        with_vis,
        go2_connection(),
        websocket_vis(),
        voxel_mapper(voxel_size=0.1),
        cost_mapper(),
        replanning_a_star_planner(),
    )
    .transports(
        {
            ("clicked_point", PointStamped): LCMTransport("/clicked_point", PointStamped),
        }
    )
    .global_config(n_workers=6, robot_model="unitree_go2")
)

if __name__ == "__main__":
    coordinator = rerun_click_test.build()
    coordinator.start()
    import signal

    signal.pause()
