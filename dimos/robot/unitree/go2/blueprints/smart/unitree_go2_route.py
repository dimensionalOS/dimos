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

"""unitree-go2-route: the Go2 smart stack plus the multi-waypoint route follower.

Remappings follow specs/001-multi-waypoint-route/contracts/route-streams.md.
The vis_module replaces the one from unitree_go2_basic (autoconnect keeps the
newer atom per module class), so its rerun config is a copy of the Go2's with
the route overlays merged in, keeping every original override and the layout.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.route_follower.control_window import RouteControl
from dimos.navigation.route_follower.module import RouteFollower
from dimos.navigation.route_follower.overlay import route_path_overlay, route_state_overlay
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,
)
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

_route_rerun_config = {
    **go2_rerun_config,
    "visual_override": {
        **go2_rerun_config["visual_override"],
        "world/route_state": route_state_overlay,
        "world/route_path": route_path_overlay,
    },
}

unitree_go2_route = (
    autoconnect(
        unitree_go2,
        RouteFollower.blueprint(),
        RouteControl.blueprint(),
        vis_module(viewer_backend=global_config.viewer, rerun_config=_route_rerun_config),
    )
    .remappings(
        [
            (RerunWebSocketServer, "clicked_point", "viewer_click"),
            (RouteFollower, "route_goal", "goal_request"),
            (RouteFollower, "goal_seen", "goal_request"),
            (RouteFollower, "route_stop", "stop_movement"),
            (RouteFollower, "stop_seen", "stop_movement"),
            (RouteFollower, "tele_cmd_seen", "tele_cmd_vel"),
        ]
    )
    .global_config(n_workers=12, robot_model="unitree_go2")
)
