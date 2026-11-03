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

from __future__ import annotations

from dimos_lcm.std_msgs import Bool, String

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import stream_name_types
from dimos.core.module import ModuleBase
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.experimental.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.experimental.patrolling.module import PatrollingModule
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.route_follower.control_window import RouteControl
from dimos.navigation.route_follower.module import RouteFollower
from dimos.navigation.route_follower.overlay import route_path_overlay, route_state_overlay
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,
)
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_route import unitree_go2_route
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def _port_stream(blueprint: Blueprint, module: type[ModuleBase], port: str) -> str:
    atom = next(a for a in blueprint.blueprints if a.module is module)
    name = blueprint.remapping_map.get((atom.name, port), port)
    assert isinstance(name, str)
    return name


def test_blueprint_builds_with_the_route_follower() -> None:
    assert isinstance(unitree_go2_route, Blueprint)
    modules = [bp.module for bp in unitree_go2_route.blueprints]
    assert RouteFollower in modules
    assert RerunWebSocketServer in modules
    assert stream_name_types(unitree_go2_route)


def test_viewer_click_is_fed_by_the_websocket_server() -> None:
    assert _port_stream(unitree_go2_route, RerunWebSocketServer, "clicked_point") == "viewer_click"
    assert _port_stream(unitree_go2_route, RouteFollower, "viewer_click") == "viewer_click"
    assert ("viewer_click", PointStamped) in stream_name_types(unitree_go2_route)


def test_forwarded_click_feeds_the_planner_and_the_movement_manager() -> None:
    assert _port_stream(unitree_go2_route, RouteFollower, "clicked_point") == "clicked_point"
    assert (
        _port_stream(unitree_go2_route, ReplanningAStarPlanner, "clicked_point") == "clicked_point"
    )
    assert _port_stream(unitree_go2_route, MovementManager, "clicked_point") == "clicked_point"
    assert ("clicked_point", PointStamped) in stream_name_types(unitree_go2_route)


def test_stop_movement_is_seen_by_the_follower() -> None:
    assert _port_stream(unitree_go2_route, MovementManager, "stop_movement") == "stop_movement"
    assert _port_stream(unitree_go2_route, RouteFollower, "stop_seen") == "stop_movement"
    assert _port_stream(unitree_go2_route, RouteFollower, "route_stop") == "stop_movement"
    assert ("stop_movement", Bool) in stream_name_types(unitree_go2_route)


def test_teleop_twists_are_seen_by_the_follower() -> None:
    assert _port_stream(unitree_go2_route, RerunWebSocketServer, "tele_cmd_vel") == "tele_cmd_vel"
    assert _port_stream(unitree_go2_route, RouteFollower, "tele_cmd_seen") == "tele_cmd_vel"
    assert ("tele_cmd_vel", Twist) in stream_name_types(unitree_go2_route)


def test_goal_request_is_seen_by_the_follower() -> None:
    assert _port_stream(unitree_go2_route, PatrollingModule, "goal_request") == "goal_request"
    assert (
        _port_stream(unitree_go2_route, WavefrontFrontierExplorer, "goal_request") == "goal_request"
    )
    assert _port_stream(unitree_go2_route, RouteFollower, "route_goal") == "goal_request"
    assert _port_stream(unitree_go2_route, RouteFollower, "goal_seen") == "goal_request"
    assert ("goal_request", PoseStamped) in stream_name_types(unitree_go2_route)


def test_route_streams_use_the_contract_types() -> None:
    wired = stream_name_types(unitree_go2_route)
    assert ("odom", PoseStamped) in wired
    assert ("goal_reached", Bool) in wired
    assert ("route_cmd", String) in wired
    assert ("route_state", String) in wired
    assert ("route_path", Path) in wired


def test_each_stream_name_keeps_a_single_message_type() -> None:
    types_by_name: dict[str, set[type]] = {}
    for name, msg_type in stream_name_types(unitree_go2_route):
        types_by_name.setdefault(name, set()).add(msg_type)
    for name, msg_types in types_by_name.items():
        assert len(msg_types) == 1, f"{name} mixes message types: {msg_types}"


def test_route_control_window_ports_are_connected() -> None:
    modules = [bp.module for bp in unitree_go2_route.blueprints]
    assert RouteControl in modules
    assert _port_stream(unitree_go2_route, RouteControl, "route_state") == "route_state"
    assert _port_stream(unitree_go2_route, RouteFollower, "route_state") == "route_state"
    assert _port_stream(unitree_go2_route, RouteControl, "route_cmd") == "route_cmd"
    assert _port_stream(unitree_go2_route, RouteFollower, "route_cmd") == "route_cmd"


def test_overlay_merge_keeps_every_original_override_key() -> None:
    bridges = [a for a in unitree_go2_route.blueprints if a.module is RerunBridgeModule]
    assert len(bridges) == 1
    override = bridges[0].kwargs["visual_override"]
    assert set(go2_rerun_config["visual_override"]) <= set(override)
    assert override["world/route_state"] is route_state_overlay
    assert override["world/route_path"] is route_path_overlay
