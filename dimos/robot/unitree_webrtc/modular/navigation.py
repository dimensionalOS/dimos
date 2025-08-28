# Copyright 2025 Dimensional Inc.
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

from dimos_lcm.std_msgs import Bool, String

from dimos.core import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.navigation.bt_navigator.navigator import BehaviorTreeNavigator
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.navigation.global_planner import AstarPlanner
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def deploy_navigation(dimos, connection):
    mapper = dimos.deploy(Map, voxel_size=0.5, cost_resolution=0.05, global_publish_interval=1.0)
    mapper.lidar.connect(connection.lidar)
    mapper.global_map.transport = LCMTransport("/global_map", LidarMessage)
    mapper.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    mapper.local_costmap.transport = LCMTransport("/local_costmap", OccupancyGrid)

    """Deploy and configure navigation modules."""
    global_planner = dimos.deploy(AstarPlanner)
    local_planner = dimos.deploy(HolonomicLocalPlanner)
    navigator = dimos.deploy(
        BehaviorTreeNavigator,
        reset_local_planner=local_planner.reset,
        check_goal_reached=local_planner.is_goal_reached,
    )
    frontier_explorer = dimos.deploy(WavefrontFrontierExplorer)

    navigator.goal.transport = LCMTransport("/navigation_goal", PoseStamped)
    navigator.goal_request.transport = LCMTransport("/goal_request", PoseStamped)
    navigator.goal_reached.transport = LCMTransport("/goal_reached", Bool)
    navigator.navigation_state.transport = LCMTransport("/navigation_state", String)
    navigator.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    global_planner.path.transport = LCMTransport("/global_path", Path)
    local_planner.cmd_vel.transport = LCMTransport("/cmd_vel", Vector3)
    frontier_explorer.goal_request.transport = LCMTransport("/goal_request", PoseStamped)
    frontier_explorer.goal_reached.transport = LCMTransport("/goal_reached", Bool)
    frontier_explorer.explore_cmd.transport = LCMTransport("/explore_cmd", Bool)
    frontier_explorer.stop_explore_cmd.transport = LCMTransport("/stop_explore_cmd", Bool)

    global_planner.target.connect(navigator.goal)

    global_planner.global_costmap.connect(mapper.global_costmap)
    global_planner.odom.connect(connection.odom)

    local_planner.path.connect(global_planner.path)
    local_planner.local_costmap.connect(mapper.local_costmap)
    local_planner.odom.connect(connection.odom)

    connection.movecmd.connect(local_planner.cmd_vel)

    navigator.odom.connect(connection.odom)

    frontier_explorer.costmap.connect(mapper.global_costmap)
    frontier_explorer.odometry.connect(connection.odom)
    websocket_vis = dimos.deploy(WebsocketVisModule, port=7779)
    websocket_vis.click_goal.transport = LCMTransport("/goal_request", PoseStamped)

    websocket_vis.robot_pose.connect(connection.odom)
    websocket_vis.path.connect(global_planner.path)
    websocket_vis.global_costmap.connect(mapper.global_costmap)

    mapper.start()
    global_planner.start()
    local_planner.start()
    navigator.start()
    websocket_vis.start()

    return mapper
