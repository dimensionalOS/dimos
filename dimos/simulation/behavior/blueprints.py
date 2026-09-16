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

"""Working layers: R1 teleop, navigation, task execution, then agent tools."""

import rerun.blueprint as rrb

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.skills import SYSTEM_PROMPT, BehaviorSkills
from dimos.simulation.behavior.types import TaskSelection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module


def _view() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world"),
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image"),
                rrb.Spatial2DView(origin="world/left_wrist_image"),
                rrb.Spatial2DView(origin="world/right_wrist_image"),
            ),
        )
    )


behavior_teleop = autoconnect(
    BehaviorConnection.blueprint(),
    vis_module(global_config.viewer, rerun_config={"blueprint": _view}).remappings(
        [(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")]
    ),
).global_config(transport="zenoh")

behavior_nav = autoconnect(
    behavior_teleop,
    BehaviorConnection.blueprint(publish_scan=True, allow_task_changes=False),
    RayTracingVoxelMap.blueprint(voxel_size=0.1, world_frame="world").remappings(
        [(RayTracingVoxelMap, "lidar", "registered_scan")]
    ),
    MLSPlannerNative.blueprint(
        world_frame="world",
        voxel_size=0.1,
        robot_height=1.5,
        wall_clearance_m=0.3,
        viz_publish_hz=2.0,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    BasicPathFollower.blueprint(world_frame="world", speed=0.3).remappings(
        [(BasicPathFollower, "nav_cmd_vel", "cmd_vel")]
    ),
    vis_module(global_config.viewer, rerun_config={"blueprint": _view}).remappings(
        [
            (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
            (RerunWebSocketServer, "clicked_point", "goal"),
        ]
    ),
).global_config(transport="zenoh")

behavior_task = autoconnect(
    behavior_teleop,
    BehaviorConnection.blueprint(task=TaskSelection()),
).global_config(transport="zenoh")

behavior_agentic = autoconnect(
    behavior_task,
    BehaviorSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=SYSTEM_PROMPT),
).global_config(transport="zenoh")
