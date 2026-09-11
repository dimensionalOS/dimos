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

"""The same independently runnable DimOS robot composition for all six ducks."""

import os
from uuid import UUID

from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.pollen.microduck.config import MICRODUCK
from microduck_world.cockpit import world_cockpit
from microduck_world.connection import SimRobotConnection
from microduck_world.control import WorldControl
from microduck_world.exploration import DuckExplorer
from microduck_world.knowledge import DuckKnowledge
from microduck_world.robot_io import ROBOT_IDS
from microduck_world.robot_mcp import RobotMcpServer
from microduck_world.roster import ROSTER
from microduck_world.scene import PROJECT_ROOT, load_world


def robot_blueprint(robot: str, generation: str) -> Blueprint:
    if robot not in ROBOT_IDS:
        raise ValueError("Unknown robot")
    UUID(generation)
    player = ROSTER[robot]
    port = int(player["mcp_port"]) + int(os.environ.get("MICRODUCK_MCP_PORT_OFFSET", "0"))
    if not 1024 <= port <= 65535:
        raise ValueError("MCP port offset is outside the allowed port range")
    scene = load_world()[1]
    directory = PROJECT_ROOT / "state/robots" / scene.id / robot / generation
    prompt = f"""You operate {player["name"]}, a small biped on the {player["team"]} football team.
You have your own camera, range sensors, map, places and conversation.
Only use knowledge from your own tools, observations and the user's instructions.
The full-world browser view and other ducks' data are unavailable to you.
Start idle. Do not explore or move until instructed. Agent mode enables navigation.
Use observe for visual questions. Record useful findings with record_observation;
use remember_object with the observation ID and a pixel on the visible object.
Positions for object annotations come from your measured depth, not guesses.
Use remember_place to name your current location. Query understanding and list_places
to recall what you have learned. Rooms and object names appearing in tool examples
are examples, not evidence that those places exist here.
Use begin_exploration and end_exploration to discover your own map's frontiers.
Kicks only perform the physical motion: approach and align with a ball first.
A kick does not place or attract a ball. Never claim a goal without observing it.
Move slowly. Use movement tools one at a time; stop exploration before navigating
to a named place or performing a trick. Use list_policies for available actions.
Never pretend to see around an obstacle or infer exact unobserved room boundaries.
Distinguish observations from hypotheses. Keep replies brief.
"""
    prompt += (
        "You know the pitch and both locker-room locations as supplied prior knowledge. "
        "Your occupancy map and other discoveries come from your own sensors. "
        "Both teams play free football, with no kickoff, timer or automatic resets. "
        "Blue defends the west goal; red defends the east goal. "
        "Teammates do not share their private observations or tools.\n"
    )
    modules = [
        SimRobotConnection.blueprint(generation=generation),
        VoxelGridMapper.blueprint(emit_every=1, voxel_size=0.03, device="CPU:0"),
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=0.03, can_pass_under=MICRODUCK.height_clearance + 0.05, can_climb=0.03
            ),
            initial_safe_radius_meters=MICRODUCK.width_clearance + 0.15,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=MICRODUCK.width_clearance,
            robot_rotation_diameter=MICRODUCK.rotation_diameter,
            stuck_time_window=10.0,
            stuck_threshold=0.15,
        ),
        WorldControl.blueprint(),
        DuckExplorer.blueprint(
            min_frontier_perimeter=0.15,
            safe_distance=0.6,
            lookahead_distance=2.0,
            max_explored_distance=6.0,
            goal_timeout=45.0,
        ),
        DuckKnowledge.blueprint(
            rooms={name: scene.rooms[name] for name in ("football", "red_lockers", "blue_lockers")},
            objects={},
            scene=scene.id,
            places_db=str(directory / "places.db"),
            knowledge_dir=str(directory / "knowledge"),
        ),
        RobotMcpServer.blueprint(robot=robot, port=port),
    ]
    if os.environ.get("OPENAI_API_KEY"):
        modules.append(
            McpClient.blueprint(
                system_prompt=prompt,
                mcp_server_url=f"http://127.0.0.1:{port}/mcp",
                trace_dir=directory / "agent-traces",
            )
        )
    stack = (
        autoconnect(*modules)
        .remappings(
            [
                (VoxelGridMapper, "lidar", "pointcloud"),
            ]
        )
        .namespace(robot)
    )
    return autoconnect(stack, world_cockpit(robot, generation)).global_config(
        robot_model="microduck",
        nerf_speed=0.5,
        viewer="none",
        n_workers=4,
        mcp_port=port,
        tool_stream_topic=f"/{robot}/{generation}/tool_streams",
        listen_host="127.0.0.1",
    )
