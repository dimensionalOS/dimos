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


"""MicroDuck in the browser: the official-policy sim (simulation.py) plus the
mapping/planning stack and a three-panel cockpit - camera, click-to-goal map,
keyboard teleop. `microduck-agentic-cockpit` adds the agent chat.

    uv run dimos --simulation mujoco run microduck-cockpit --local-relay
    uv run dimos --simulation mujoco run microduck-agentic-cockpit --local-relay
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.agents.voice_input import VoiceInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pSHMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.pollen.microduck.blueprints.simulation import (
    _MicroDuckCoordinator,
    microduck_stack,
)
from dimos.robot.pollen.microduck.skills import MicroduckSkills
from dimos.stream.audio.decode import ffmpeg_requirement
from dimos.web.cockpit import Chat, Col, Map2D, Row, Teleop, Video, cockpit

# Footprint for the planner: a 14 cm wide, 26 cm tall duck plus margin.
_WIDTH = 0.2
_ROTATION_DIAMETER = 0.3
_HEIGHT = 0.28

_nav = autoconnect(
    microduck_stack({"enable_pointcloud": True, "pointcloud_fps": 5.0}),
    VoxelGridMapper.blueprint(voxel_size=0.03, device="CPU:0"),
    CostMapper.blueprint(
        config=HeightCostConfig(resolution=0.03, can_pass_under=_HEIGHT, can_climb=0.03)
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=_WIDTH, robot_rotation_diameter=_ROTATION_DIAMETER
    ),
    MovementManager.blueprint(),
).remappings(
    [
        (VoxelGridMapper, "lidar", "pointcloud"),
        (_MicroDuckCoordinator, "twist_command", "cmd_vel"),
    ]
)

_panels = Row(
    Video("color_image", title="Duck cam"),
    Col(
        Map2D(goal="goal_request", title="Map (click to set a goal)"),
        Teleop(max_linear=0.4, max_angular=1.0, title="Keyboard teleop"),
        shares=[3, 1],
    ),
    shares=[2, 1],
)

microduck_cockpit = (
    autoconnect(_nav, cockpit(layout=_panels))
    .transports({("pointcloud", PointCloud2): pSHMTransport("/microduck/pointcloud")})
    .global_config(robot_model="microduck", simulation="mujoco", n_workers=6)
)

_SYSTEM_PROMPT = """\
You are the brain of MicroDuck, a 25 cm tall two-legged duck robot in a
simulated room. You walk about 0.1 m/s, so crossing the room takes a while.
Use where_am_i to orient yourself, go_to(x, y) to walk somewhere, stop_moving
to halt, observe to look through the head camera, and perform(name) for tricks
(list_tricks). Sit only when asked; stand_up before walking. Keep answers
short and playful."""

microduck_agentic_cockpit = (
    autoconnect(
        _nav,
        MicroduckSkills.blueprint(),
        ObserveSkill.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(system_prompt=_SYSTEM_PROMPT),
        VoiceInput.blueprint(),
        cockpit(layout=Row(_panels, Chat(title="Agent chat"), shares=[3, 1])),
    )
    .transports({("pointcloud", PointCloud2): pSHMTransport("/microduck/pointcloud")})
    .requirements(ffmpeg_requirement)
    .global_config(robot_model="microduck", simulation="mujoco", n_workers=8)
)
