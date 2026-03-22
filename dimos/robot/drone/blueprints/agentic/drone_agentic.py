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

"""Agentic drone blueprint — autonomous drone with LLM agent control.

Composes on top of drone_basic (connection + camera + vis) and adds
tracking, mapping skills, and an LLM agent.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.google_maps_skill_container import GoogleMapsSkillContainer
from dimos.agents.skills.osm import OsmSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import autoconnect
from dimos.robot.drone.blueprints.basic.drone_basic import (
    drone_basic,
    drone_basic_gazebo,
    drone_basic_gazebo_spatial,
)
from dimos.robot.drone.drone_spatial_nav_skill import DroneSpatialNavSkill
from dimos.robot.drone.drone_tracking_module import DroneTrackingModule

DRONE_SYSTEM_PROMPT = """\
You control a drone over MAVLink (ArduPilot-compatible). Use the tool/schema names and parameters exactly as exposed.
Confirm actions and report results. For GPS missions (fly_to), use safe altitudes appropriate to the environment; do not invent extreme altitudes. Use is_flying_to_target to see if a fly_to is still active.

Motion (see each tool's Args): move (velocity, body NED: x right, y forward, z down m/s); move_by_distance (body displacement m → local NED setpoint when available, else timed forward velocity); go_to_position (absolute local NED: x North, y East, z Down — z negative means up);
rotate_to(heading_deg) for compass yaw (0° North, 90° East), takeoff, land, arm, disarm, set_mode.
Tracking/follow: follow_object (velocity via move_twist only — do not use move_by_distance for tracking). Perception: observe (camera frame). Maps/OSM: place and route tools when GPS or location context applies.
Spatial stack (drone-agentic-gazebo-spatial): navigate_to_where_i_saw(description) runs CLIP on stored views and sends a local NED position target; memory fills when the drone moves and TF+video are available.

Example GPS waypoints (San Francisco area):
6th and Natoma intersection: 37.78019978319006, -122.40770815020853,
454 Natoma (Office): 37.780967465525244, -122.40688342010769
5th and mission intersection: 37.782598539339695, -122.40649441875473
6th and mission intersection: 37.781007204789354, -122.40868447123661"""

drone_agentic = autoconnect(
    drone_basic,
    DroneTrackingModule.blueprint(outdoor=False),
    GoogleMapsSkillContainer.blueprint(),
    OsmSkill.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=DRONE_SYSTEM_PROMPT, model="gpt-4o"),
    WebInput.blueprint(),
).remappings(
    [
        (DroneTrackingModule, "video_input", "video"),
        (DroneTrackingModule, "cmd_vel", "movecmd_twist"),
    ]
)

drone_agentic_gazebo = autoconnect(
    drone_basic_gazebo,
    DroneTrackingModule.blueprint(outdoor=False),
    GoogleMapsSkillContainer.blueprint(),
    OsmSkill.blueprint(),
    Agent.blueprint(system_prompt=DRONE_SYSTEM_PROMPT, model="gpt-4o-mini"),
    WebInput.blueprint(),
).remappings(
    [
        (DroneTrackingModule, "video_input", "video"),
        (DroneTrackingModule, "cmd_vel", "movecmd_twist"),
    ]
)

drone_agentic_gazebo_spatial = autoconnect(
    drone_basic_gazebo_spatial,
    DroneTrackingModule.blueprint(outdoor=False),
    GoogleMapsSkillContainer.blueprint(),
    OsmSkill.blueprint(),
    DroneSpatialNavSkill.blueprint(),
    Agent.blueprint(system_prompt=DRONE_SYSTEM_PROMPT, model="gpt-4o-mini"),
    WebInput.blueprint(),
).remappings(
    [
        (DroneTrackingModule, "video_input", "video"),
        (DroneTrackingModule, "cmd_vel", "movecmd_twist"),
    ]
)

__all__ = [
    "DRONE_SYSTEM_PROMPT",
    "drone_agentic",
    "drone_agentic_gazebo",
    "drone_agentic_gazebo_spatial",
]
