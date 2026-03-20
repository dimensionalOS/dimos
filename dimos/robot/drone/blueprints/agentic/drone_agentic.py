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

from dimos.agents.agent import Agent
from dimos.agents.skills.google_maps_skill_container import GoogleMapsSkillContainer
from dimos.agents.skills.osm import OsmSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import autoconnect
from dimos.robot.drone.blueprints.basic.drone_basic import drone_basic
from dimos.robot.drone.drone_tracking_module import DroneTrackingModule

DRONE_SYSTEM_PROMPT = """\
You are controlling a DJI drone with MAVLink interface.
You have access to drone control skills you are already flying so only run move_twist, set_mode, and fly_to.
When the user gives commands, use the appropriate skills to control the drone.
Always confirm actions and report results. Send fly_to commands only at above 200 meters altitude to be safe.
Here are some GPS locations to remember
6th and Natoma intersection: 37.78019978319006, -122.40770815020853,
454 Natoma (Office): 37.780967465525244, -122.40688342010769
5th and mission intersection: 37.782598539339695, -122.40649441875473
6th and mission intersection: 37.781007204789354, -122.40868447123661"""

drone_agentic = autoconnect(
    drone_basic,
    DroneTrackingModule.blueprint(outdoor=False),
    GoogleMapsSkillContainer.blueprint(),
    OsmSkill.blueprint(),
    Agent.blueprint(system_prompt=DRONE_SYSTEM_PROMPT, model="gpt-4o"),
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
]
