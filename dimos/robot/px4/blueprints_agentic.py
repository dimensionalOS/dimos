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

"""The agentic PX4 blueprints: teleop plus an LLM agent that flies by the skills.

``px4-agentic``, ``px4-sitl-agentic``
    ``px4-teleop`` (or its simulator twin) with Px4SkillContainer, the MCP server and the
    agent: ``dimos agent-send "take off to 2 meters"``. Same composition as
    ``unitree_go2_agentic``.

Apart from ``blueprints.py`` because the agent needs the ``agents`` extra (langchain),
which the aircraft does not install for plain flight.
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.px4.blueprints import px4_sitl_teleop, px4_teleop
from dimos.robot.px4.skill_container import Px4SkillContainer

PX4_SYSTEM_PROMPT = """\
You fly a PX4 quadcopter for an operator who stands next to it with the RC transmitter.
Use the skills to do what the operator says, one flight command at a time, and report
each result in a sentence.

- Distances are metres. Directions are compass directions: south is a negative north_m,
  west is a negative east_m. Altitudes are above the takeoff point.
- "Take off to 2 meters" is takeoff(altitude_m=2). "Go 2 meters south at 3 m altitude"
  is go_to(north_m=-2, altitude_m=3). "Come back" is go_to(relative=False).
- A refused command names the reason; mode_not_offboard means the RC pilot has the
  aircraft. Tell the operator; never retry with different numbers on your own.
- The RC pilot can take over at any moment. If a skill reports that, stop and say so.
"""


_px4_agent = autoconnect(
    Px4SkillContainer.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=PX4_SYSTEM_PROMPT),
)

px4_agentic = autoconnect(px4_teleop, _px4_agent)
px4_sitl_agentic = autoconnect(px4_sitl_teleop, _px4_agent)
