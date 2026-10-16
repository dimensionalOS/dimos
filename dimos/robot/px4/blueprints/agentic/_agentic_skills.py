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

"""Agentic skills used by the PX4 agentic blueprints.

The agent needs the ``agents`` extra (langchain), which the aircraft does not install for
plain flight, so these live apart from the basic blueprints.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.px4.skill_container import Px4SkillContainer
from dimos.robot.px4.system_prompt import PX4_SYSTEM_PROMPT

_agentic_skills = autoconnect(
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=PX4_SYSTEM_PROMPT),
    Px4SkillContainer.blueprint(),
)
