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

"""DimSim agentic blueprint — spatial + MCP agent + skills + human input."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.robot.sim.blueprints.nav.sim_spatial import sim_spatial
from dimos.robot.sim.tf_module import _camera_info_static

sim_agentic = autoconnect(
    sim_spatial,
    McpServer.blueprint(),
    McpClient.blueprint(),
    NavigationSkillContainer.blueprint(),
    PersonFollowSkillContainer.blueprint(camera_info=_camera_info_static()),
    PatrollingModule.blueprint(),
    WebInput.blueprint(),
    SpeakSkill.blueprint(),
)

__all__ = ["sim_agentic"]
