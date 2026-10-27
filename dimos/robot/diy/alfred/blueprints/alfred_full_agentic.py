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

"""Alfred, the whole robot with its camera, plus an LLM agent that can drive, navigate, look and
plan the arms and lift from a sentence.

    OPENAI_API_KEY=... dimos run alfred-full-agentic
    dimos agent-send "go to 2, 1"          # or the Desktop chat, or any MCP client on :9990

Layered on alfred-full-perception. McpServer exposes every @skill in the stack (AlfredHighLevel's
``move_velocity``, the pillar's ``home``/``set_position``, PointNav's ``go_to``/``stop_navigation``,
ObserveSkill's ``observe``, ManipulationSkills' ``go_home``/``move_to_pose``/...); McpClient is the
agent on gpt-5.6-luna with Alfred's prompt. PointNav's goal is the same ``clicked_point`` the
Controller's "Go here" publishes, and its NaN cancel is the planner's cancel. The agent runs no
``explore`` yet: that needs a 2D costmap, the frontier explorer and a goal-reached signal from the
follower.
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.agents.skills.point_nav import PointNavSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.diy.alfred.blueprints.alfred_full import BASE_FRAME, ODOM_FRAME
from dimos.robot.diy.alfred.blueprints.alfred_full_perception import alfred_full_perception
from dimos.robot.diy.alfred.system_prompt import ALFRED_SYSTEM_PROMPT

alfred_full_agentic = autoconnect(
    alfred_full_perception,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=ALFRED_SYSTEM_PROMPT),
    PointNavSkillContainer.blueprint(world_frame=ODOM_FRAME, base_frame=BASE_FRAME).remappings(
        [(PointNavSkillContainer, "goal", "clicked_point")]
    ),
    ManipulationSkills.blueprint(),
    ObserveSkill.blueprint(),
).global_config(n_workers=16, robot_model="alfred")
