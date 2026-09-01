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

"""Agentic xArm manipulation blueprints."""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.common.agent_prompts import (
    BASE_MANIPULATION_AGENT_SYSTEM_PROMPT,
    MANIPULATION_AGENT_SYSTEM_PROMPT,
)
from dimos.robot.manipulators.xarm.blueprints.basic import xarm7_planner_coordinator
from dimos.robot.manipulators.xarm.blueprints.grasp import xarm_grasp
from dimos.robot.manipulators.xarm.blueprints.simulation import xarm_perception_sim

xarm7_planner_coordinator_agent = autoconnect(
    xarm7_planner_coordinator,
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=BASE_MANIPULATION_AGENT_SYSTEM_PROMPT),
)

# The stack already carries a detector and a segmenter; adding the agent on top
# packs them into one worker, where the detector's lazy transformers import fails
# outright, so give them room.
xarm_grasp_agent = autoconnect(
    xarm_grasp,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MANIPULATION_AGENT_SYSTEM_PROMPT),
).global_config(n_workers=6)

xarm_perception_sim_agent = autoconnect(
    xarm_perception_sim,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MANIPULATION_AGENT_SYSTEM_PROMPT),
)
