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
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.manipulators.dual_openyam.blueprints.agentic import (
    DUAL_OPENYAM_AGENT_SYSTEM_PROMPT,
    dual_openyam_grasp_agent,
)
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import dual_openyam_grasp
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config


def test_agent_blueprint_adds_mcp_over_the_grasp_stack() -> None:
    modules = {atom.module for atom in dual_openyam_grasp_agent.active_blueprints}
    grasp_modules = {atom.module for atom in dual_openyam_grasp.active_blueprints}

    assert modules == grasp_modules | {McpServer, McpClient}
    assert PickAndPlaceModule in modules
    client = next(a for a in dual_openyam_grasp_agent.active_blueprints if a.module is McpClient)
    assert client.kwargs["system_prompt"] is DUAL_OPENYAM_AGENT_SYSTEM_PROMPT


def test_agent_prompt_names_every_planning_group() -> None:
    for group in dual_openyam_model_config().planning_groups:
        assert group.name in DUAL_OPENYAM_AGENT_SYSTEM_PROMPT
    assert "planning_group" in DUAL_OPENYAM_AGENT_SYSTEM_PROMPT
    assert DUAL_OPENYAM_AGENT_SYSTEM_PROMPT.isascii()
