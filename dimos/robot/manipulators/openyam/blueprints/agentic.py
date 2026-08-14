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

"""Agentic OpenYAM blueprints: manipulation skills exposed to an LLM agent.

Run against the real arm (macOS: DIMOS_TRANSPORT=lcm, OPENAI_API_KEY set):

    dimos run openyam-pickplace-can-agent

then talk to it from another terminal:

    dimos humancli
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.agent_prompts import (
    BASE_MANIPULATION_AGENT_SYSTEM_PROMPT,
)
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam,
    coordinator_openyam_can,
)
from dimos.robot.manipulators.openyam.blueprints.perception import openyam_pickplace

openyam_pickplace_can_agent = autoconnect(
    openyam_pickplace,
    coordinator_openyam_can,
    McpServer.blueprint(),
    # Claude via LangChain provider resolution; needs ANTHROPIC_API_KEY and
    # the langchain-anthropic package in the environment.
    McpClient.blueprint(
        system_prompt=BASE_MANIPULATION_AGENT_SYSTEM_PROMPT,
        model="anthropic:claude-sonnet-4-5",
    ),
)

openyam_pickplace_mock_agent = autoconnect(
    openyam_pickplace,
    coordinator_openyam,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=BASE_MANIPULATION_AGENT_SYSTEM_PROMPT),
)
