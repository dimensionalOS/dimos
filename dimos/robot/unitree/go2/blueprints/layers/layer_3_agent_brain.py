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

from typing import Any

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.unitree.go2.blueprints.layers.context_provider import _Go2ContextProvider
from dimos.robot.unitree.go2.blueprints.layers.expert_router import _Go2ExpertRouter


def _go2_agent_brain_with_client(**mcp_client_kwargs: Any) -> Blueprint:
    """Layer 3: expert routing, context, MCP tools, and the LLM/VLM agent."""
    return autoconnect(
        _Go2ExpertRouter.blueprint(),
        _Go2ContextProvider.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(**mcp_client_kwargs),
    )


_go2_agent_brain = _go2_agent_brain_with_client()

__all__ = ["_go2_agent_brain", "_go2_agent_brain_with_client"]
