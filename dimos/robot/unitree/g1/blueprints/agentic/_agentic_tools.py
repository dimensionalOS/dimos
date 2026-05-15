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

"""Agentic tools used by higher-level G1 blueprints."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.tools.navigation import NavigationToolContainer
from dimos.agents.tools.speak_tool import SpeakTool
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.system_prompt import G1_SYSTEM_PROMPT
from dimos.robot.unitree.g1.tool_container import UnitreeG1ToolContainer

_agentic_tools = autoconnect(
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT),
    NavigationToolContainer.blueprint(),
    SpeakTool.blueprint(),
    UnitreeG1ToolContainer.blueprint(),
)

__all__ = ["_agentic_tools"]
