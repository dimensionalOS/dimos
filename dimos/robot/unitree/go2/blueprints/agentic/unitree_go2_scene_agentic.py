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

"""Go2 scene understanding exposed over MCP — a test harness for scene skills.

Extends `unitree-go2-scene` (ObjectSceneRegistrationModule in lidar mode:
detect -> locate -> ObjectDB, plus the contextual-labeling skills) with:

- `McpServer`: every `@skill` in the graph (detect, list_observed_items,
  identify_object, refine_observed_labels, ...) becomes an MCP tool at
  http://localhost:9990/mcp. Attach an MCP client to drive the skills
  interactively (see dimos/agents/mcp/README.md):

      claude mcp add --transport http --scope project dimos http://localhost:9990/mcp

- `McpClient`: an in-graph LLM agent (needs OPENAI_API_KEY) consuming the
  same tools, so the skills can also be exercised with natural language via
  the `agent_send` tool.

Run with:

    uv run dimos run unitree-go2-scene-agentic
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_scene import unitree_go2_scene

unitree_go2_scene_agentic = autoconnect(
    unitree_go2_scene,
    McpServer.blueprint(),
    McpClient.blueprint(),
)
