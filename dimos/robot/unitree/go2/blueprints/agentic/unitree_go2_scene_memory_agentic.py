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

"""Scene + spatial-memory catalog exposed over MCP and an in-graph agent.

Extends `unitree-go2-scene-memory` (object detection + spatial memory, fused
in `catalog_scene`) with an `McpServer` (every `@skill` becomes an MCP tool at
http://localhost:9990/mcp) and an `McpClient` LLM agent (needs OPENAI_API_KEY)
so the combined catalog can be driven with natural language.

Run with:

    uv run dimos run unitree-go2-scene-memory-agentic
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_scene_memory import (
    unitree_go2_scene_memory,
)
from dimos.skills.mapping.floorplan_skill import FloorplanSkillContainer
from dimos.skills.mapping.lidar_signal_skills import LidarSignalSkills

unitree_go2_scene_memory_agentic = autoconnect(
    unitree_go2_scene_memory,
    FloorplanSkillContainer.blueprint(),
    LidarSignalSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(),
)
