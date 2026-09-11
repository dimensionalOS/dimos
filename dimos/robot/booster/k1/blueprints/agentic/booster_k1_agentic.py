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

"""Booster K1 agentic blueprint: spatial + agent + skills."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.booster.k1.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.booster.k1.blueprints.smart.booster_k1_spatial import booster_k1_spatial

booster_k1_agentic = autoconnect(
    booster_k1_spatial,
    McpServer.blueprint(),
    McpClient.blueprint(
        system_prompt=(
            "You operate a Booster K1 humanoid robot. Use only the tools advertised by "
            "the MCP server. The robot provides a camera and direct velocity commands, "
            "with stand and lie-down skills. Odometry, lidar, pointclouds, and autonomous "
            "navigation are not available from this connection. Spatial memory requires "
            "an external world-to-base transform. Do not claim to measure travel distance "
            "or completed motion from command acceptance. Use short, bounded walks and "
            "change posture only when explicitly requested."
        ),
    ),
    _common_agentic,
)

__all__ = ["booster_k1_agentic"]
