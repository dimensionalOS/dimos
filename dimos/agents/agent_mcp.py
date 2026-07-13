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

"""Agent-MCP: bare McpServer/McpClient pair for local MCP Q&A with an LLM.

Whatever `@skill`-decorated modules end up in this graph become MCP tools
automatically (see McpServer.on_system_modules) — this blueprint intentionally
carries none itself, so it stays a minimal connection point rather than
duplicating the skill coverage already exercised by the individual demo-* and
unitree-go2-*-agentic blueprints.

Pinned to port 9991 (instead of the repo-wide default 9990 that every other
agentic blueprint uses) so this can run alongside another MCP server — e.g. a
robot-commander blueprint in a sim — without a port clash. McpServer binds
`global_config.mcp_port`; McpClient's `mcp_server_url` has its own separate
default that doesn't follow `mcp_port` automatically, so both are set here to
keep server and client pointed at the same port.

Run with:

    uv run dimos run agent-mcp

Then attach an MCP client, e.g. Claude Code:

    claude mcp add --transport http --scope project dimos http://localhost:9991/mcp

To use a different port at launch time (e.g. to match wherever the other MCP
server landed), override both sides together:

    MCPCLIENT__MCP_SERVER_URL=http://localhost:<port>/mcp \\
        uv run dimos --mcp-port <port> run agent-mcp

See dimos/agents/mcp/README.md for the full MCP client/server flow.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect

AGENT_MCP_PORT = 9991

agent_mcp = autoconnect(
    McpServer.blueprint(),
    McpClient.blueprint(mcp_server_url=f"http://localhost:{AGENT_MCP_PORT}/mcp"),
).global_config(mcp_port=AGENT_MCP_PORT)
