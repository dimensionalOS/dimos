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

from dimos.utils.generic import connect_host

DEFAULT_MCP_PORT = 9990
DEFAULT_MCP_TIMEOUT = 30
DEFAULT_MCP_URL = f"http://localhost:{DEFAULT_MCP_PORT}/mcp"

# `HumanMessage.additional_kwargs` key naming the tool a message on McpClient's
# `agent` stream came from. A tool's image result is fed to the model as a user
# turn (a tool response cannot carry images for OpenAI models), and chat UIs read
# this key to render that turn as tool output rather than as typed input.
TOOL_ORIGIN_KEY = "tool_origin"


def mcp_url(host: str, port: int) -> str:
    """Convert a local server bind address into a dialable MCP URL."""
    return f"http://{connect_host(host)}:{port}/mcp"
