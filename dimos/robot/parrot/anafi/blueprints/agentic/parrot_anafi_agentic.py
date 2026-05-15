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
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.parrot.anafi.blueprints.basic.parrot_anafi_basic import parrot_anafi_basic

parrot_anafi_agentic = autoconnect(
    parrot_anafi_basic,
    McpServer.blueprint(),
    McpClient.blueprint(),
    WebInput.blueprint(),
)

__all__ = [
    "parrot_anafi_agentic",
]
