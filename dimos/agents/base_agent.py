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

from langchain_core.messages.base import BaseMessage

from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out


class BaseAgentConfig(ModuleConfig):
    mcp_server_url: str = "http://localhost:9990/mcp"


class BaseAgent(Module):
    """Base for agents: the chat streams and an MCP client for calling skills."""

    config: BaseAgentConfig
    human_input: In[str]
    agent: Out[BaseMessage]
    agent_idle: Out[bool]

    mcp: McpClient

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.mcp = McpClient(self.config.mcp_server_url)

    @rpc
    def stop(self) -> None:
        self.mcp.close()
        super().stop()
