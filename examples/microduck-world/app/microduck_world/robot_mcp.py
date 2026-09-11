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

"""One stock MCP server per robot process, with an explicit local tool boundary."""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.rpc_client import RPCClient


class RobotMcpConfig(ModuleConfig):
    robot: str
    port: int


class RobotMcpServer(McpServer):
    config: RobotMcpConfig
    dedicated_worker = True

    def _start_server(self, port: int | None = None) -> None:
        super()._start_server(self.config.port)

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        prefix = self.config.robot + "/"
        own = [module for module in modules if module.remote_name.startswith(prefix)]
        # The framework's generic agent_send skill targets a global human_input
        # topic. Each browser already has its own scoped input, so do not expose
        # the server's operator/introspection skills to robot agents.
        own = [module for module in own if not issubclass(module.actor_class, McpServer)]
        super().on_system_modules(own)
