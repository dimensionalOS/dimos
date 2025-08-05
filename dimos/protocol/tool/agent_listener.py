# Copyright 2025 Dimensional Inc.
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

from dataclasses import dataclass

from dimos.protocol.service import Service
from dimos.protocol.tool.comms import AgentMsg, LCMToolComms, ToolCommsSpec
from dimos.protocol.tool.tool import ToolContainer, ToolConfig


@dataclass
class AgentInputConfig:
    agent_comms: type[ToolCommsSpec] = LCMToolComms


class AgentInput(ToolContainer):
    running_tools: dict[str, ToolContainer] = {}

    def __init__(self) -> None:
        super().__init__()

    def start(self) -> None:
        self.agent_comms.start()
        self.agent_comms.subscribe(self.handle_message)

    def stop(self) -> None:
        self.agent_comms.stop()

    # updates local tool state (appends to streamed data if needed etc)
    # checks if agent needs to be called if AgentMsg has Return call_agent or Stream call_agent
    def handle_message(self, msg: AgentMsg) -> None:
        print(f"Received message: {msg}")

    def get_state(self): ...

    # outputs data for the agent call
    # clears the local state (finished tool calls)
    def agent_call(self): ...

    # outputs a list of tools that are registered
    # for the agent to introspect
    def get_tools(self): ...

    def register_tools(self, container: ToolContainer):
        for tool_name, tool in container.tools.items():
            print(f"Registering tool: {tool_name}, {tool}")

    @property
    def tools(self) -> dict[str, ToolConfig]:
        """Returns a dictionary of tools registered in this container."""
        # Aggregate all tools from registered containers
        all_tools: dict[str, ToolConfig] = {}
        for container_name, container in self.running_tools.items():
            for tool_name, tool_config in container.tools.items():
                all_tools[f"{container_name}.{tool_name}"] = tool_config
        return all_tools
