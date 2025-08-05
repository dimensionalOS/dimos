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
from dimos.protocol.tool.comms import AgentMsg, LCMToolComms, MsgType, ToolCommsSpec
from dimos.protocol.tool.tool import ToolConfig, ToolContainer
from dimos.protocol.tool.types import Stream
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.tool.agent_input")


@dataclass
class AgentInputConfig:
    agent_comms: type[ToolCommsSpec] = LCMToolComms


class AgentInput(ToolContainer):
    _static_containers: list[ToolContainer]
    _dynamic_containers: list[ToolContainer]
    _tool_state: dict[str, list[AgentMsg]]
    _tools: dict[str, ToolConfig]

    def __init__(self) -> None:
        super().__init__()
        self._static_containers = []
        self._dynamic_containers = []
        self._tools = {}
        self._tool_state = {}

    def start(self) -> None:
        self.agent_comms.start()
        self.agent_comms.subscribe(self.handle_message)

    def stop(self) -> None:
        self.agent_comms.stop()

    # updates local tool state (appends to streamed data if needed etc)
    # checks if agent needs to be called if AgentMsg has Return call_agent or Stream call_agent
    def handle_message(self, msg: AgentMsg) -> None:
        print("AgentInput received message", msg)
        self.update_state(msg.tool_name, msg)

    def update_state(self, tool_name: str, msg: AgentMsg) -> None:
        if tool_name not in self._tool_state:
            self._tool_state[tool_name] = []
        self._tool_state[tool_name].append(msg)

        # we check if message should trigger an agent call
        if self.should_call_agent(msg):
            self.call_agent()

    def should_call_agent(self, msg) -> bool:
        tool_config = self._tools.get(msg.tool_name)
        if not tool_config:
            logger.warning(
                f"Tool {msg.tool_name} not found in registered tools but tool message received {msg}"
            )
            return False

        if msg.type == MsgType.start:
            return False

        if msg.type == MsgType.stream:
            if tool_config.stream == Stream.none or tool_config.stream == Stream.passive:
                return False
            if tool_config.stream == Stream.call_agent:
                return True

    def collect_state(self):
        ...
        # return {"tool_name": {"state": tool_state, messages: list[AgentMsg]}}

    # outputs data for the agent call
    # clears the local state (finished tool calls)
    def get_agent_query(self):
        state = self.collect_state()
        ...

    # given toolcontainers can run remotely, we are
    # caching available tools from static containers
    #
    # dynamic containers will be queried at runtime via
    # .tools() method
    def register_tools(self, container: ToolContainer):
        print("registering tool container", container)
        if not container.dynamic_tools:
            self._static_containers.append(container)
            for name, tool_config in container.tools().items():
                self._tools[name] = tool_config
        else:
            self._dynamic_containers.append(container)

    def tools(self) -> dict[str, ToolConfig]:
        # static container tooling is already cached
        all_tools: dict[str, ToolConfig] = {**self._tools}

        # Then aggregate tools from dynamic containers
        for container in self._dynamic_containers:
            for tool_name, tool_config in container.tools().items():
                all_tools[tool_name] = tool_config

        return all_tools
