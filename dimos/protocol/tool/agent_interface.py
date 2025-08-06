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

from copy import copy
from dataclasses import dataclass
from enum import Enum
from pprint import pformat
from typing import Callable, Optional

from dimos.protocol.tool.comms import AgentMsg, LCMToolComms, MsgType, ToolCommsSpec
from dimos.protocol.tool.tool import ToolConfig, ToolContainer
from dimos.protocol.tool.types import Reducer, Return, Stream
from dimos.types.timestamped import TimestampedCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.tool.agent_input")


@dataclass
class AgentInputConfig:
    agent_comms: type[ToolCommsSpec] = LCMToolComms


class ToolStateEnum(Enum):
    pending = 0
    running = 1
    ret = 2
    error = 3


class ToolState(TimestampedCollection):
    name: str
    state: ToolStateEnum
    tool_config: ToolConfig

    def __init__(self, name: str, tool_config: Optional[ToolConfig] = None) -> None:
        super().__init__()
        if tool_config is None:
            self.tool_config = ToolConfig(
                name=name, stream=Stream.none, ret=Return.none, reducer=Reducer.none
            )
        else:
            self.tool_config = tool_config

        self.state = ToolStateEnum.pending
        self.name = name

    # returns True if the agent should be called for this message
    def handle_msg(self, msg: AgentMsg) -> bool:
        self.add(msg)

        if msg.type == MsgType.stream:
            if self.tool_config.stream == Stream.none or self.tool_config.stream == Stream.passive:
                return False
            if self.tool_config.stream == Stream.call_agent:
                return True

        if msg.type == MsgType.ret:
            self.state = ToolStateEnum.ret
            if self.tool_config.ret == Return.call_agent:
                return True
            return False

        if msg.type == MsgType.error:
            self.state = ToolStateEnum.error
            return True

        if msg.type == MsgType.start:
            self.state = ToolStateEnum.running
            return False

    def __str__(self) -> str:
        head = f"ToolState(state={self.state}"

        if self.state == ToolStateEnum.ret or self.state == ToolStateEnum.error:
            head += ", ran for="
        else:
            head += ", running for="

        head += f"{self.duration():.2f}s"

        if len(self):
            return head + f", messages={list(self._items)})"
        return head + ", No Messages)"


class AgentInterface(ToolContainer):
    _static_containers: list[ToolContainer]
    _dynamic_containers: list[ToolContainer]
    _tool_state: dict[str, ToolState]
    _tools: dict[str, ToolConfig]
    _agent_callback: Optional[Callable[[dict[str, ToolState]], any]] = None

    # agent callback is called with a state snapshot once system decides that agents needs
    # to be woken up
    def __init__(self, agent_callback: Callable[[dict[str, ToolState]], any] = None) -> None:
        super().__init__()
        self._agent_callback = agent_callback
        self._static_containers = []
        self._dynamic_containers = []
        self._tools = {}
        self._tool_state = {}

    def start(self) -> None:
        self.agent_comms.start()
        self.agent_comms.subscribe(self.handle_message)

    def stop(self) -> None:
        self.agent_comms.stop()

    # this is used by agent to call tools
    def execute_tool(self, tool_name: str, *args, **kwargs) -> None:
        tool_config = self.get_tool_config(tool_name)
        if not tool_config:
            logger.error(
                f"Tool {tool_name} not found in registered tools, but agent tried to call it (did a dynamic tool expire?)"
            )
            return

        # This initializes the tool state if it doesn't exist
        self._tool_state[tool_name] = ToolState(name=tool_name, tool_config=tool_config)
        return tool_config.call(*args, **kwargs)

    # updates local tool state (appends to streamed data if needed etc)
    # checks if agent needs to be called if AgentMsg has Return call_agent or Stream call_agent
    def handle_message(self, msg: AgentMsg) -> None:
        logger.info(f"Tool msg {msg}")

        if self._tool_state.get(msg.tool_name) is None:
            logger.warn(
                f"Tool state for {msg.tool_name} not found, (tool not called by our agent?) initializing. (message received: {msg})"
            )
            self._tool_state[msg.tool_name] = ToolState(name=msg.tool_name)

        should_call_agent = self._tool_state[msg.tool_name].handle_msg(msg)
        if should_call_agent:
            self.call_agent()

    # Returns a snapshot of the current state of tool runs.
    # If clear is True, it will assume the snapshot is being sent to an agent
    # and will clear the finished tool runs.
    def state_snapshot(self, clear: bool = True) -> dict[str, ToolState]:
        if not clear:
            return self._tool_state

        ret = copy(self._tool_state)

        to_delete = []
        # Since state is exported, we can clear the finished tool runs
        for tool_name, tool_run in self._tool_state.items():
            if tool_run.state == ToolStateEnum.ret:
                logger.info(f"Tool {tool_name} finished")
                to_delete.append(tool_name)
            if tool_run.state == ToolStateEnum.error:
                logger.error(f"Tool run error for {tool_name}")
                to_delete.append(tool_name)

        for tool_name in to_delete:
            logger.debug(f"Tool {tool_name} finished, removing from state")
            del self._tool_state[tool_name]

        return ret

    def call_agent(self) -> None:
        """Call the agent with the current state of tool runs."""
        logger.info(f"Calling agent with current tool state: {self.state_snapshot(clear=False)}")

        state = self.state_snapshot(clear=True)

        if self._agent_callback:
            self._agent_callback(state)

    def __str__(self):
        # Convert objects to their string representations
        def stringify_value(obj):
            if isinstance(obj, dict):
                return {k: stringify_value(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [stringify_value(item) for item in obj]
            else:
                return str(obj)

        ret = stringify_value(self._tool_state)

        return f"AgentInput({pformat(ret, indent=2, depth=3, width=120, compact=True)})"

    # Outputs data for the agent call
    # clears the local state (finished tool calls)
    def get_agent_query(self):
        return self.state_snapshot()

    # Given toolcontainers can run remotely, we are
    # caching available tools from static containers
    #
    # dynamic containers will be queried at runtime via
    # .tools() method
    def register_tools(self, container: ToolContainer):
        if not container.dynamic_tools:
            logger.info(f"Registering static tool container, {container}")
            self._static_containers.append(container)
            for name, tool_config in container.tools().items():
                self._tools[name] = tool_config.bind(getattr(container, name))
        else:
            logger.info(f"Registering dynamic tool container, {container}")
            self._dynamic_containers.append(container)

    def get_tool_config(self, tool_name: str) -> Optional[ToolConfig]:
        tool_config = self._tools.get(tool_name)
        if not tool_config:
            tool_config = self.tools().get(tool_name)
        return tool_config

    def tools(self) -> dict[str, ToolConfig]:
        # static container tooling is already cached
        all_tools: dict[str, ToolConfig] = {**self._tools}

        # Then aggregate tools from dynamic containers
        for container in self._dynamic_containers:
            for tool_name, tool_config in container.tools().items():
                all_tools[tool_name] = tool_config.bind(getattr(container, tool_name))

        return all_tools
