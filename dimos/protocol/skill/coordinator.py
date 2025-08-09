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

import asyncio
from copy import copy
from dataclasses import dataclass
from enum import Enum
from pprint import pformat, pprint
from typing import Any, List, Optional

from langchain_core.tools import tool as langchain_tool

from dimos.agents2 import ToolCall, ToolMessage

from dimos.protocol.skill.comms import LCMSkillComms, MsgType, SkillCommsSpec, SkillMsg
from dimos.protocol.skill.skill import SkillConfig, SkillContainer
from dimos.protocol.skill.type import Reducer, Return, Stream
from dimos.types.timestamped import TimestampedCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.skill.coordinator")


@dataclass
class AgentInputConfig:
    agent_comms: type[SkillCommsSpec] = LCMSkillComms


class SkillStateEnum(Enum):
    pending = 0
    running = 1
    completed = 2
    error = 3


# TODO pending timeout, running timeout, etc.
class SkillState(TimestampedCollection):
    call_id: str
    name: str
    state: SkillStateEnum
    skill_config: SkillConfig

    def __init__(self, call_id: str, name: str, skill_config: Optional[SkillConfig] = None) -> None:
        super().__init__()
        self.skill_config = skill_config or SkillConfig(
            name=name, stream=Stream.none, ret=Return.none, reducer=Reducer.none, schema={}
        )

        self.state = SkillStateEnum.pending
        self.call_id = call_id
        self.name = name

    def agent_encode(self) -> ToolMessage:
        last_msg = self._items[-1]
        return ToolMessage(last_msg.content, name=self.name, tool_call_id=self.call_id)

    # returns True if the agent should be called for this message
    def handle_msg(self, msg: SkillMsg) -> bool:
        self.add(msg)

        if msg.type == MsgType.stream:
            if (
                self.skill_config.stream == Stream.none
                or self.skill_config.stream == Stream.passive
            ):
                return False

            if self.skill_config.stream == Stream.call_agent:
                return True

        if msg.type == MsgType.ret:
            self.state = SkillStateEnum.completed
            if self.skill_config.ret == Return.call_agent:
                return True
            return False

        if msg.type == MsgType.error:
            self.state = SkillStateEnum.error
            return True

        if msg.type == MsgType.start:
            self.state = SkillStateEnum.running
            return False

        return False

    def __str__(self) -> str:
        head = f"SkillState({self.name} {self.state}, call_id={self.call_id}"

        if self.state == SkillStateEnum.completed or self.state == SkillStateEnum.error:
            head += ", ran for="
        else:
            head += ", running for="

        head += f"{self.duration():.2f}s"

        if len(self):
            return head + f", messages={list(self._items)})"
        return head + ", No Messages)"


SkillStates = dict[str, SkillState]


class SkillCoordinator(SkillContainer):
    empty: bool = True

    _static_containers: list[SkillContainer]
    _dynamic_containers: list[SkillContainer]
    _skill_state: dict[str, SkillState]  # key is call_id, not skill_name
    _skills: dict[str, SkillConfig]
    _updates_available: asyncio.Event
    _loop: Optional[asyncio.AbstractEventLoop]

    def __init__(self) -> None:
        super().__init__()
        self._static_containers = []
        self._dynamic_containers = []
        self._skills = {}
        self._skill_state = {}
        self._updates_available = asyncio.Event()
        self._loop = None

    def start(self) -> None:
        # Try to get the current event loop
        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:
            # No loop running, we'll set it when wait_for_updates is called
            pass
        self.agent_comms.start()
        self.agent_comms.subscribe(self.handle_message)

    def stop(self) -> None:
        self.agent_comms.stop()

    def len(self) -> int:
        return len(self._skills)

    def __len__(self) -> int:
        return self.len()

    # this can be converted to non-langchain json schema output
    # and langchain takes this output as well
    # just faster for now
    def get_tools(self) -> list[dict]:
        # return [skill.schema for skill in self.skills().values()]

        ret = []
        for name, skill_config in self.skills().items():
            # print(f"Tool {name} config: {skill_config}, {skill_config.f}")
            ret.append(langchain_tool(skill_config.f))

        return ret

    # Used by agent to execute tool calls
    def execute_tool_calls(self, tool_calls: List[ToolCall]) -> None:
        """Execute a list of tool calls from the agent."""
        for tool_call in tool_calls:
            logger.info(f"executing skill call {tool_call}")
            self.call(
                tool_call.get("id"),
                tool_call.get("name"),
                tool_call.get("args"),
            )

    # internal skill call
    def call(self, call_id: str, skill_name: str, args: dict[str, Any]) -> None:
        skill_config = self.get_skill_config(skill_name)
        if not skill_config:
            logger.error(
                f"Skill {skill_name} not found in registered skills, but agent tried to call it (did a dynamic skill expire?)"
            )
            return

        # This initializes the skill state if it doesn't exist
        self._skill_state[call_id] = SkillState(
            name=skill_name, skill_config=skill_config, call_id=call_id
        )
        return skill_config.call(call_id, *args.get("args", []), **args.get("kwargs", {}))

    # Receives a message from active skill
    # Updates local skill state (appends to streamed data if needed etc)
    #
    # Checks if agent needs to be notified (if ToolConfig has Return=call_agent or Stream=call_agent)
    def handle_message(self, msg: SkillMsg) -> None:
        logger.info(f"{msg.skill_name}, {msg.call_id} - {msg}")

        if self._skill_state.get(msg.call_id) is None:
            logger.warn(
                f"Skill state for {msg.skill_name} (call_id={msg.call_id}) not found, (skill not called by our agent?) initializing. (message received: {msg})"
            )
            self._skill_state[msg.call_id] = SkillState(call_id=msg.call_id, name=msg.skill_name)

        should_notify = self._skill_state[msg.call_id].handle_msg(msg)

        if should_notify:
            # Thread-safe way to set the event
            if self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(self._updates_available.set)
            else:
                # Fallback for when no loop is available
                self._updates_available.set()

    def has_active_skills(self) -> bool:
        # check if dict is empty
        if self._skill_state == {}:
            return False
        return True

    async def wait_for_updates(self, timeout: Optional[float] = None) -> True:
        """Wait for skill updates to become available.

        This method should be called by the agent when it's ready to receive updates.
        It will block until updates are available or timeout is reached.

        Args:
            timeout: Optional timeout in seconds

        Returns:
            True if updates are available, False on timeout
        """
        # Ensure we have the current event loop
        if not self._loop:
            self._loop = asyncio.get_running_loop()

        try:
            if timeout:
                await asyncio.wait_for(self._updates_available.wait(), timeout=timeout)
            else:
                await self._updates_available.wait()
            return True
        except asyncio.TimeoutError:
            return False

    def generate_snapshot(self, clear: bool = False) -> SkillStates:
        """Generate a fresh snapshot of completed skills and optionally clear them."""
        ret = copy(self._skill_state)

        if clear:
            self._updates_available.clear()
            to_delete = []
            # Since snapshot is being sent to agent, we can clear the finished skill runs
            for call_id, skill_run in self._skill_state.items():
                if skill_run.state == SkillStateEnum.completed:
                    logger.info(f"Skill {skill_run.name} (call_id={call_id}) finished")
                    to_delete.append(call_id)
                if skill_run.state == SkillStateEnum.error:
                    logger.error(f"Skill run error for {skill_run.name} (call_id={call_id})")
                    to_delete.append(call_id)

            for call_id in to_delete:
                logger.debug(f"Call {call_id} finished, removing from state")
                del self._skill_state[call_id]

        return ret

    def __str__(self):
        # Convert objects to their string representations
        def stringify_value(obj):
            if isinstance(obj, dict):
                return {k: stringify_value(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [stringify_value(item) for item in obj]
            else:
                return str(obj)

        ret = stringify_value(self._skill_state)

        return f"SkillCoordinator({pformat(ret, indent=2, depth=3, width=120, compact=True)})"

    # Given skillcontainers can run remotely, we are
    # Caching available skills from static containers
    #
    # Dynamic containers will be queried at runtime via
    # .skills() method
    def register_skills(self, container: SkillContainer):
        self.empty = False
        if not container.dynamic_skills:
            logger.info(f"Registering static skill container, {container}")
            self._static_containers.append(container)
            for name, skill_config in container.skills().items():
                self._skills[name] = skill_config.bind(getattr(container, name))
        else:
            logger.info(f"Registering dynamic skill container, {container}")
            self._dynamic_containers.append(container)

    def get_skill_config(self, skill_name: str) -> Optional[SkillConfig]:
        skill_config = self._skills.get(skill_name)
        if not skill_config:
            skill_config = self.skills().get(skill_name)
        return skill_config

    def skills(self) -> dict[str, SkillConfig]:
        # Static container skilling is already cached
        all_skills: dict[str, SkillConfig] = {**self._skills}

        # Then aggregate skills from dynamic containers
        for container in self._dynamic_containers:
            for skill_name, skill_config in container.skills().items():
                all_skills[skill_name] = skill_config.bind(getattr(container, skill_name))

        return all_skills
