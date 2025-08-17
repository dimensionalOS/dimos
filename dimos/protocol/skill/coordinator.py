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
import time
from copy import copy
from dataclasses import dataclass
from enum import Enum
from typing import Any, List, Literal, Optional

from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    MessageLikeRepresentation,
    SystemMessage,
    ToolCall,
    ToolMessage,
)
from langchain_core.tools import tool as langchain_tool
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.core import rpc
from dimos.core.module import get_loop
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.skill import SkillConfig, SkillContainer
from dimos.protocol.skill.type import MsgType, Reducer, Return, SkillMsg, Stream
from dimos.types.timestamped import TimestampedCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.skill.coordinator")


@dataclass
class SkillCoordinatorConfig:
    skill_transport: type[SkillCommsSpec] = LCMSkillComms


class SkillStateEnum(Enum):
    pending = 0
    running = 1
    completed = 2
    error = 3

    def colored_name(self) -> Text:
        """Return the state name as a rich Text object with color."""
        colors = {
            SkillStateEnum.pending: "yellow",
            SkillStateEnum.running: "blue",
            SkillStateEnum.completed: "green",
            SkillStateEnum.error: "red",
        }
        return Text(self.name, style=colors.get(self, "white"))


# TODO pending timeout, running timeout, etc.
#
# This object maintains the state of a skill run on a caller end
class SkillState:
    call_id: str
    name: str
    state: SkillStateEnum
    skill_config: SkillConfig

    msg_count: int = 0

    start_msg: SkillMsg[Literal[MsgType.start]] = None
    end_msg: SkillMsg[Literal[MsgType.ret]] = None
    error_msg: SkillMsg[Literal[MsgType.error]] = None
    ret_msg: SkillMsg[Literal[MsgType.ret]] = None
    reduced_stream_msg: List[SkillMsg[Literal[MsgType.reduced_stream]]] = None

    def __init__(self, call_id: str, name: str, skill_config: Optional[SkillConfig] = None) -> None:
        super().__init__()

        self.skill_config = skill_config or SkillConfig(
            name=name, stream=Stream.none, ret=Return.none, reducer=Reducer.all, schema={}
        )

        self.state = SkillStateEnum.pending
        self.call_id = call_id
        self.name = name

    def duration(self) -> float:
        """Calculate the duration of the skill run."""
        if self.start_msg and self.end_msg:
            return self.end_msg.ts - self.start_msg.ts
        elif self.start_msg:
            return time.time() - self.start_msg.ts
        else:
            return 0.0

    def agent_encode(self) -> ToolMessage:
        agent_data = {"state": self.state.name, "ran_for": f"{round(self.duration())} seconds"}

        if self.state == SkillStateEnum.running:
            if self.reduced_stream_msg:
                agent_data["stream_data"] = self.reduced_stream_msg.content

        if self.state == SkillStateEnum.completed:
            if self.reduced_stream_msg:
                agent_data["return_value"] = self.reduced_stream_msg.content
            else:
                agent_data["return_value"] = self.ret_msg.content

        if self.state == SkillStateEnum.error:
            agent_data["return_value"] = self.error_msg.content
            if self.reduced_stream_msg:
                agent_data["stream_data"] = self.reduced_stream_msg.content

        if self.error_msg:
            if self.reduced_stream_msg:
                agent_data["stream_data"] = self.reduced_stream_msg.content
            agent_data["error"] = {
                "msg": self.error_msg.content.get("msg", "Unknown error"),
                "traceback": self.error_msg.content.get("traceback", "No traceback available"),
            }

        return ToolMessage(agent_data, name=self.name, tool_call_id=self.call_id)

    # returns True if the agent should be called for this message
    def handle_msg(self, msg: SkillMsg) -> bool:
        self.msg_count += 1
        if msg.type == MsgType.stream:
            self.reduced_stream_msg = self.skill_config.reducer(self.reduced_stream_msg, msg)

            if (
                self.skill_config.stream == Stream.none
                or self.skill_config.stream == Stream.passive
            ):
                return False

            if self.skill_config.stream == Stream.call_agent:
                return True

        if msg.type == MsgType.ret:
            self.state = SkillStateEnum.completed
            self.ret_msg = msg
            if self.skill_config.ret == Return.call_agent:
                return True
            return False

        if msg.type == MsgType.error:
            self.state = SkillStateEnum.error
            self.error_msg = msg
            return True

        if msg.type == MsgType.start:
            self.state = SkillStateEnum.running
            self.start_msg = msg
            return False

        return False

    def __len__(self) -> int:
        return self.msg_count

    def __str__(self) -> str:
        # For standard string representation, we'll use rich's Console to render the colored text
        console = Console(force_terminal=True, legacy_windows=False)
        colored_state = self.state.colored_name()

        # Build the parts of the string
        parts = [Text(f"SkillState({self.name} "), colored_state, Text(f", call_id={self.call_id}")]

        if self.state == SkillStateEnum.completed or self.state == SkillStateEnum.error:
            parts.append(Text(", ran for="))
        else:
            parts.append(Text(", running for="))

        parts.append(Text(f"{self.duration():.2f}s"))

        if len(self):
            parts.append(Text(f", msg_count={self.msg_count})"))
        else:
            parts.append(Text(", No Messages)"))

        # Combine all parts into a single Text object
        combined = Text()
        for part in parts:
            combined.append(part)

        # Render to string with console
        with console.capture() as capture:
            console.print(combined, end="")
        return capture.get()


# subclassed the dict just to have a better string representation
class SkillStateDict(dict[str, SkillState]):
    """Custom dict for skill states with better string representation."""

    def agent_encode(self) -> list[ToolMessage]:
        """Encode all skill states into a list of ToolMessages for the agent."""
        return [skill_state.agent_encode() for skill_state in self.values()]

    def table(self) -> Table:
        # Add skill states section
        states_table = Table(show_header=True)
        states_table.add_column("Call ID", style="dim", width=12)
        states_table.add_column("Skill", style="white")
        states_table.add_column("State", style="white")
        states_table.add_column("Duration", style="yellow")
        states_table.add_column("Messages", style="dim")

        for call_id, skill_state in self.items():
            # Get colored state name
            state_text = skill_state.state.colored_name()

            # Duration formatting
            if (
                skill_state.state == SkillStateEnum.completed
                or skill_state.state == SkillStateEnum.error
            ):
                duration = f"{skill_state.duration():.2f}s"
            else:
                duration = f"{skill_state.duration():.2f}s..."

            # Messages info
            msg_count = str(len(skill_state))

            states_table.add_row(
                call_id[:8] + "...", skill_state.name, state_text, duration, msg_count
            )

        if not self:
            states_table.add_row("", "[dim]No active skills[/dim]", "", "", "")
        return states_table

    def __str__(self):
        console = Console(force_terminal=True, legacy_windows=False)

        # Render to string with title above
        with console.capture() as capture:
            console.print(Text("  SkillState", style="bold blue"))
            console.print(self.table())
        return capture.get().strip()


# This class is responsible for managing the lifecycle of skills,
# handling skill calls, and coordinating communication between the agent and skills.
#
# It aggregates skills from static and dynamic containers, manages skill states,
# and decides when to notify the agent about updates.
class SkillCoordinator(SkillContainer):
    default_config = SkillCoordinatorConfig
    empty: bool = True

    _static_containers: list[SkillContainer]
    _dynamic_containers: list[SkillContainer]
    _skill_state: SkillStateDict  # key is call_id, not skill_name
    _skills: dict[str, SkillConfig]
    _updates_available: asyncio.Event
    _loop: Optional[asyncio.AbstractEventLoop]

    def __init__(self) -> None:
        SkillContainer.__init__(self)
        self._loop = get_loop()
        self._static_containers = []
        self._dynamic_containers = []
        self._skills = {}
        self._skill_state = SkillStateDict()
        self._updates_available = asyncio.Event()

    @rpc
    def start(self) -> None:
        self.skill_transport.start()
        self.skill_transport.subscribe(self.handle_message)

    @rpc
    def stop(self) -> None:
        self.skill_transport.stop()

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
            self.call_skill(
                tool_call.get("id"),
                tool_call.get("name"),
                tool_call.get("args"),
            )

    # internal skill call
    def call_skill(self, call_id: str, skill_name: str, args: dict[str, Any]) -> None:
        skill_config = self.get_skill_config(skill_name)
        if not skill_config:
            logger.error(
                f"Skill {skill_name} not found in registered skills, but agent tried to call it (did a dynamic skill expire?)"
            )
            return

        # This initializes the skill state if it doesn't exist
        self._skill_state[call_id] = SkillState(
            call_id=call_id, name=skill_name, skill_config=skill_config
        )

        return skill_config.call(call_id, *args.get("args", []), **args.get("kwargs", {}))

    # Receives a message from active skill
    # Updates local skill state (appends to streamed data if needed etc)
    #
    # Checks if agent needs to be notified (if ToolConfig has Return=call_agent or Stream=call_agent)
    def handle_message(self, msg: SkillMsg) -> None:
        logger.info(f"SkillMsg from {msg.skill_name}, {msg.call_id} - {msg}")

        if self._skill_state.get(msg.call_id) is None:
            logger.warn(
                f"Skill state for {msg.skill_name} (call_id={msg.call_id}) not found, (skill not called by our agent?) initializing. (message received: {msg})"
            )
            self._skill_state[msg.call_id] = SkillState(call_id=msg.call_id, name=msg.skill_name)

        should_notify = self._skill_state[msg.call_id].handle_msg(msg)

        if should_notify:
            self._loop.call_soon_threadsafe(self._updates_available.set)

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
        try:
            if timeout:
                await asyncio.wait_for(self._updates_available.wait(), timeout=timeout)
            else:
                await self._updates_available.wait()
            return True
        except asyncio.TimeoutError:
            return False

    def generate_snapshot(self, clear: bool = True) -> SkillStateDict:
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
                    error_msg = skill_run.error_msg.content.get("msg", "Unknown error")
                    error_traceback = skill_run.error_msg.content.get(
                        "traceback", "No traceback available"
                    )

                    logger.error(
                        f"Skill error for {skill_run.name} (call_id={call_id}): {error_msg}"
                    )
                    print(error_traceback)
                    to_delete.append(call_id)

            for call_id in to_delete:
                logger.debug(f"Call {call_id} finished, removing from state")
                del self._skill_state[call_id]

        return ret

    def __str__(self):
        console = Console(force_terminal=True, legacy_windows=False)

        # Create main table without any header
        table = Table(show_header=False)

        # Add containers section
        containers_table = Table(show_header=True, show_edge=False, box=None)
        containers_table.add_column("Type", style="cyan")
        containers_table.add_column("Container", style="white")

        # Add static containers
        for container in self._static_containers:
            containers_table.add_row("Static", str(container))

        # Add dynamic containers
        for container in self._dynamic_containers:
            containers_table.add_row("Dynamic", str(container))

        if not self._static_containers and not self._dynamic_containers:
            containers_table.add_row("", "[dim]No containers registered[/dim]")

        # Add skill states section
        states_table = self._skill_state.table()
        states_table.show_edge = False
        states_table.box = None

        # Combine into main table
        table.add_column("Section", style="bold")
        table.add_column("Details", style="none")
        table.add_row("Containers", containers_table)
        table.add_row("Skills", states_table)

        # Render to string with title above
        with console.capture() as capture:
            console.print(Text("  SkillCoordinator", style="bold blue"))
            console.print(table)
        return capture.get().strip()

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
