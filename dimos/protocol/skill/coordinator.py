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
import json
import time
from typing import Any, Literal

from langchain_core.messages import ToolMessage
from langchain_core.tools import tool as langchain_tool
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.skill import SkillConfig, SkillContainer  # type: ignore[attr-defined]
from dimos.protocol.skill.type import MsgType, Output, Reducer, Return, SkillMsg, Stream
from dimos.protocol.skill.utils import interpret_tool_call_args
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class SkillCoordinatorConfig(ModuleConfig):
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


# This object maintains the state of a skill run on a caller end
class SkillState:
    call_id: str
    name: str
    state: SkillStateEnum
    skill_config: SkillConfig

    msg_count: int = 0
    sent_tool_msg: bool = False

    start_msg: SkillMsg[Literal[MsgType.start]] = None  # type: ignore[assignment]
    end_msg: SkillMsg[Literal[MsgType.ret]] = None  # type: ignore[assignment]
    error_msg: SkillMsg[Literal[MsgType.error]] = None  # type: ignore[assignment]
    ret_msg: SkillMsg[Literal[MsgType.ret]] = None  # type: ignore[assignment]
    reduced_stream_msg: list[SkillMsg[Literal[MsgType.reduced_stream]]] = None  # type: ignore[assignment]

    def __init__(self, call_id: str, name: str, skill_config: SkillConfig | None = None) -> None:
        super().__init__()

        self.skill_config = skill_config or SkillConfig(
            name=name,
            stream=Stream.none,
            ret=Return.none,
            reducer=Reducer.all,
            output=Output.standard,
            schema={},
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

    def content(self) -> dict[str, Any] | str | int | float | None:  # type: ignore[return]
        if self.state == SkillStateEnum.running:
            if self.reduced_stream_msg:
                return self.reduced_stream_msg.content  # type: ignore[attr-defined, no-any-return]

        if self.state == SkillStateEnum.completed:
            if self.reduced_stream_msg:  # are we a streaming skill?
                return self.reduced_stream_msg.content  # type: ignore[attr-defined, no-any-return]
            return self.ret_msg.content  # type: ignore[return-value]

        if self.state == SkillStateEnum.error:
            print("Error msg:", self.error_msg.content)
            if self.reduced_stream_msg:
                (self.reduced_stream_msg.content + "\n" + self.error_msg.content)  # type: ignore[attr-defined]
            else:
                return self.error_msg.content  # type: ignore[return-value]

    def agent_encode(self) -> ToolMessage | str:
        # tool call can emit a single ToolMessage
        # subsequent messages are considered SituationalAwarenessMessages,
        # those are collapsed into a HumanMessage, that's artificially prepended to history

        if not self.sent_tool_msg:
            self.sent_tool_msg = True
            return ToolMessage(
                self.content() or "Querying, please wait, you will receive a response soon.",  # type: ignore[arg-type]
                name=self.name,
                tool_call_id=self.call_id,
            )
        else:
            return json.dumps(
                {
                    "name": self.name,
                    "call_id": self.call_id,
                    "state": self.state.name,
                    "data": self.content(),
                    "ran_for": self.duration(),
                }
            )

    # returns True if the agent should be called for this message
    def handle_msg(self, msg: SkillMsg) -> bool:  # type: ignore[type-arg]
        self.msg_count += 1
        if msg.type == MsgType.stream:
            self.state = SkillStateEnum.running
            self.reduced_stream_msg = self.skill_config.reducer(self.reduced_stream_msg, msg)  # type: ignore[arg-type, assignment]

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

    def __str__(self) -> str:
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
class SkillCoordinator(Module):
    default_config = SkillCoordinatorConfig  # type: ignore[assignment]
    empty: bool = True

    _static_containers: list[SkillContainer]
    _dynamic_containers: list[SkillContainer]
    _skill_state: SkillStateDict  # key is call_id, not skill_name
    _skills: dict[str, SkillConfig]
    _updates_available: asyncio.Event | None
    _agent_loop: asyncio.AbstractEventLoop | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._static_containers = []
        self._dynamic_containers = []
        self._skills = {}
        self._skill_state = SkillStateDict()

        # Defer event creation until we're in the correct loop context
        self._updates_available = None
        self._agent_loop = None
        self._pending_notifications = 0  # Count pending notifications
        self._closed_coord = False
        self._transport_unsub_fn = None

    def _ensure_updates_available(self) -> asyncio.Event:
        """Lazily create the updates available event in the correct loop context."""
        if self._updates_available is None:
            # Create the event in the current running loop, not the stored loop
            try:
                loop = asyncio.get_running_loop()
                # print(f"[DEBUG] Creating _updates_available event in current loop {id(loop)}")
                # Always use the current running loop for the event
                # This ensures the event is created in the context where it will be used
                self._updates_available = asyncio.Event()
                # Store the loop where the event was created - this is the agent's loop
                self._agent_loop = loop
                # print(
                #    f"[DEBUG] Created _updates_available event {id(self._updates_available)} in agent loop {id(loop)}"
                # )
            except RuntimeError:
                # No running loop, defer event creation until we have the proper context
                # print(f"[DEBUG] No running loop, deferring event creation")
                # Don't create the event yet - wait for the proper loop context
                pass
        else:
            ...
            # print(f"[DEBUG] Reusing _updates_available event {id(self._updates_available)}")
        return self._updates_available  # type: ignore[return-value]

    @rpc
    def start(self) -> None:
        super().start()
        self.skill_transport.start()
        self._transport_unsub_fn = self.skill_transport.subscribe(self.handle_message)

    @rpc
    def stop(self) -> None:
        self._close_module()
        self._closed_coord = True
        self.skill_transport.stop()
        if self._transport_unsub_fn:
            self._transport_unsub_fn()

        # Stop all registered skill containers
        for container in self._static_containers:
            container.stop()
        for container in self._dynamic_containers:
            container.stop()

        super().stop()

    def len(self) -> int:
        return len(self._skills)

    def __len__(self) -> int:
        return self.len()

    # this can be converted to non-langchain json schema output
    # and langchain takes this output as well
    # just faster for now
    def get_tools(self) -> list[dict]:  # type: ignore[type-arg]
        return [
            langchain_tool(skill_config.f)  # type: ignore[arg-type, misc]
            for skill_config in self.skills().values()
            if not skill_config.hide_skill
        ]

    # internal skill call
    def call_skill(
        self, call_id: str | Literal[False], skill_name: str, args: dict[str, Any]
    ) -> None:
        if not call_id:
            call_id = str(time.time())
        skill_config = self.get_skill_config(skill_name)
        if not skill_config:
            logger.error(
                f"Skill {skill_name} not found in registered skills, but agent tried to call it (did a dynamic skill expire?)"
            )
            return

        self._skill_state[call_id] = SkillState(
            call_id=call_id, name=skill_name, skill_config=skill_config
        )

        # TODO agent often calls the skill again if previous response is still loading.
        # maybe create a new skill_state linked to a previous one? not sure

        arg_keywords = args.get("args") or {}
        arg_list = []

        if isinstance(arg_keywords, list):
            arg_list = arg_keywords
            arg_keywords = {}

        arg_list, arg_keywords = interpret_tool_call_args(args)

        return skill_config.call(  # type: ignore[no-any-return]
            call_id,
            *arg_list,
            **arg_keywords,
        )

    # Receives a message from active skill
    # Updates local skill state (appends to streamed data if needed etc)
    #
    # Checks if agent needs to be notified (if ToolConfig has Return=call_agent or Stream=call_agent)
    def handle_message(self, msg: SkillMsg) -> None:  # type: ignore[type-arg]
        if self._closed_coord:
            import traceback

            traceback.print_stack()
            return
        # logger.info(f"SkillMsg from {msg.skill_name}, {msg.call_id} - {msg}")

        if self._skill_state.get(msg.call_id) is None:
            logger.warn(
                f"Skill state for {msg.skill_name} (call_id={msg.call_id}) not found, (skill not called by our agent?) initializing. (message received: {msg})"
            )
            self._skill_state[msg.call_id] = SkillState(call_id=msg.call_id, name=msg.skill_name)

        should_notify = self._skill_state[msg.call_id].handle_msg(msg)

        if should_notify:
            updates_available = self._ensure_updates_available()
            if updates_available is None:
                print("[DEBUG] Event not created yet, deferring notification")
                return

            try:
                current_loop = asyncio.get_running_loop()
                agent_loop = getattr(self, "_agent_loop", self._loop)
                # print(
                #    f"[DEBUG] handle_message: current_loop={id(current_loop)}, agent_loop={id(agent_loop) if agent_loop else 'None'}, event={id(updates_available)}"
                # )
                if agent_loop and agent_loop != current_loop:
                    # print(
                    #    f"[DEBUG] Calling set() via call_soon_threadsafe from loop {id(current_loop)} to agent loop {id(agent_loop)}"
                    # )
                    agent_loop.call_soon_threadsafe(updates_available.set)
                else:
                    # print(f"[DEBUG] Calling set() directly in current loop {id(current_loop)}")
                    updates_available.set()
            except RuntimeError:
                # No running loop, use call_soon_threadsafe if we have an agent loop
                agent_loop = getattr(self, "_agent_loop", self._loop)
                # print(
                #    f"[DEBUG] No current running loop, agent_loop={id(agent_loop) if agent_loop else 'None'}"
                # )
                if agent_loop:
                    # print(
                    #    f"[DEBUG] Calling set() via call_soon_threadsafe to agent loop {id(agent_loop)}"
                    # )
                    agent_loop.call_soon_threadsafe(updates_available.set)
                else:
                    # print(f"[DEBUG] Event creation was deferred, can't notify")
                    pass

    def has_active_skills(self) -> bool:
        if not self.has_passive_skills():
            return False
        for skill_run in self._skill_state.values():
            # check if this skill will notify agent
            if skill_run.skill_config.ret == Return.call_agent:
                return True
            if skill_run.skill_config.stream == Stream.call_agent:
                return True
        return False

    def has_passive_skills(self) -> bool:
        # check if dict is empty
        if self._skill_state == {}:
            return False
        return True

    async def wait_for_updates(self, timeout: float | None = None) -> True:  # type: ignore[valid-type]
        """Wait for skill updates to become available.

        This method should be called by the agent when it's ready to receive updates.
        It will block until updates are available or timeout is reached.

        Args:
            timeout: Optional timeout in seconds

        Returns:
            True if updates are available, False on timeout
        """
        updates_available = self._ensure_updates_available()
        if updates_available is None:
            # Force event creation now that we're in the agent's loop context
            # print(f"[DEBUG] wait_for_updates: Creating event in current loop context")
            current_loop = asyncio.get_running_loop()
            self._updates_available = asyncio.Event()
            self._agent_loop = current_loop
            updates_available = self._updates_available
            # print(
            #    f"[DEBUG] wait_for_updates: Created event {id(updates_available)} in loop {id(current_loop)}"
            # )

        try:
            current_loop = asyncio.get_running_loop()

            # Double-check the loop context before waiting
            if self._agent_loop != current_loop:
                # print(f"[DEBUG] Loop context changed! Recreating event for loop {id(current_loop)}")
                self._updates_available = asyncio.Event()
                self._agent_loop = current_loop
                updates_available = self._updates_available

            # print(
            #    f"[DEBUG] wait_for_updates: current_loop={id(current_loop)}, event={id(updates_available)}, is_set={updates_available.is_set()}"
            # )
            if timeout:
                # print(f"[DEBUG] Waiting for event with timeout {timeout}")
                await asyncio.wait_for(updates_available.wait(), timeout=timeout)
            else:
                print("[DEBUG] Waiting for event without timeout")
                await updates_available.wait()
            print("[DEBUG] Event was set! Returning True")
            return True
        except asyncio.TimeoutError:
            print("[DEBUG] Timeout occurred while waiting for event")
            return False
        except RuntimeError as e:
            if "bound to a different event loop" in str(e):
                print(
                    "[DEBUG] Event loop binding error detected, recreating event and returning False to retry"
                )
                # Recreate the event in the current loop
                current_loop = asyncio.get_running_loop()
                self._updates_available = asyncio.Event()
                self._agent_loop = current_loop
                return False
            else:
                raise

    def generate_snapshot(self, clear: bool = True) -> SkillStateDict:
        """Generate a fresh snapshot of completed skills and optionally clear them."""
        ret = copy(self._skill_state)

        if clear:
            updates_available = self._ensure_updates_available()
            if updates_available is not None:
                # print(f"[DEBUG] generate_snapshot: clearing event {id(updates_available)}")
                updates_available.clear()
            else:
                ...
                # rint(f"[DEBUG] generate_snapshot: event not created yet, nothing to clear")
            to_delete = []
            # Since snapshot is being sent to agent, we can clear the finished skill runs
            for call_id, skill_run in self._skill_state.items():
                if skill_run.state == SkillStateEnum.completed:
                    logger.info(f"Skill {skill_run.name} (call_id={call_id}) finished")
                    to_delete.append(call_id)
                if skill_run.state == SkillStateEnum.error:
                    error_msg = skill_run.error_msg.content.get("msg", "Unknown error")  # type: ignore[union-attr]
                    error_traceback = skill_run.error_msg.content.get(  # type: ignore[union-attr]
                        "traceback", "No traceback available"
                    )

                    logger.error(
                        f"Skill error for {skill_run.name} (call_id={call_id}): {error_msg}"
                    )
                    print(error_traceback)
                    to_delete.append(call_id)

                elif (
                    skill_run.state == SkillStateEnum.running
                    and skill_run.reduced_stream_msg is not None
                ):
                    # preserve ret as a copy
                    ret[call_id] = copy(skill_run)
                    logger.debug(
                        f"Resetting accumulator for skill {skill_run.name} (call_id={call_id})"
                    )
                    skill_run.reduced_stream_msg = None  # type: ignore[assignment]

            for call_id in to_delete:
                logger.debug(f"Call {call_id} finished, removing from state")
                del self._skill_state[call_id]

        return ret

    def __str__(self) -> str:
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
    def register_skills(self, container: SkillContainer) -> None:
        self.empty = False
        if not container.dynamic_skills():
            logger.info(f"Registering static skill container, {container}")
            self._static_containers.append(container)
            for name, skill_config in container.skills().items():
                self._skills[name] = skill_config.bind(getattr(container, name))
        else:
            logger.info(f"Registering dynamic skill container, {container}")
            self._dynamic_containers.append(container)

    def get_skill_config(self, skill_name: str) -> SkillConfig | None:
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
