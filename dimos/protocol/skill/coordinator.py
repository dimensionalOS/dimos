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
import threading
import time
from typing import Annotated, Any, Literal

from annotated_doc import Doc
from langchain_core.messages import ToolMessage
from langchain_core.tools import tool as langchain_tool
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.core import rpc
from dimos.core.module import Module, get_loop
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.skill import SkillConfig, SkillContainer  # type: ignore[attr-defined]
from dimos.protocol.skill.type import MsgType, Output, Reducer, Return, SkillMsg, Stream
from dimos.protocol.skill.utils import interpret_tool_call_args
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class SkillCoordinatorConfig:
    """Configuration for the SkillCoordinator module.

    The SkillCoordinator is the central orchestration layer between agents and skills,
    managing skill lifecycle, state tracking, and cross-event-loop message routing. This
    configuration class controls how skills communicate with the coordinator.
    """

    skill_transport: Annotated[
        type[SkillCommsSpec],
        Doc(
            """Communication transport implementation for skill messages between skills (thread pools)
            and coordinator. Must implement SkillCommsSpec (publish/subscribe semantics).
            Defaults to LCMSkillComms (LCM over "/skill" channel). Custom transports can
            implement the SkillCommsSpec interface."""
        ),
    ] = LCMSkillComms


class SkillStateEnum(Enum):
    """Lifecycle state of a skill invocation.

    State Transition Flow (unidirectional, message-driven):
        pending → running → (completed | error)

        - pending: Scheduled but not yet executing
        - running: Actively executing in thread pool
        - completed: Finished successfully
        - error: Terminated with an exception

    State transitions correspond directly to message types (MsgType.start, stream, ret, error).
    Terminal states (completed, error) trigger automatic cleanup when clear=True in
    generate_snapshot(), guaranteeing exactly one terminal state per invocation.
    """

    pending = 0
    running = 1
    completed = 2
    error = 3

    def colored_name(
        self,
    ) -> Annotated[Text, Doc("The state name as a rich Text object with color styling.")]:
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
    """Tracks execution state of a single skill invocation.

    Manages the skill lifecycle (pending → running → completed/error), accumulates stream
    messages via the configured reducer, and encodes state for agent consumption using the
    dual-protocol pattern.

    Dual-Protocol Pattern:
        - First agent_encode() call: Returns ToolMessage (LangChain protocol compatibility)
        - Subsequent calls: Return JSON status updates (skill name, call_id, state, data, duration)

    State Transitions (via handle_msg()):
        - MsgType.start: pending → running
        - MsgType.stream: maintains running, applies reducer
        - MsgType.ret: running → completed
        - MsgType.error: any state → error

    Notification Logic (handle_msg() returns True when):
        - Stream messages with Stream.call_agent
        - Return messages with Return.call_agent
        - Error messages (always)
        - Start messages never trigger notification

    Example:
        >>> from dimos.protocol.skill.type import Stream, Reducer, Output
        >>> config = SkillConfig(
        ...     name="navigate_to",
        ...     ret=Return.call_agent,
        ...     stream=Stream.none,
        ...     reducer=Reducer.all,
        ...     output=Output.standard,
        ...     schema={}
        ... )
        >>> skill_state = SkillState(call_id="abc123", name="navigate_to", skill_config=config)
        >>> start_msg = SkillMsg(call_id="abc123", skill_name="navigate_to", content={}, type=MsgType.start)
        >>> skill_state.handle_msg(start_msg)  # Transitions to running
        False
        >>> tool_msg = skill_state.agent_encode()  # First call returns ToolMessage
        >>> isinstance(tool_msg, ToolMessage)
        True
        >>> json_update = skill_state.agent_encode()  # Subsequent calls return JSON
        >>> import json
        >>> data = json.loads(json_update)
        >>> data['name']
        'navigate_to'
    """

    call_id: Annotated[
        str,
        Doc(
            "Unique identifier for this skill invocation, used for message routing and tool result correlation"
        ),
    ]
    name: Annotated[str, Doc("Name of the skill being executed")]
    state: Annotated[
        SkillStateEnum,
        Doc(
            "Current lifecycle state tracking execution progress (pending, running, completed, error)"
        ),
    ]
    skill_config: Annotated[
        SkillConfig,
        Doc(
            "Configuration controlling skill behavior including streaming mode, return mode, reducer function, and output format"
        ),
    ]

    msg_count: Annotated[
        int,
        Doc(
            "Total number of SkillMsg messages received from the skill execution, used for progress tracking"
        ),
    ] = 0
    sent_tool_msg: Annotated[
        bool,
        Doc(
            "Flag tracking whether the initial ToolMessage has been sent, ensures correct protocol adherence"
        ),
    ] = False

    start_msg: Annotated[
        SkillMsg[Literal[MsgType.start]] | None,
        Doc("The MsgType.start message marking execution begin, used for duration calculation"),
    ] = None
    end_msg: Annotated[
        SkillMsg[Literal[MsgType.ret]] | None,
        Doc("Terminal message (either ret or error), marks completion timestamp"),
    ] = None
    error_msg: Annotated[
        SkillMsg[Literal[MsgType.error]] | None,
        Doc(
            "MsgType.error message if skill terminated with exception, contains error details for agent"
        ),
    ] = None
    ret_msg: Annotated[
        SkillMsg[Literal[MsgType.ret]] | None,
        Doc("MsgType.ret message with final return value, only present for successful completion"),
    ] = None
    reduced_stream_msg: Annotated[
        list[SkillMsg[Literal[MsgType.reduced_stream]]] | None,
        Doc(
            "Accumulated stream messages after applying the configured reducer function, provides incremental progress for streaming skills"
        ),
    ] = None

    def __init__(
        self,
        call_id: Annotated[str, Doc("Unique identifier for this skill invocation")],
        name: Annotated[str, Doc("Name of the skill being executed")],
        skill_config: Annotated[
            SkillConfig | None,
            Doc(
                "Optional configuration controlling skill behavior. If None, defaults to no streaming, no return, all reducer, standard output"
            ),
        ] = None,
    ) -> None:
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

    def duration(
        self,
    ) -> Annotated[
        float,
        Doc(
            """Duration in seconds. Returns elapsed time if completed,
            time since start if running, or 0.0 if not started."""
        ),
    ]:
        """Calculate the duration of the skill run."""
        if self.start_msg and self.end_msg:
            return self.end_msg.ts - self.start_msg.ts
        elif self.start_msg:
            return time.time() - self.start_msg.ts
        else:
            return 0.0

    def content(
        self,
    ) -> Annotated[
        dict[str, Any] | str | int | float | None,
        Doc("""
            The content from the skill's execution state.

            Returns the reduced stream message content when running,
            the return message content (or reduced stream if streaming) when completed,
            or the error message content (with optional stream context) when errored.
            Returns None for pending state or when no content is available.
        """),
    ]:
        """Get the content from the current skill execution state."""
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
                return self.reduced_stream_msg.content + "\n" + self.error_msg.content  # type: ignore[attr-defined]
            else:
                return self.error_msg.content  # type: ignore[return-value]

        return None

    def agent_encode(
        self,
    ) -> Annotated[
        ToolMessage | str,
        Doc(
            """ToolMessage on first call, JSON string on subsequent calls.

            This dual-protocol pattern bridges LangChain's tool call requirements
            (one ToolMessage per tool_call_id) with the need for ongoing status updates
            from long-running skills.

            First call returns a ToolMessage that completes the tool invocation protocol
            and enters permanent conversation history. Subsequent calls return JSON-encoded
            state snapshots that get aggregated into an AIMessage providing situational
            awareness about active skills, without violating the one-ToolMessage constraint.
            """
        ),
    ]:
        """Encode skill state for agent consumption using dual-protocol pattern."""
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
    def handle_msg(
        self,
        msg: Annotated[SkillMsg, Doc("The skill message to process")],  # type: ignore[type-arg]
    ) -> Annotated[
        bool,
        Doc(
            """Whether the coordinator should notify the agent about this message.
            True for errors (always), stream messages with Stream.call_agent,
            and return messages with Return.call_agent. False otherwise."""
        ),
    ]:
        """Process an incoming skill message and update internal state.

        Updates the skill's execution state based on the message type. For stream
        messages, applies the configured reducer to accumulate outputs. The return
        value determines whether the coordinator should schedule an agent call to
        process this message.

        Notification logic:
        - Start messages: Never notify (skill is initializing)
        - Stream messages: Notify only if configured with Stream.call_agent
        - Return messages: Notify only if configured with Return.call_agent
        - Error messages: Always notify (errors require agent attention)
        """
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
    """Dictionary mapping call_id to SkillState with Rich-formatted table display.

    Provides table() and __str__() methods for debugging and monitoring skill execution
    in SkillCoordinator.
    Table columns: Call ID, Skill, State (colored), Duration, Messages.
    """

    def table(self) -> Annotated[Table, Doc("Rich Table with formatted skill state columns")]:
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
    """Central orchestration layer between agents and skills.

    Manages skill lifecycle, state tracking, and message routing across event loops,
    decoupling agents (asyncio) from skills (thread pools) using lazy event creation
    and thread-safe cross-loop notification.

    Container Types:
        - Static: Fixed skills cached at registration for O(1) lookup
        - Dynamic: Runtime-generated skills queried on-demand for context-dependent generation

    Cross-Event-Loop Synchronization:
        - asyncio.Event created lazily in agent's loop on first wait_for_updates()
        - call_soon_threadsafe bridges transport loop and agent loop
        - Message-driven state tracking via SkillState objects

    Examples:
        Basic coordinator setup and skill invocation:

        >>> from dimos.core.module import Module
        >>> from dimos.protocol.skill.skill import skill
        >>>
        >>> # Note that you'll need to do a bit more for the skill to be available to llm agents -- see the tutorial.
        >>> class NavigationModule(Module):
        ...     @skill()
        ...     def navigate_to(self, location: str) -> str:
        ...         return f"Navigating to {location}"
        >>>
        >>> # Set up coordinator
        >>> coordinator = SkillCoordinator()
        >>> coordinator.register_skills(NavigationModule())
        >>> coordinator.start()
        >>> coordinator.call_skill(call_id="123", skill_name="navigate_to", args={"args": ["kitchen"]})
        >>>
        >>> # Verify skill state was created
        >>> snapshot = coordinator.generate_snapshot(clear=False)
        >>> "123" in snapshot
        True
        >>> coordinator.stop()

        Agent integration with update loop (async):

        >>> import asyncio
        >>> # (In actual async context)
        >>> # await coordinator.wait_for_updates(timeout=1.0)
        >>> # snapshot = coordinator.generate_snapshot(clear=True)
        >>> # for call_id, state in snapshot.items():
        >>> #     message = state.agent_encode()  # First: ToolMessage, then: JSON

    Notes:
        - Not thread-safe for _skill_state (single coordinator loop assumed)
        - generate_snapshot(clear=True) provides atomic read-and-clear, removing terminal states
        - Completed/errored skills removed after snapshot(clear=True)
        - Message flow pattern: Skills publish messages in a fixed sequence:
            1. One `start` message when execution begins
            2. Zero or more `stream` messages during execution (for incremental progress)
            3. Exactly one terminal message: either `ret` (success) or `error` (failure)
    """

    default_config = SkillCoordinatorConfig  # type: ignore[assignment]
    empty: bool = True

    _static_containers: Annotated[
        list[SkillContainer],
        Doc(
            "Containers with fixed skills known at registration time. Skills are cached immediately for performance."
        ),
    ]
    _dynamic_containers: Annotated[
        list[SkillContainer],
        Doc(
            "Containers whose skills depend on runtime context. Queried on each skills() call; not cached."
        ),
    ]
    _skill_state: Annotated[
        SkillStateDict,
        Doc(
            "Maps call_id to SkillState objects tracking each skill invocation. Key is call_id (unique per invocation), not skill_name (reusable)."
        ),
    ]
    _skills: Annotated[
        dict[str, SkillConfig], Doc("Cached static skills for O(1) lookup performance.")
    ]
    _updates_available: Annotated[
        asyncio.Event | None,
        Doc(
            "Event signaling skill updates ready for agent processing. Created lazily in agent's event loop on first wait_for_updates() call."
        ),
    ]
    _loop: Annotated[
        asyncio.AbstractEventLoop | None, Doc("Coordinator's own event loop for message handling.")
    ]
    _loop_thread: threading.Thread | None
    _agent_loop: Annotated[
        asyncio.AbstractEventLoop | None,
        Doc("Agent's event loop, captured when updates_available event is created."),
    ]

    def __init__(self) -> None:
        # TODO: Why isn't this super().__init__() ?
        SkillContainer.__init__(self)
        self._loop, self._loop_thread = get_loop()
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
        self,
        call_id: Annotated[
            str | Literal[False],
            Doc("""Unique identifier for this skill invocation. If False, a
            timestamp-based ID will be auto-generated. This ID is used to
            track skill execution state and correlate responses."""),
        ],
        skill_name: Annotated[
            str,
            Doc("""Name of the skill to invoke, as registered in the
            coordinator's skill registry."""),
        ],
        args: Annotated[
            dict[str, Any],
            Doc("""Dictionary containing skill invocation arguments. Expected to
            contain an "args" key with either a list of positional arguments
            or a dict of keyword arguments. Will be interpreted by
            `interpret_tool_call_args` to extract positional and keyword args."""),
        ],
    ) -> None:
        """Execute a skill invocation requested by an agent.

        Creates a SkillState to track execution and delegates to the skill's call method.
        Auto-generates call_id from timestamp if not provided. Logs error and returns
        early if skill not found (e.g., expired dynamic skill).
        """
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
    def handle_message(
        self,
        msg: Annotated[
            SkillMsg,  # type: ignore[type-arg]
            Doc(
                """The incoming skill message containing status updates, output, or errors.
                Must contain a valid call_id and skill_name."""
            ),
        ],
    ) -> None:
        """Process incoming skill messages and notify the agent when needed.

        Routes messages to the appropriate SkillState. If notification is required
        (based on skill config), sets the agent's updates_available event using
        call_soon_threadsafe for cross-loop communication.

        Handles orphan messages (no SkillState) by lazy initialization with warning.
        Post-shutdown messages are silently dropped.
        """
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

    async def wait_for_updates(
        self,
        timeout: Annotated[float | None, Doc("Optional timeout in seconds")] = None,
    ) -> Annotated[bool, Doc("True if updates are available, False on timeout")]:
        """Wait for skill updates to become available.

        This method should be called by the agent when it's ready to receive updates.
        It will block until updates are available or timeout is reached.
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

    def generate_snapshot(
        self,
        clear: Annotated[
            bool,
            Doc(
                """Whether to perform cleanup after snapshot generation. If True,
                removes completed/errored skills from tracking, resets stream accumulators
                for running skills, and clears the updates_available event. If False,
                returns a simple copy without side effects."""
            ),
        ] = True,
    ) -> Annotated[
        SkillStateDict,
        Doc(
            """Dictionary mapping call_id to SkillState objects. Each SkillState contains
            the skill's execution state, accumulated outputs, and error information.
            The returned dict is a copy independent of internal state."""
        ),
    ]:
        """Generate an atomic snapshot of skill states with optional cleanup.

        Returns a point-in-time copy of all tracked skill invocations. When clear=True,
        performs atomic read-and-clear: removes terminal states (completed/error), resets
        stream accumulators for running skills, and clears the updates_available event.
        """
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
    def register_skills(
        self,
        container: Annotated[
            SkillContainer,
            Doc(
                """The skill container to register. Must implement the SkillContainer
                protocol with a dynamic_skills() method and a skills() method that returns
                a mapping of skill names to SkillConfig objects."""
            ),
        ],
    ) -> None:
        """Register a skill container with the coordinator, making its skills available to agents.

        Static containers (dynamic_skills() == False): Skills cached immediately for O(1) lookup.
        Dynamic containers (dynamic_skills() == True): Skills queried at runtime for context-dependent generation.

        Skill resolution order: cached static skills first, then dynamic container query.
        """
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
