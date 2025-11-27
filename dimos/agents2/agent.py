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

"""LLM-based agent orchestration bridging reasoning with robot skill execution.

This module implements DimOS's neurosymbolic agent architecture: LLM-based agents
that invoke robot skills through a structured tool-calling protocol.

Core Classes
------------
Agent
    Base agent class requiring explicit loop control via `query()` or `agent_loop()`.
    Integrates with `SkillCoordinator` to execute long-running skills asynchronously.

LlmAgent
    Agent variant that auto-starts its processing loop on `start()`. Useful for
    blueprint composition with `autoconnect()` and `ModuleCoordinator`.

Exports
-------
The module's `__all__` includes:

- `Agent`: For explicit loop control
- `llm_agent`: Blueprint factory (`LlmAgent.blueprint`) for composition
- `deploy`: Convenience helper for standalone agent deployment

Internal utilities (not exported):

- `SkillStateSummary`: TypedDict for skill state snapshots in LLM messages
- `snapshot_to_messages`: Transform skill state to LangChain message protocol

Architecture
------------
Agents coordinate with a `SkillCoordinator` to discover skills, bind them as LLM
tools, and execute them asynchronously with streaming updates.
The event-driven loop alternates between LLM invocations and skill execution,
with state changes triggering agent calls.

Testing
-------
For deterministic testing without LLM API calls, use `MockModel` from
`dimos.agents2.testing` to inject predetermined responses:

>>> from dimos.agents2.testing import MockModel
>>> from langchain_core.messages import AIMessage
>>> mock = MockModel(responses=[AIMessage(content="Test response")])
>>> agent = Agent(system_prompt="Test", model_instance=mock)

See also
--------
dimos.agents2.spec : AgentSpec base class defining the agent interface
dimos.protocol.skill.coordinator : SkillCoordinator for managing skill lifecycle
dimos.agents2.cli.human : HumanInput module for CLI-based agent interaction
dimos.agents2.testing : Testing utilities including MockModel
dimos.utils.cli.agentspy : CLI tool for real-time monitoring of agent messages
"""

import asyncio
import datetime
import json
from operator import itemgetter
import os
from typing import Annotated, Any, TypedDict
import uuid

from annotated_doc import Doc
from langchain.chat_models import init_chat_model
from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    SystemMessage,
    ToolCall,
    ToolMessage,
)
from langchain_huggingface import ChatHuggingFace, HuggingFacePipeline

from dimos.agents2.ollama_agent import ensure_ollama_model
from dimos.agents2.spec import AgentSpec, Model, Provider
from dimos.agents2.system_prompt import get_system_prompt
from dimos.core import DimosCluster, rpc
from dimos.protocol.skill.coordinator import (
    SkillCoordinator,
    SkillState,
    SkillStateDict,
)
from dimos.protocol.skill.skill import SkillContainer
from dimos.protocol.skill.type import Output
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


SYSTEM_MSG_APPEND = "\nYour message history will always be appended with a System Overview message that provides situational awareness."


def toolmsg_from_state(state: SkillState) -> ToolMessage:
    if state.skill_config.output != Output.standard:
        content = "output attached in separate messages"
    else:
        content = state.content()  # type: ignore[assignment]

    return ToolMessage(
        # if agent call has been triggered by another skill,
        # and this specific skill didn't finish yet but we need a tool call response
        # we return a message explaining that execution is still ongoing
        content=content
        or "Running, you will be called with an update, no need for subsequent tool calls",
        name=state.name,
        tool_call_id=state.call_id,
    )


class SkillStateSummary(TypedDict):
    """Lightweight snapshot of skill execution state for LLM situational awareness.

    JSON-serializable representation of SkillState included in state overview messages
    sent to LLM agents. Informs agents about skills running but not yet acknowledged
    via ToolMessage, enabling tracking of ongoing operations.

    Typically created internally by the agent system. Users rarely construct these directly.

    Examples:
        Creating a summary dictionary directly:

        >>> summary: SkillStateSummary = {
        ...     "name": "navigate_to",
        ...     "call_id": "abc-123",
        ...     "state": "running",
        ...     "data": "Moving to target location"
        ... }
        >>> print(summary["name"])
        navigate_to
        >>> print(summary["state"])
        running

        Multiple summaries in a state overview message:

        >>> import json
        >>> from langchain_core.messages import AIMessage
        >>>
        >>> summaries: list[SkillStateSummary] = [
        ...     {"name": "scan_room", "call_id": "uuid1", "state": "running", "data": "Scanning..."},
        ...     {"name": "navigate_to", "call_id": "uuid2", "state": "completed", "data": "Arrived"}
        ... ]
        >>> overview = "\\n".join(json.dumps(s) for s in summaries)
        >>> msg = AIMessage(content=f"State Overview:\\n{overview}")
    """

    name: Annotated[
        str, Doc("The skill's registered identifier (e.g., 'navigate_to', 'scan_room').")
    ]
    call_id: Annotated[str, Doc("Unique identifier string for this specific skill invocation.")]
    state: Annotated[str, Doc("Execution state: 'pending', 'running', 'completed', or 'error'.")]
    data: Annotated[
        Any,
        Doc(
            """Skill output content or placeholder message. For standard output modes,
            contains result of SkillState.content(). For Output.image, contains
            literal string 'data will be in a separate message'."""
        ),
    ]


def summary_from_state(state: SkillState, special_data: bool = False) -> SkillStateSummary:
    content = state.content()
    if isinstance(content, dict):
        content = json.dumps(content)

    if not isinstance(content, str):
        content = str(content)

    return {
        "name": state.name,
        "call_id": state.call_id,
        "state": state.state.name,
        "data": state.content() if not special_data else "data will be in a separate message",
    }


def _custom_json_serializers(obj):  # type: ignore[no-untyped-def]
    if isinstance(obj, datetime.date | datetime.datetime):
        return obj.isoformat()
    raise TypeError(f"Type {type(obj)} not serializable")


def snapshot_to_messages(
    state: Annotated[
        SkillStateDict,
        Doc(
            """Snapshot from SkillCoordinator.generate_snapshot() mapping call_id to
            SkillState objects with execution state, outputs, and configuration."""
        ),
    ],
    tool_calls: Annotated[
        list[ToolCall],
        Doc(
            """Tool calls from the previous agent message, used to match skills requiring
            ToolMessage responses per LangChain's protocol."""
        ),
    ],
) -> Annotated[
    dict,
    Doc(
        """Dictionary with three keys mapping to message lists:
        'tool_msgs' (list[ToolMessage]): Tool responses for skills matching tool_calls;
        'history_msgs' (list[HumanMessage]): Persistent messages from Output.human skills;
        'state_msgs' (list[AIMessage | HumanMessage]): Transient state awareness messages."""
    ),
]:
    """Transform skill execution snapshot into LangChain messages for agent loop.

    Internal function called by Agent.agent_loop() at two points during execution.
    Implements a three-tier message routing protocol separating tool responses
    (satisfying LangChain's tool calling protocol) from state awareness messages
    (tracking long-running skills) and persistent history (human input, critical events).

    Notes:
        This is an internal transformation layer. Users should not call this directly.

        Skills are processed sorted by duration (shortest first). Routing rules by output type:

        - Output.standard: Tool response if matching call_id, else state overview
        - Output.human: Always routes to history_msgs, bypassing tool protocol
        - Output.image: Tool response with placeholder plus separate HumanMessage

        tool_msgs and history_msgs persist in conversation; state_msgs are transient
        and replaced on each update cycle.

    See also:
        Agent.agent_loop: Primary caller during state change detection and message generation.
        toolmsg_from_state: Helper creating tool response messages.
        summary_from_state: Helper creating state overview summaries.
    """
    # builds a set of tool call ids from a previous agent request
    tool_call_ids = set(
        map(itemgetter("id"), tool_calls),
    )

    # build a tool msg responses
    tool_msgs: list[ToolMessage] = []

    # build a general skill state overview (for longer running skills)
    state_overview: list[dict[str, SkillStateSummary]] = []

    # for special skills that want to return a separate message
    # (images for example, requires to be a HumanMessage)
    special_msgs: list[HumanMessage] = []

    # for special skills that want to return a separate message that should
    # stay in history, like actual human messages, critical events
    history_msgs: list[HumanMessage] = []

    # Initialize state_msg
    state_msg = None

    for skill_state in sorted(
        state.values(),
        key=lambda skill_state: skill_state.duration(),
    ):
        if skill_state.call_id in tool_call_ids:
            tool_msgs.append(toolmsg_from_state(skill_state))

        if skill_state.skill_config.output == Output.human:
            content = skill_state.content()
            if not content:
                continue
            history_msgs.append(HumanMessage(content=content))  # type: ignore[arg-type]
            continue

        special_data = skill_state.skill_config.output == Output.image
        if special_data:
            content = skill_state.content()
            if not content:
                continue
            special_msgs.append(HumanMessage(content=content))  # type: ignore[arg-type]

        if skill_state.call_id in tool_call_ids:
            continue

        state_overview.append(summary_from_state(skill_state, special_data))  # type: ignore[arg-type]

    if state_overview:
        state_overview_str = "\n".join(
            json.dumps(s, default=_custom_json_serializers) for s in state_overview
        )
        state_msg = AIMessage("State Overview:\n" + state_overview_str)

    return {  # type: ignore[return-value]
        "tool_msgs": tool_msgs,
        "history_msgs": history_msgs,
        "state_msgs": ([state_msg] if state_msg else []) + special_msgs,
    }


class Agent(AgentSpec):
    """Neurosymbolic orchestrator bridging LLM reasoning with robot skill execution.

    Implements an event-driven agent loop that alternates between LLM invocations
    and skill execution. Maintains conversation history, coordinates skill lifecycle
    through a `SkillCoordinator`, and transforms skill state updates into LangChain
    messages for continued reasoning.

    Lifecycle: INITIALIZED → STARTED (after `start()`) → RUNNING (during `agent_loop()`) → back to STARTED (loop completes) → STOPPED (after `stop()`).

    Agent vs. LlmAgent:
        Use `Agent` when you need explicit control over when the processing loop starts
        (typically via `query()` calls). Use `LlmAgent` when you want the agent to
        auto-start its loop on `start()`, which is essential for blueprint composition
        with `autoconnect()` and `ModuleCoordinator`.

    Attributes:
        coordinator (SkillCoordinator):
            Manages skill registration, execution, and state tracking.
        system_message (SystemMessage):
            Initial system prompt appended with state overview notice.
        state_messages (list[AIMessage | HumanMessage]):
            Transient messages for current skill state; replaced each update cycle.

    Notes:
        The agent loop terminates when `coordinator.has_active_skills()` returns False.
        Skills with `Return.none`, `Return.passive`, `Stream.none`, or `Stream.passive`
        don't prevent termination.

        For testing, use `MockModel` from `dimos.agents2.testing` to inject
        deterministic responses without requiring real LLM API calls.

    See also:
        LlmAgent: Auto-starts loop on `start()` for blueprint composition.
        AgentSpec: Base class defining agent interface.
        SkillCoordinator: Skill lifecycle manager.
        query: Synchronous blocking interface for agent queries.
        agent_loop: Core async processing loop.

    Examples:
        >>> from dimos.agents2.agent import Agent
        >>> from dimos.agents2.testing import MockModel
        >>> from langchain_core.messages import AIMessage
        >>> mock = MockModel(responses=[AIMessage(content="The answer is 42")])
        >>> agent = Agent(system_prompt="You are a helpful assistant.", model_instance=mock)
        >>> agent.start()
        >>> result = agent.query("What is the meaning of life?")
        >>> result
        'The answer is 42'
        >>> agent.stop()
    """

    system_message: SystemMessage
    state_messages: list[AIMessage | HumanMessage]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        *args,
        **kwargs,
    ) -> None:
        AgentSpec.__init__(self, *args, **kwargs)

        self.state_messages = []
        self.coordinator = SkillCoordinator()
        self._history = []  # type: ignore[var-annotated]
        self._agent_id = str(uuid.uuid4())
        self._agent_stopped = False

        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                self.system_message = SystemMessage(self.config.system_prompt + SYSTEM_MSG_APPEND)
            else:
                self.config.system_prompt.content += SYSTEM_MSG_APPEND  # type: ignore[operator]
                self.system_message = self.config.system_prompt
        else:
            self.system_message = SystemMessage(get_system_prompt() + SYSTEM_MSG_APPEND)

        self.publish(self.system_message)

        # Use provided model instance if available, otherwise initialize from config
        if self.config.model_instance:
            self._llm = self.config.model_instance
        else:
            # For Ollama provider, ensure the model is available before initializing
            if self.config.provider.value.lower() == "ollama":
                ensure_ollama_model(self.config.model)

            # For HuggingFace, we need to create a pipeline and wrap it in ChatHuggingFace
            if self.config.provider.value.lower() == "huggingface":
                llm = HuggingFacePipeline.from_model_id(
                    model_id=self.config.model,
                    task="text-generation",
                    pipeline_kwargs={
                        "max_new_tokens": 512,
                        "temperature": 0.7,
                    },
                )
                self._llm = ChatHuggingFace(llm=llm, model_id=self.config.model)
            else:
                self._llm = init_chat_model(  # type: ignore[call-overload]
                    model_provider=self.config.provider, model=self.config.model
                )

    @rpc
    def get_agent_id(self) -> str:
        return self._agent_id

    @rpc
    def start(self) -> None:
        super().start()
        self.coordinator.start()

    @rpc
    def stop(self) -> None:
        self.coordinator.stop()
        self._agent_stopped = True
        super().stop()

    def clear_history(self) -> None:
        self._history.clear()

    def append_history(self, *msgs: list[AIMessage | HumanMessage]) -> None:
        for msg in msgs:
            self.publish(msg)  # type: ignore[arg-type]

        self._history.extend(msgs)

    def history(self):  # type: ignore[no-untyped-def]
        return [self.system_message, *self._history, *self.state_messages]

    # Used by agent to execute tool calls
    def execute_tool_calls(self, tool_calls: list[ToolCall]) -> None:
        """Execute a list of tool calls from the agent."""
        if self._agent_stopped:
            logger.warning("Agent is stopped, cannot execute tool calls.")
            return
        for tool_call in tool_calls:
            logger.info(f"executing skill call {tool_call}")
            self.coordinator.call_skill(
                tool_call.get("id"),  # type: ignore[arg-type]
                tool_call.get("name"),  # type: ignore[arg-type]
                tool_call.get("args"),  # type: ignore[arg-type]
            )

    # used to inject skill calls into the agent loop without agent asking for it
    def run_implicit_skill(
        self,
        skill_name: Annotated[
            str,
            Doc(
                """Name of the registered skill to invoke. Must match a skill in the
                coordinator's registry."""
            ),
        ],
        **kwargs,
    ) -> None:
        """Inject skill invocation without agent awareness or decision-making.

        Programmatic skill execution that bypasses normal agent reasoning. Primary use
        is bootstrapping agent sessions with initial skills like HumanInput, which
        must run before the agent can begin processing queries.

        Differences from execute_tool_calls():
            - **Trigger source**: Programmatic/external vs. agent-initiated via LLM
            - **Call ID**: Always uses `False` (auto-generated) vs. LLM-provided ID
            - **Visibility**: Implicit to agent vs. tracked in conversation history
            - **Use cases**: Bootstrap/events/background vs. deliberate agent tool use

        Examples:
            >>> from dimos.agents2.agent import Agent
            >>> from dimos.agents2.testing import MockModel
            >>> from langchain_core.messages import AIMessage
            >>> mock = MockModel(responses=[AIMessage(content="Ready")])
            >>> agent = Agent(system_prompt="Test assistant", model_instance=mock)
            >>> agent.start()
            >>> # In practice, register a SkillContainer first, then run its skill:
            >>> # agent.register_skills(my_skill_container)
            >>> # agent.run_implicit_skill("my_skill", param="value")
            >>> agent.stop()

        Notes:
            - Uses `call_id=False` to trigger auto-generation by the coordinator
            - Skills execute asynchronously through the coordinator
            - Silently returns with warning if agent is stopped

        See also:
            execute_tool_calls: Handle agent-initiated tool calls with LLM-provided IDs.
            Agent.agent_loop: Main processing loop that handles skill responses.
        """
        if self._agent_stopped:
            logger.warning("Agent is stopped, cannot execute implicit skill calls.")
            return
        self.coordinator.call_skill(False, skill_name, {"args": kwargs})

    async def agent_loop(
        self,
        first_query: Annotated[
            str,
            Doc(
                """Initial human query to process. If provided, appended to conversation history
                as a HumanMessage before the loop begins. Defaults to empty string, enabling
                event-driven mode where the agent waits for skills or external events."""
            ),
        ] = "",
    ) -> Annotated[
        str | None,
        Doc(
            """The content of the final AIMessage from the LLM on normal termination.
            Returns literal string 'Agent is stopped.' if called when agent is stopped.
            Returns None if an exception occurs during execution."""
        ),
    ]:
        """Run the agent's core processing loop until all skills complete.

        Implements an event-driven execution cycle that alternates between LLM reasoning
        and skill execution. Each iteration: (1) binds current tools, (2) invokes the LLM
        with conversation history plus state overview, (3) executes any tool calls,
        (4) waits for skill updates via async event notification, and (5) transforms
        skill results into structured messages for the next iteration.

        Examples:
            Most users should use `query()` instead, which has an executable doctest.
            (xdoctest 1.3.0 doesn't detect doctests in ``async def`` methods.)

            >>> response = await agent.agent_loop("What is 2+2?")  # doctest: +SKIP

        Notes:
            **When to use vs. alternatives**:
              - Use `agent_loop()` directly: In async contexts where you need explicit control
              - Use `query()`: For synchronous/blocking calls from non-async code
              - Use `loop_thread()`: For fire-and-forget background processing

            **Termination conditions**:
              The loop terminates when `coordinator.has_active_skills()` returns False.
              Active skills are those configured with `Return.call_agent` or `Stream.call_agent`.
              Skills with `Return.none` or `Return.passive` don't prevent termination.

        See also:
            query: Synchronous wrapper for agent_loop, blocks until completion.
            loop_thread: Fire-and-forget variant that schedules agent_loop in background.
            coordinator.has_active_skills: Method determining loop termination.
        """
        # TODO: Should I add a lock here to prevent concurrent calls to agent_loop?

        if self._agent_stopped:
            logger.warning("Agent is stopped, cannot run agent loop.")
            # return "Agent is stopped."
            import traceback

            traceback.print_stack()
            return "Agent is stopped."

        self.state_messages = []
        if first_query:
            self.append_history(HumanMessage(first_query))  # type: ignore[arg-type]

        def _get_state() -> str:
            # TODO: FIX THIS EXTREME HACK
            update = self.coordinator.generate_snapshot(clear=False)
            snapshot_msgs = snapshot_to_messages(update, msg.tool_calls)  # type: ignore[attr-defined]
            return json.dumps(snapshot_msgs, sort_keys=True, default=lambda o: repr(o))

        try:
            while True:
                # we are getting tools from the coordinator on each turn
                # since this allows for skillcontainers to dynamically provide new skills
                tools = self.get_tools()  # type: ignore[no-untyped-call]
                self._llm = self._llm.bind_tools(tools)  # type: ignore[assignment]

                # publish to /agent topic for observability
                for state_msg in self.state_messages:
                    self.publish(state_msg)

                # history() builds our message history dynamically
                # ensures we include latest system state, but not old ones.
                messages = self.history()  # type: ignore[no-untyped-call]

                # Some LLMs don't work without any human messages. Add an initial one.
                if len(messages) == 1 and isinstance(messages[0], SystemMessage):
                    messages.append(
                        HumanMessage(
                            "Everything is initialized. I'll let you know when you should act."
                        )
                    )
                    self.append_history(messages[-1])

                msg = self._llm.invoke(messages)

                self.append_history(msg)  # type: ignore[arg-type]

                logger.info(f"Agent response: {msg.content}")

                state = _get_state()

                if msg.tool_calls:  # type: ignore[attr-defined]
                    self.execute_tool_calls(msg.tool_calls)  # type: ignore[attr-defined]

                # print(self)
                # print(self.coordinator)

                self._write_debug_history_file()

                if not self.coordinator.has_active_skills():
                    logger.info("No active tasks, exiting agent loop.")
                    return msg.content

                # coordinator will continue once a skill state has changed in
                # such a way that agent call needs to be executed

                if state == _get_state():
                    await self.coordinator.wait_for_updates()

                # we request a full snapshot of currently running, finished or errored out skills
                # we ask for removal of finished skills from subsequent snapshots (clear=True)
                update = self.coordinator.generate_snapshot(clear=True)

                # generate tool_msgs and general state update message,
                # depending on a skill having associated tool call from previous interaction
                # we will return a tool message, and not a general state message
                snapshot_msgs = snapshot_to_messages(update, msg.tool_calls)  # type: ignore[attr-defined]

                self.state_messages = snapshot_msgs.get("state_msgs", [])  # type: ignore[attr-defined]
                self.append_history(
                    *snapshot_msgs.get("tool_msgs", []),  # type: ignore[attr-defined]
                    *snapshot_msgs.get("history_msgs", []),  # type: ignore[attr-defined]
                )

        except Exception as e:
            logger.error(f"Error in agent loop: {e}")
            import traceback

            traceback.print_exc()

    @rpc
    def loop_thread(
        self,
    ) -> Annotated[
        bool,
        Doc(
            """Always returns True, indicating the loop was scheduled (not that it started
            executing or will complete successfully)."""
        ),
    ]:
        """Start the agent's autonomous execution loop in the background.

        Fire-and-forget method that schedules the agent loop without blocking.
        Returns immediately while the agent continues processing asynchronously
        in its event loop thread. Unlike query(), this never waits for completion.

        Examples:
            Basic fire-and-forget usage:

            >>> from dimos.agents2 import Agent
            >>> agent = Agent(system_prompt="You are a helpful assistant.")
            >>> agent.start()
            >>> agent.loop_thread()  # Returns immediately
            True
            >>> agent.stop()
            >>> # Agent was processing autonomously in background until stopped

        Notes:
            - Agent loop executes with empty initial query (`agent_loop("")`)
            - Multiple calls create concurrent loops (may cause race conditions)
            - Called automatically by `LlmAgent.start()` for auto-start behavior

            **When to use vs. alternatives**:
              - Use `loop_thread()`: Fire-and-forget background processing
              - Use `query()`: Blocking call that waits for agent response
              - Use `query_async()`: Async contexts requiring await

        See also:
            query: Blocking method that waits for agent response.
            query_async: Async version for use in async contexts.
            LlmAgent.start: Automatically calls loop_thread() on startup.
        """
        asyncio.run_coroutine_threadsafe(self.agent_loop(), self._loop)  # type: ignore[arg-type]
        return True

    @rpc
    def query(
        self,
        query: Annotated[str, Doc("The user query to process.")],
    ) -> Annotated[
        str | None,
        Doc(
            """The agent's response (final AIMessage content).
            Returns 'Agent is stopped.' if agent was stopped.
            Returns None on error (exception logged, not propagated)."""
        ),
    ]:
        """Send a query to the agent and block until response.

        Synchronous wrapper around `agent_loop()` for use in non-async code.
        Blocks the calling thread until completion. Can be called from any
        thread, including RPC handlers.

        Notes:
            Uses `asyncio.run_coroutine_threadsafe()` to safely schedule execution
            on the agent's event loop from any calling thread.

        Examples:
            >>> from dimos.agents2.agent import Agent
            >>> from dimos.agents2.testing import MockModel
            >>> from langchain_core.messages import AIMessage
            >>> mock = MockModel(responses=[AIMessage(content="The answer is 4")])
            >>> agent = Agent(system_prompt="Math assistant", model_instance=mock)
            >>> agent.start()
            >>> result = agent.query("What is 2+2?")
            >>> result
            'The answer is 4'
            >>> agent.stop()

        See also:
            query_async: Async version for use in async contexts.
            agent_loop: The underlying async processing loop.
            loop_thread: Fire-and-forget variant for background processing.
        """
        # TODO: could this be
        # from distributed.utils import sync
        # return sync(self._loop, self.agent_loop, query)
        return asyncio.run_coroutine_threadsafe(self.agent_loop(query), self._loop).result()  # type: ignore[arg-type]

    async def query_async(
        self,
        query: Annotated[str, Doc("The user query to process.")],
    ) -> Annotated[
        str | None,
        Doc(
            """The agent's response (final AIMessage content).
            Returns 'Agent is stopped.' if agent was stopped.
            Returns None on error (exception logged, not propagated)."""
        ),
    ]:
        """Send a query to the agent and await the response.

        Async wrapper around `agent_loop()` for use in async contexts.
        Directly awaits the agent loop in the **caller's** event loop.

        Notes:
            The caller and agent should typically share the same loop; cross-loop awaiting
            can cause issues with skill coordination.
            Not RPC-decorated (use `query()` for RPC).

        Examples:
            >>> import asyncio
            >>> from dimos.agents2.agent import Agent
            >>> from dimos.agents2.testing import MockModel
            >>> from langchain_core.messages import AIMessage
            >>> async def test_query_async():
            ...     mock = MockModel(responses=[AIMessage(content="The answer is 4")])
            ...     agent = Agent(system_prompt="Math assistant", model_instance=mock)
            ...     agent.start()
            ...     result = await agent.query_async("What is 2+2?")
            ...     agent.stop()
            ...     return result
            >>> asyncio.run(test_query_async())
            'The answer is 4'

        See also:
            query: Synchronous/blocking version for non-async contexts.
            agent_loop: The underlying async processing loop.
            loop_thread: Fire-and-forget variant for background processing.
        """
        return await self.agent_loop(query)

    @rpc
    def register_skills(
        self,
        container: Annotated[
            SkillContainer, Doc("Skill container instance to register with the agent.")
        ],
        run_implicit_name: Annotated[
            str | None,
            Doc(
                """Optional skill name to run implicitly after registration.
                Commonly used to auto-start streaming skills like HumanInput."""
            ),
        ] = None,
    ):
        """Register a skill container with the agent's coordinator.

        Makes all @skill decorated methods from the container available to the agent
        for LLM tool calling. Optionally runs a specified skill implicitly after registration.

        Examples:
            Basic registration:

            >>> from dimos.agents2.agent import Agent
            >>> from dimos.agents2.testing import MockModel
            >>> from langchain_core.messages import AIMessage
            >>> mock = MockModel(responses=[AIMessage(content="Ready")])
            >>> agent = Agent(system_prompt="Test assistant", model_instance=mock)
            >>> agent.start()
            >>> # In practice, pass actual SkillContainer instances:
            >>> # agent.register_skills(skill_container)
            >>> agent.stop()

        See also:
            run_implicit_skill: Invoke skills without agent awareness.
            SkillCoordinator.register_skills: Underlying registration mechanism.
        """
        ret = self.coordinator.register_skills(container)  # type: ignore[func-returns-value]

        if run_implicit_name:
            self.run_implicit_skill(run_implicit_name)

        return ret

    def get_tools(self):  # type: ignore[no-untyped-def]
        return self.coordinator.get_tools()

    def _write_debug_history_file(self) -> None:
        file_path = os.getenv("DEBUG_AGENT_HISTORY_FILE")
        if not file_path:
            return

        history = [x.__dict__ for x in self.history()]  # type: ignore[no-untyped-call]

        with open(file_path, "w") as f:
            json.dump(history, f, default=lambda x: repr(x), indent=2)


class LlmAgent(Agent):
    """Agent that automatically starts its processing loop on startup.

    LlmAgent is especially useful when combining with other modules in a blueprint.
    When `start()` is called, it automatically invokes `loop_thread()`, eliminating
    manual loop initiation and making agents composable as standard modules.

    When to use each:
        Use LlmAgent when:
            - Using blueprint pattern with autoconnect()
            - Agent should start processing immediately on system startup
            - Building autonomous systems with ModuleCoordinator

        Use Agent when:
            - Using the deploy() helper for standalone agents
            - Need explicit control over when processing begins
            - Building query-driven systems with explicit query() calls

    Examples:
        Direct instantiation showing auto-start behavior:

        >>> from dimos.agents2.agent import Agent, LlmAgent
        >>> from dimos.agents2.testing import MockModel
        >>> from langchain_core.messages import AIMessage
        >>> mock1 = MockModel(responses=[AIMessage(content="Ready")])
        >>> agent = Agent(system_prompt="Test", model_instance=mock1)
        >>> agent.start()
        >>> agent.loop_thread()  # Manual loop start required
        True
        >>> agent.stop()
        >>>
        >>> mock2 = MockModel(responses=[AIMessage(content="Ready")])
        >>> agent2 = LlmAgent(system_prompt="Test", model_instance=mock2)
        >>> agent2.start()  # Automatically calls loop_thread()
        >>> agent2.stop()

        Blueprint pattern (typical production usage):

        >>> from dimos.core.blueprints import autoconnect  # doctest: +SKIP
        >>> from dimos.agents2.agent import llm_agent  # doctest: +SKIP
        >>> from dimos.agents2.cli.human import human_input  # doctest: +SKIP
        >>> from dimos.agents2.skills.demo_calculator_skill import demo_calculator_skill  # doctest: +SKIP
        >>> blueprint = autoconnect(  # doctest: +SKIP
        ...     demo_calculator_skill(),
        ...     llm_agent(system_prompt="You are a helpful assistant."),
        ...     human_input(),
        ... )
        >>> coordinator = blueprint.build()  # doctest: +SKIP
        >>> coordinator.start_all_modules()  # doctest: +SKIP
        >>> coordinator.loop()  # doctest: +SKIP

    Notes:
        The implementation overrides only start() to add self.loop_thread() after
        super().start(). All other methods are inherited from Agent unchanged.

        This makes LlmAgent compatible with ModuleCoordinator's uniform start()/stop()
        interface, eliminating agent-specific initialization in the blueprint system.

    See also:
        Agent: Base agent class with manual loop control.
        deploy: Convenience helper for standalone agent deployment.
        ModuleCoordinator.start_all_modules: Triggers LlmAgent auto-start.
    """

    @rpc
    def start(self) -> None:
        super().start()
        self.loop_thread()

    @rpc
    def stop(self) -> None:
        super().stop()


llm_agent = LlmAgent.blueprint


def deploy(
    dimos: Annotated[DimosCluster, Doc("The DimosCluster instance for distributed deployment.")],
    system_prompt: Annotated[
        str, Doc("Initial instructions for the LLM agent's behavior.")
    ] = "You are a helpful assistant for controlling a Unitree Go2 robot.",
    model: Annotated[
        Model, Doc("The LLM model to use (e.g., GPT_4O, CLAUDE_35_SONNET).")
    ] = Model.GPT_4O,
    provider: Annotated[
        Provider, Doc("The model provider (e.g., OPENAI, ANTHROPIC).")
    ] = Provider.OPENAI,  # type: ignore[attr-defined]
    skill_containers: Annotated[
        list[SkillContainer] | None,
        Doc("Optional list of skill containers to register with the agent."),
    ] = None,
) -> Annotated[Agent, Doc("The deployed and running agent instance.")]:
    """Convenience helper for deploying a standalone LLM agent with HumanInput skill.

    Creates a fixed configuration: Agent + HumanInput.
    Starts immediately and returns running instance.
    Cannot be composed with other modules after creation.
    If you need to compose modules in a more flexible way, use the blueprint pattern instead.

    Examples:
        >>> # +SKIP: no MockModel injection point; uses real LLM providers
        >>> from dimos import core  # doctest: +SKIP
        >>> from dimos.agents2.skills.demo_calculator_skill import DemoCalculatorSkill  # doctest: +SKIP
        >>> cluster = core.start()  # doctest: +SKIP
        >>> calculator = cluster.deploy(DemoCalculatorSkill)  # doctest: +SKIP
        >>> agent = deploy(  # doctest: +SKIP
        ...     cluster,
        ...     system_prompt="You are a helpful calculator assistant.",
        ...     skill_containers=[calculator]
        ... )

        By contrast, the blueprint pattern is more flexible:

        >>> from dimos.core.blueprints import autoconnect  # doctest: +SKIP
        >>> from dimos.agents2.agent import llm_agent  # doctest: +SKIP
        >>> from dimos.agents2.cli.human import human_input  # doctest: +SKIP
        >>> from dimos.agents2.skills.demo_calculator_skill import demo_calculator_skill  # doctest: +SKIP
        >>>
        >>> blueprint = autoconnect(  # doctest: +SKIP
        ...     demo_calculator_skill(),
        ...     llm_agent(system_prompt="You are a helpful calculator assistant."),
        ...     human_input()
        ... )
        >>> coordinator = blueprint.build()  # doctest: +SKIP
        >>> coordinator.start_all_modules()  # doctest: +SKIP

    See also:
        llm_agent: Blueprint-based agent for flexible composition.
        Agent: The agent class instantiated by this function.
    """
    from dimos.agents2.cli.human import HumanInput

    if skill_containers is None:
        skill_containers = []
    agent = dimos.deploy(  # type: ignore[attr-defined]
        Agent,
        system_prompt=system_prompt,
        model=model,
        provider=provider,
    )

    human_input = dimos.deploy(HumanInput)  # type: ignore[attr-defined]
    human_input.start()

    agent.register_skills(human_input)

    for skill_container in skill_containers:
        print("Registering skill container:", skill_container)
        agent.register_skills(skill_container)

    agent.run_implicit_skill("human")
    agent.start()
    agent.loop_thread()

    return agent  # type: ignore[no-any-return]


__all__ = ["Agent", "deploy", "llm_agent"]
