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

import json
from queue import Empty, Queue
from threading import Event, RLock, Thread
from typing import TYPE_CHECKING, Any, Protocol
import uuid

from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
from langchain_core.tools import StructuredTool
from langgraph.graph.state import CompiledStateGraph
from reactivex.disposable import Disposable

from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.agents.utils import pretty_print_langchain_message
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig, SkillInfo
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.core.stream import In, Out
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT, RPCSpec
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    from langchain_core.language_models import BaseChatModel


class AgentConfig(ModuleConfig):
    system_prompt: str | None = SYSTEM_PROMPT
    model: str = "gpt-4o"
    model_fixture: str | None = None


class Agent(Module[AgentConfig]):
    default_config = AgentConfig

    # on_system_modules imports langchain, creates the agent graph, and calls
    # get_skills() on every module via LCM RPC. This easily exceeds the default
    # 120s, especially on first run when model weights may need to be loaded.
    rpc_timeouts = {"on_system_modules": 180.0}

    agent: Out[BaseMessage]
    human_input: In[str]
    agent_idle: Out[bool]

    _lock: RLock
    _state_graph: CompiledStateGraph[Any, Any, Any, Any] | None
    _message_queue: Queue[BaseMessage]
    _skill_registry: dict[str, SkillInfo]
    _history: list[BaseMessage]
    _thread: Thread
    _stop_event: Event

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._state_graph = None
        self._message_queue = Queue()
        self._history = []
        self._skill_registry = {}
        self._thread = Thread(
            target=self._thread_loop,
            name=f"{self.__class__.__name__}-thread",
            daemon=True,
        )
        self._stop_event = Event()

    @rpc
    def start(self) -> None:
        super().start()

        def _on_human_input(string: str) -> None:
            self._message_queue.put(HumanMessage(content=string))

        self._disposables.add(Disposable(self.human_input.subscribe(_on_human_input)))

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().stop()

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        assert self.rpc is not None

        if self.config.model.startswith("ollama:"):
            from dimos.agents.ollama_agent import ensure_ollama_model

            ensure_ollama_model(self.config.model.removeprefix("ollama:"))

        model: str | BaseChatModel = self.config.model
        if self.config.model_fixture is not None:
            from dimos.agents.testing import MockModel

            model = MockModel(json_path=self.config.model_fixture)

        skills = [skill for module in modules for skill in (module.get_skills() or [])]
        self._skill_registry = {skill.func_name: skill for skill in skills}

        with self._lock:
            # Here to prevent unwanted imports in the file.
            from langchain.agents import create_agent

            self._state_graph = create_agent(
                model=model,
                tools=[_skill_to_tool(self, skill, self.rpc) for skill in skills],
                system_prompt=self.config.system_prompt,
            )
            self._thread.start()

    @rpc
    def add_message(self, message: BaseMessage) -> None:
        self._message_queue.put(message)

    @rpc
    def dispatch_continuation(
        self, continuation: dict[str, Any], continuation_context: dict[str, Any]
    ) -> None:
        """Execute a tool continuation with detection data, bypassing the LLM.

        Called by trigger tools (e.g. look_out_for) to immediately invoke a
        follow-up tool when a detection fires, without waiting for the LLM to
        reason about the next action.

        Args:
            continuation: ``{"tool": "<name>", "args": {…}}`` — the tool to
                call and its arguments.  Argument values that are strings
                starting with ``$`` are treated as template variables and
                resolved against *continuation_context* (e.g. ``"$bbox"``).
            continuation_context: runtime detection data, e.g.
                ``{"bbox": [x1, y1, x2, y2], "label": "person"}``.
        """
        tool_name = continuation.get("tool")
        if not tool_name:
            self._message_queue.put(
                HumanMessage(f"Continuation failed: missing 'tool' key in {continuation}")
            )
            return

        skill_info = self._skill_registry.get(tool_name)
        if skill_info is None:
            self._message_queue.put(
                HumanMessage(f"Continuation failed: tool '{tool_name}' not found")
            )
            return

        tool_args: dict[str, Any] = dict(continuation.get("args", {}))

        # Substitute $-prefixed template variables from continuation_context
        for key, value in tool_args.items():
            if isinstance(value, str) and value.startswith("$"):
                context_key = value[1:]
                if context_key in continuation_context:
                    tool_args[key] = continuation_context[context_key]

        rpc_call = RpcCall(None, self.rpc, skill_info.func_name, skill_info.class_name, [])
        try:
            result = rpc_call(**tool_args)
        except Exception as e:
            self._message_queue.put(
                HumanMessage(f"Continuation '{tool_name}' failed with error: {e}")
            )
            return

        label = continuation_context.get("label", "unknown")
        self._message_queue.put(
            HumanMessage(
                f"Automatically executed '{tool_name}' as a continuation of lookout "
                f"detection (detected: {label}). Result: {result or 'started'}"
            )
        )

    def _thread_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                message = self._message_queue.get(timeout=0.5)
            except Empty:
                continue

            with self._lock:
                if not self._state_graph:
                    raise ValueError("No state graph initialized")
                self._process_message(self._state_graph, message)

    def _process_message(
        self, state_graph: CompiledStateGraph[Any, Any, Any, Any], message: BaseMessage
    ) -> None:
        self.agent_idle.publish(False)
        self._history.append(message)
        pretty_print_langchain_message(message)
        self.agent.publish(message)

        for update in state_graph.stream({"messages": self._history}, stream_mode="updates"):
            for node_output in update.values():
                for msg in node_output.get("messages", []):
                    self._history.append(msg)
                    pretty_print_langchain_message(msg)
                    self.agent.publish(msg)

        if self._message_queue.empty():
            self.agent_idle.publish(True)


class AgentSpec(Spec, Protocol):
    def add_message(self, message: BaseMessage) -> None: ...
    def dispatch_continuation(
        self, continuation: dict[str, Any], continuation_context: dict[str, Any]
    ) -> None: ...


def _skill_to_tool(agent: Agent, skill: SkillInfo, rpc: RPCSpec) -> StructuredTool:
    rpc_call = RpcCall(
        None, rpc, skill.func_name, skill.class_name, [], timeout=DEFAULT_RPC_TIMEOUT
    )

    def wrapped_func(*args: Any, **kwargs: Any) -> str | list[dict[str, Any]]:
        result = None

        try:
            result = rpc_call(*args, **kwargs)
        except Exception as e:
            return f"Exception: Error: {e}"

        if result is None:
            return "It has started. You will be updated later."

        if hasattr(result, "agent_encode"):
            uuid_ = str(uuid.uuid4())
            _append_image_to_history(agent, skill, uuid_, result)
            return f"Tool call started with UUID: {uuid_}"

        return str(result)

    return StructuredTool(
        name=skill.func_name,
        func=wrapped_func,
        args_schema=json.loads(skill.args_schema),
    )


def _append_image_to_history(agent: Agent, skill: SkillInfo, uuid_: str, result: Any) -> None:
    agent.add_message(
        HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": f"This is the artefact for the '{skill.func_name}' tool with UUID:={uuid_}.",
                },
                *result.agent_encode(),
            ]
        )
    )
