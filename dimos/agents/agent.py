# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Iterator
from pathlib import Path
from queue import Empty, Queue
from threading import Event, RLock, Thread
from typing import Any
import uuid
import warnings

from langchain_core._api.deprecation import LangChainPendingDeprecationWarning

# Importing langchain_core un-mutes its pending-deprecation warnings, so this ignore
# must be registered after that import to take precedence. It silences the noisy
# `allowed_objects` warning emitted when langchain.agents pulls in langgraph.checkpoint.
warnings.filterwarnings("ignore", category=LangChainPendingDeprecationWarning)

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
from langchain_core.tools import StructuredTool
from langgraph.graph.state import CompiledStateGraph
from reactivex.disposable import Disposable

from dimos.agents.base_agent import BaseAgent
from dimos.agents.llm_trace import tracing_http_client
from dimos.agents.mcp import tool_stream
from dimos.agents.mcp.constants import TOOL_ORIGIN_KEY, TURN_FAILED_KEY
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.agents.mcp.spec import McpServerSpec
from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.agents.utils import pretty_print_langchain_message
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RESPONSES_REASONING_MODEL_PREFIXES = ("gpt-5", "o1", "o3", "o4")
# How much of a failed turn's error message goes into the agent's history.
_FAILED_TURN_ERROR_CHARS = 300
_MCP_TIMEOUT_S = 120
_MCP_READY_TIMEOUT_S = 60.0


def _preload_agent_deps() -> None:
    import langchain.agents
    import langchain.chat_models  # noqa: F401
    import langchain_openai  # noqa: F401


def init_model(model_name: str, trace_dir: Path | None = None) -> Any:
    """Initialize a model while preserving LangChain provider resolution.

    With *trace_dir*, every request/response body goes to disk whole
    (:mod:`dimos.agents.llm_trace`). Only OpenAI-backed models take the
    ``http_client``; other providers keep working, untraced at the wire.
    """
    # ~2s: langchain's chat-model machinery pulls transformers+torch; deferred to
    # keep module import (test collection, CLI startup) light.
    from langchain.chat_models import init_chat_model
    from langchain_openai import ChatOpenAI

    client = None if trace_dir is None else tracing_http_client(trace_dir)
    if ":" in model_name or not model_name.startswith(_RESPONSES_REASONING_MODEL_PREFIXES):
        model = init_chat_model(model=model_name)
        if client is not None and isinstance(model, ChatOpenAI):
            return init_chat_model(model=model_name, http_client=client)
        return model

    return ChatOpenAI(
        model=model_name,
        use_responses_api=True,
        reasoning={"effort": "medium", "summary": "auto"},
        http_client=client,
    )


class AgentConfig(ModuleConfig):
    system_prompt: str | None = SYSTEM_PROMPT
    model: str = "gpt-5.6-luna"
    model_fixture: str | None = None
    # None: the McpServer in this blueprint; without one the agent has no tools.
    mcp_server_url: str | None = None
    trace_dir: Path | None = None


class Agent(BaseAgent):
    config: AgentConfig

    # Injected when the blueprint has an McpServer.
    _mcp_server: McpServerSpec | None = None

    _lock: RLock
    _state_graph: CompiledStateGraph[Any, Any, Any, Any] | None
    _message_queue: Queue[BaseMessage]
    _agent_tools: list[StructuredTool] | None
    _tool_registry: dict[str, dict[str, Any]]
    _history: list[BaseMessage]
    _thread: Thread
    _stop_event: Event
    _mcp: McpAdapter | None
    _tool_stream_cleanup: Callable[[], None] | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._state_graph = None
        self._message_queue = Queue()
        self._agent_tools = None
        self._tool_registry = {}
        self._history = []
        self._thread = Thread(
            target=self._thread_loop,
            name=f"{self.__class__.__name__}-thread",
            daemon=True,
        )
        self._stop_event = Event()
        self._mcp = None
        self._tool_stream_cleanup = None
        Thread(target=_preload_agent_deps, name="Agent-preload", daemon=True).start()

    def _resolve_mcp(self) -> McpAdapter | None:
        """The MCP endpoint: configured, else the blueprint's McpServer, else none."""
        if self.config.mcp_server_url is not None:
            url = self.config.mcp_server_url
        elif self._mcp_server is not None:
            url = str(self._mcp_server.mcp_url())
        else:
            return None
        return McpAdapter(url, timeout=_MCP_TIMEOUT_S)

    def _mcp_tool_call(
        self, mcp: McpAdapter, name: str, arguments: dict[str, Any]
    ) -> dict[str, Any]:
        return mcp.call_tool(name, arguments, meta={"progressToken": str(uuid.uuid4())})

    def _on_tool_stream_message(self, msg: dict[str, Any]) -> None:
        method = msg.get("method")
        params = msg.get("params") or {}
        if method == tool_stream.NOTIFICATIONS_PROGRESS_METHOD:
            text = params.get("message") or ""
            tool_name = (params.get("_meta") or {}).get("tool_name") or "tool"
        elif method == tool_stream.NOTIFICATIONS_MESSAGE_METHOD:
            text = params.get("data") or ""
            tool_name = params.get("logger") or "tool"
        else:
            return
        if not text:
            return
        self._message_queue.put(HumanMessage(content=f"[tool:{tool_name}] {text}"))

    def _fetch_tools(self, mcp: McpAdapter) -> list[StructuredTool]:
        if not mcp.wait_for_ready(timeout=_MCP_READY_TIMEOUT_S):
            raise RuntimeError(f"Failed to fetch tools from MCP server {mcp.url}")

        raw_tools = mcp.list_tools()
        self._tool_registry = {t["name"]: t for t in raw_tools}
        tools = [self._mcp_tool_to_langchain(mcp, t) for t in raw_tools]

        if not tools:
            logger.warning("No tools found from MCP server.")
        else:
            tool_names = [t.name for t in tools]
            logger.info("Discovered tools from MCP server.", tools=tool_names, n_tools=len(tools))

        return tools

    def _mcp_tool_to_langchain(self, mcp: McpAdapter, mcp_tool: dict[str, Any]) -> StructuredTool:
        name = mcp_tool["name"]
        description = mcp_tool.get("description", "")
        input_schema = mcp_tool.get("inputSchema", {"type": "object", "properties": {}})

        def call_tool(**kwargs: Any) -> str:
            result = self._mcp_tool_call(mcp, name, kwargs)
            content = result.get("content", [])
            parts = [c.get("text", "") for c in content if c.get("type") == "text"]
            text = "\n".join(parts)

            # Images need to be added to the history separately because they
            # cannot be included in the tool response for OpenAI models and
            # probably others.
            for item in content:
                if item.get("type") != "text":
                    uuid_ = str(uuid.uuid4())
                    text += f"Tool call started with UUID: {uuid_}. You will be updated with the result soon."
                    _append_image_to_history(self, name, uuid_, item)

            return text

        return StructuredTool(
            name=name,
            description=description,
            func=call_tool,
            args_schema=input_schema,
        )

    @rpc
    def start(self) -> None:
        super().start()

        def _on_human_input(string: str) -> None:
            self._message_queue.put(HumanMessage(content=string))

        self.register_disposable(Disposable(self.human_input.subscribe(_on_human_input)))

        # Subscribe directly over LCM rather than through the server's GET
        # /mcp SSE channel.  HTTP would add a startup race: the first few
        # updates of a short-lived stream can fire before the SSE connection
        # is established.  External clients like Claude Code still use GET
        # /mcp, which the server fans out to from the same LCM topic.
        self._tool_stream_cleanup = tool_stream.subscribe(self._on_tool_stream_message)

    @rpc
    def on_system_modules(self, _modules: list[RPCClient]) -> None:
        self._mcp = self._resolve_mcp()
        if self._mcp is None:
            logger.info("No MCP server configured; the agent runs without tools.")
            self._agent_tools = []
        else:
            self._agent_tools = self._fetch_tools(self._mcp)
        self._rebuild_agent()
        with self._lock:
            if not self._thread.is_alive():
                self._thread.start()

    def _rebuild_agent(self) -> None:
        # ~2s: pulls transformers+torch; deferred to keep module import light.
        from langchain.agents import create_agent

        # Under the lock, or a concurrent set_trace_dir can lose its path to this build.
        with self._lock:
            if self.config.model_fixture is not None:
                from dimos.agents.testing.mock_model import MockModel

                model = MockModel(json_path=self.config.model_fixture)
            else:
                model = init_model(self.config.model, trace_dir=self.config.trace_dir)
            self._state_graph = create_agent(
                model=model,
                tools=self._agent_tools or [],
                system_prompt=self.config.system_prompt,
            )

    @rpc
    def stop(self) -> None:
        # Unsubscribe first so no new tool-stream messages can arrive while
        # the worker thread is draining and joining.
        if self._tool_stream_cleanup is not None:
            self._tool_stream_cleanup()
            self._tool_stream_cleanup = None
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @rpc
    def trace_dir(self) -> Path | None:
        """Where raw LLM request/response bodies go; ``None`` when tracing is off."""
        return self.config.trace_dir

    @rpc
    def set_trace_dir(self, path: str | None) -> None:
        """Point raw LLM capture at *path* and rebuild the model to pick it
        up; ``None`` turns tracing off."""
        with self._lock:
            self.config.trace_dir = Path(path) if path is not None else None
            if self._state_graph is not None:
                self._rebuild_agent()

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

        mcp = self._mcp
        if mcp is None or tool_name not in self._tool_registry:
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

        try:
            result = self._mcp_tool_call(mcp, tool_name, tool_args)
            content = result.get("content", [])
            parts = [c.get("text", "") for c in content if c.get("type") == "text"]
            text = "\n".join(parts)
        except Exception as e:
            self._message_queue.put(
                HumanMessage(f"Continuation '{tool_name}' failed with error: {e}")
            )
            return

        label = continuation_context.get("label", "unknown")
        self._message_queue.put(
            HumanMessage(
                f"Automatically executed '{tool_name}' as a continuation of lookout "
                f"detection (detected: {label}). Result: {text or 'started'}"
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
        turn_start = len(self._history)
        self._record(message)

        try:
            for update in state_graph.stream({"messages": self._history}, stream_mode="updates"):
                for node_output in update.values():
                    for msg in node_output.get("messages", []):
                        self._record(msg)
        except Exception as e:
            # Letting this escape would end the thread, and no later message, tool
            # updates included, would be processed. Log it and return; the failed
            # message is not retried.
            logger.exception("Agent turn failed", error=f"{type(e).__name__}: {e}")
            failure = f"{type(e).__name__}: {str(e)[:_FAILED_TURN_ERROR_CHARS]}"
            # Model APIs reject a history with an unanswered tool call, so answer
            # each one with an error.
            for call_id in _unanswered_tool_calls(self._history[turn_start + 1 :]):
                self._record(
                    ToolMessage(
                        content=(
                            f"No result: the agent turn failed with {failure}. "
                            "The call may or may not have run."
                        ),
                        tool_call_id=call_id,
                        status="error",
                    )
                )
            self._record(
                AIMessage(
                    content=f"This turn failed with {failure}",
                    additional_kwargs={TURN_FAILED_KEY: True},
                )
            )

        if self._message_queue.empty():
            self.agent_idle.publish(True)

    def _record(self, message: BaseMessage) -> None:
        self._history.append(message)
        pretty_print_langchain_message(message)
        self.agent.publish(message)


def _unanswered_tool_calls(messages: list[BaseMessage]) -> Iterator[str]:
    answered = {m.tool_call_id for m in messages if isinstance(m, ToolMessage)}
    for message in messages:
        if not isinstance(message, AIMessage):
            continue
        for call in message.tool_calls:
            call_id = call["id"]
            if call_id is not None and call_id not in answered:
                yield call_id


def _append_image_to_history(agent: Agent, func_name: str, uuid_: str, result: Any) -> None:
    # Tagged with its tool, or chat UIs would show this user turn as typed input.
    agent.add_message(
        HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": f"This is the artefact for the '{func_name}' tool with UUID:={uuid_}.",
                },
                result,
            ],
            additional_kwargs={TOOL_ORIGIN_KEY: func_name},
        )
    )
