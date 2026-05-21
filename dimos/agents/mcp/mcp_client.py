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

from collections.abc import Callable
from queue import Empty, Queue
from threading import Event, RLock, Thread
import time
from typing import Annotated, Any, cast
import uuid

import httpx
from langchain.agents import AgentState, create_agent
from langchain_core.messages import AIMessage, HumanMessage, ToolCall, ToolMessage
from langchain_core.messages.base import BaseMessage
from langchain_core.tools import InjectedToolCallId, StructuredTool
from langgraph.graph.message import add_messages
from langgraph.graph.state import CompiledStateGraph
from langgraph.types import Command
from reactivex.disposable import Disposable

from dimos.agents.mcp import tool_stream
from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.agents.utils import pretty_print_langchain_message
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

logger = setup_logger()


def _fix_parallel_tool_batches(messages: list[BaseMessage]) -> list[BaseMessage]:
    """Reorder interleaved [Tool, Human, Tool, Human, ...] runs that
    follow a parallel-tool-call AIMessage into [Tool, Tool, ..., Human,
    Human, ...] so OpenAI's "all parallel tool responses must be
    contiguous" rule is satisfied.

    Image-carrying HumanMessages emitted by the MCP tool wrapper are
    tagged with `additional_kwargs["tool_call_id"]` matching the
    originating tool call, which is how we pair each Human with its
    parallel batch.
    """
    out = list(messages)
    i = 0
    while i < len(out):
        msg = out[i]
        tool_calls = getattr(msg, "tool_calls", None) or []
        if isinstance(msg, AIMessage) and len(tool_calls) >= 2:
            expected_ids = {tc.get("id") for tc in tool_calls if tc.get("id")}
            tool_msgs: list[ToolMessage] = []
            other_msgs: list[BaseMessage] = []
            j = i + 1
            while j < len(out):
                m = out[j]
                if isinstance(m, ToolMessage) and m.tool_call_id in expected_ids:
                    tool_msgs.append(m)
                    j += 1
                elif (
                    isinstance(m, HumanMessage)
                    and getattr(m, "additional_kwargs", {}).get("tool_call_id") in expected_ids
                ):
                    other_msgs.append(m)
                    j += 1
                else:
                    break
            if tool_msgs and other_msgs and {m.tool_call_id for m in tool_msgs} == expected_ids:
                out[i + 1 : j] = [*tool_msgs, *other_msgs]
        i += 1
    return out


def _reorder_tool_responses(
    left: list[BaseMessage], right: list[BaseMessage] | BaseMessage
) -> list[BaseMessage]:
    """Standard add_messages merge, then fix any parallel-tool batches."""
    # add_messages is typed against langgraph's permissive Messages union;
    # list[BaseMessage] is invariant so we cast at the boundary.
    merged = cast("list[BaseMessage]", add_messages(cast("Any", left), cast("Any", right)))
    return _fix_parallel_tool_batches(merged)


class _OrderedAgentState(AgentState[Any]):
    # Override the messages reducer to keep parallel ToolMessages contiguous.
    messages: Annotated[list[BaseMessage], _reorder_tool_responses]  # type: ignore[misc]


class McpClientConfig(ModuleConfig):
    system_prompt: str | None = SYSTEM_PROMPT
    model: str = "gpt-4o"
    model_fixture: str | None = None
    mcp_server_url: str = "http://localhost:9990/mcp"


class McpClient(Module):
    config: McpClientConfig
    agent: Out[BaseMessage]
    human_input: In[str]
    agent_idle: Out[bool]

    _lock: RLock
    _state_graph: CompiledStateGraph[Any, Any, Any, Any] | None
    _message_queue: Queue[BaseMessage]
    _tool_registry: dict[str, dict[str, Any]]
    _history: list[BaseMessage]
    _thread: Thread
    _stop_event: Event
    _http_client: httpx.Client
    _seq_ids: SequentialIds
    _tool_stream_cleanup: Callable[[], None] | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._state_graph = None
        self._message_queue = Queue()
        self._tool_registry = {}
        self._history = []
        self._thread = Thread(
            target=self._thread_loop,
            name=f"{self.__class__.__name__}-thread",
            daemon=True,
        )
        self._stop_event = Event()
        self._http_client = httpx.Client(timeout=120.0)
        self._seq_ids = SequentialIds()
        self._tool_stream_cleanup = None

    def __reduce__(self) -> Any:
        return (self.__class__, (), {})

    def _mcp_request(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        body: dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": self._seq_ids.next(),
            "method": method,
        }
        if params is not None:
            body["params"] = params

        resp = self._http_client.post(self.config.mcp_server_url, json=body)
        resp.raise_for_status()
        data = resp.json()

        if "error" in data:
            raise RuntimeError(f"MCP error {data['error']['code']}: {data['error']['message']}")

        result: dict[str, Any] = data.get("result")
        return result

    def _mcp_tool_call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        progress_token = str(uuid.uuid4())
        return self._mcp_request(
            "tools/call",
            {
                "name": name,
                "arguments": arguments,
                "_meta": {"progressToken": progress_token},
            },
        )

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

    def _fetch_tools(self, timeout: float = 60.0, interval: float = 1.0) -> list[StructuredTool]:
        result = self._try_fetch_tools(timeout=timeout, interval=interval)
        if result is None:
            raise RuntimeError(
                f"Failed to fetch tools from MCP server {self.config.mcp_server_url}"
            )

        raw_tools = result.get("tools", [])
        self._tool_registry = {t["name"]: t for t in raw_tools}
        tools = [self._mcp_tool_to_langchain(t) for t in raw_tools]

        if not tools:
            logger.warning("No tools found from MCP server.")
        else:
            tool_names = [t.name for t in tools]
            logger.info("Discovered tools from MCP server.", tools=tool_names, n_tools=len(tools))

        return tools

    def _try_fetch_tools(self, timeout: float, interval: float) -> dict[str, Any] | None:
        deadline = time.monotonic() + timeout

        while True:
            try:
                self._mcp_request("initialize")
                break
            except (httpx.ConnectError, httpx.RemoteProtocolError):
                if time.monotonic() >= deadline:
                    return None
                time.sleep(interval)

        return self._mcp_request("tools/list")

    def _mcp_tool_to_langchain(self, mcp_tool: dict[str, Any]) -> StructuredTool:
        name = mcp_tool["name"]
        description = mcp_tool.get("description", "")
        input_schema = mcp_tool.get("inputSchema", {"type": "object", "properties": {}})

        def call_tool(
            tool_call_id: Annotated[str, InjectedToolCallId],
            **kwargs: Any,
        ) -> str | Command[Any]:
            result = self._mcp_tool_call(name, kwargs)
            content = result.get("content", [])
            text = "\n".join(c.get("text", "") for c in content if c.get("type") == "text")
            image_blocks = [c for c in content if c.get("type") != "text"]

            if not image_blocks:
                return text

            # Vision content can't be embedded inside a ToolMessage for OpenAI
            # (and others), so we use Command to append a follow-up HumanMessage
            # carrying the image blocks within the same agent turn.
            #
            # The HumanMessage is tagged with `additional_kwargs["tool_call_id"]`
            # so `_fix_parallel_tool_batches` can pair it with the right
            # ToolMessage when multiple parallel tool calls return images
            # in one batch (OpenAI requires the parallel ToolMessages to
            # stay contiguous).
            summary = text or f"{name} returned {len(image_blocks)} non-text artefact(s)."
            intro = f"Artefacts returned by '{name}' (image follows):"
            return Command(
                update={
                    "messages": [
                        ToolMessage(content=summary, tool_call_id=tool_call_id),
                        HumanMessage(
                            content=[{"type": "text", "text": intro}, *image_blocks],
                            additional_kwargs={"tool_call_id": tool_call_id},
                        ),
                    ]
                }
            )

        return _McpStructuredTool(
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
        tools = self._fetch_tools()

        model: str | Any = self.config.model
        if self.config.model_fixture is not None:
            from dimos.agents.testing import MockModel

            model = MockModel(json_path=self.config.model_fixture)

        with self._lock:
            self._state_graph = create_agent(
                model=model,
                tools=tools,
                system_prompt=self.config.system_prompt,
                state_schema=cast("type[AgentState[Any]]", _OrderedAgentState),
            )
            if not self._thread.is_alive():
                self._thread.start()

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
        self._http_client.close()
        super().stop()

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

        if tool_name not in self._tool_registry:
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
            result = self._mcp_tool_call(tool_name, tool_args)
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
        self._history.append(message)
        pretty_print_langchain_message(message)
        self.agent.publish(message)

        for update in state_graph.stream({"messages": self._history}, stream_mode="updates"):
            for node_output in update.values():
                for msg in node_output.get("messages", []):
                    self._history.append(msg)
                    pretty_print_langchain_message(msg)
                    self.agent.publish(msg)

        # The graph applies _reorder_tool_responses to its internal channel,
        # but stream_mode="updates" emits raw node outputs in completion
        # order — and langgraph does not re-run reducers when an initial
        # state dict is fed back into stream() on the next turn. Mirror the
        # reducer here so _history matches what the graph would produce.
        self._history = _fix_parallel_tool_batches(self._history)

        if self._message_queue.empty():
            self.agent_idle.publish(True)


class _McpStructuredTool(StructuredTool):
    """StructuredTool that propagates `tool_call_id` to MCP tools whose
    `args_schema` is a JSON-Schema dict.

    Langchain auto-injects `InjectedToolCallId` only when `args_schema` is
    a Pydantic model; MCP servers supply JSON-Schema dicts (the server's
    authoritative contract for the LLM), so we have to bridge ourselves.

    The bridge uses only public Runnable surface — `invoke` / `ainvoke`
    accept a `ToolCall` dict as documented input, and we copy its `id`
    field into the function's kwargs before delegating.
    """

    def invoke(
        self,
        input: str | dict[Any, Any] | ToolCall,
        config: Any = None,
        **kwargs: Any,
    ) -> Any:
        return super().invoke(_inject_tool_call_id(input), config=config, **kwargs)

    async def ainvoke(
        self,
        input: str | dict[Any, Any] | ToolCall,
        config: Any = None,
        **kwargs: Any,
    ) -> Any:
        return await super().ainvoke(_inject_tool_call_id(input), config=config, **kwargs)


def _inject_tool_call_id(
    input: str | dict[Any, Any] | ToolCall,
) -> dict[Any, Any]:
    """Copy the `ToolCall.id` field into `args.tool_call_id` so the inner
    `call_tool` closure receives it as a kwarg. JSON-Schema dicts don't
    validate against extra keys, so this is transparent to the schema.

    Raises ValueError on any invocation that isn't a `ToolCall`-shaped
    dict with a non-null `id` — MCP tools have no other valid call path.
    """
    if not (isinstance(input, dict) and input.get("type") == "tool_call"):
        raise ValueError(
            "MCP tool must be invoked via a ToolCall (a dict with "
            "type='tool_call' and a non-null id), not a bare input."
        )
    tool_call_id = input.get("id")
    if tool_call_id is None:
        raise ValueError("MCP tool ToolCall is missing a non-null id.")
    return {**input, "args": {**(input.get("args") or {}), "tool_call_id": tool_call_id}}
