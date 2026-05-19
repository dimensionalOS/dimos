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

import json
from collections.abc import Callable
from queue import Empty, Queue
from threading import Event, RLock, Thread
import time
from typing import Any
import uuid

import httpx
from langchain.agents import create_agent
from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
from langchain_core.tools import StructuredTool
from langgraph.graph.state import CompiledStateGraph
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

_RECORD_TOOL_NAME = "record_skill_outcome"
_OUTCOME_SKIP_TOOLS = frozenset(
    {
        "get_context",
        "route_task",
        "predict_skill_outcome",
        "record_skill_outcome",
        "summarize_skill_outcomes",
        "server_status",
        "list_modules",
        "agent_send",
    }
)
_TOOL_DOMAINS = {
    "navigate_with_text": "navigation",
    "stop_navigation": "navigation",
    "relative_move": "robot_motion",
    "execute_sport_command": "robot_motion",
    "follow_person": "person_follow",
    "stop_following": "person_follow",
    "look_out_for": "perception",
    "stop_looking_out": "perception",
    "tag_location": "memory",
    "start_security_patrol": "security",
    "stop_security_patrol": "security",
    "speak": "speech",
    "wait": "utility",
    "current_time": "utility",
}


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

    def _mcp_tool_call(
        self, name: str, arguments: dict[str, Any], record_outcome: bool = True
    ) -> dict[str, Any]:
        progress_token = str(uuid.uuid4())
        try:
            result = self._mcp_request(
                "tools/call",
                {
                    "name": name,
                    "arguments": arguments,
                    "_meta": {"progressToken": progress_token},
                },
            )
        except Exception as exc:
            if record_outcome:
                self._record_tool_exception(name, exc)
            raise

        if record_outcome:
            self._record_tool_outcome(name, result)
        return result

    def _should_record_tool_outcome(self, name: str) -> bool:
        return name not in _OUTCOME_SKIP_TOOLS and _RECORD_TOOL_NAME in self._tool_registry

    def _record_tool_exception(self, name: str, exc: Exception) -> None:
        if not self._should_record_tool_outcome(name):
            return
        self._record_tool_payload(
            {
                "skill_name": name,
                "success": False,
                "domain": _infer_tool_domain(name),
                "error_code": "EXECUTION_FAILED",
                "message": _truncate(f"MCP request failed: {exc}"),
                "risk": "high",
                "recovery": "Inspect the tool error before retrying.",
            }
        )

    def _record_tool_outcome(self, name: str, result: dict[str, Any]) -> None:
        if not self._should_record_tool_outcome(name):
            return
        self._record_tool_payload(_outcome_payload_from_result(name, result))

    def _record_tool_payload(self, payload: dict[str, Any]) -> None:
        try:
            self._mcp_tool_call(_RECORD_TOOL_NAME, payload, record_outcome=False)
        except Exception:
            logger.warning("Failed to record MCP tool outcome", tool=payload["skill_name"])

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

        def call_tool(**kwargs: Any) -> str:
            result = self._mcp_tool_call(name, kwargs)
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

        if self._message_queue.empty():
            self.agent_idle.publish(True)


def _append_image_to_history(
    mcp_client: McpClient, func_name: str, uuid_: str, result: Any
) -> None:
    mcp_client.add_message(
        HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": f"This is the artefact for the '{func_name}' tool with UUID:={uuid_}.",
                },
                result,
            ]
        )
    )


def _outcome_payload_from_result(name: str, result: dict[str, Any]) -> dict[str, Any]:
    text = _content_text(result)
    structured = _parse_skill_result_text(text)
    if structured is not None:
        success = bool(structured.get("success", False))
        message = str(structured.get("message") or "")
        error_code = str(structured.get("error_code") or "")
        raw_metadata = structured.get("metadata")
        metadata = raw_metadata if isinstance(raw_metadata, dict) else {}
        risk = str(metadata.get("risk") or ("unknown" if success else "medium"))
        recovery = _recovery_text(metadata)
    else:
        message = text
        failure = _looks_like_failure_text(text)
        success = not failure
        error_code = "EXECUTION_FAILED" if failure else ""
        risk = "high" if failure else "unknown"
        recovery = "Inspect the tool error before retrying." if failure else ""

    return {
        "skill_name": name,
        "success": success,
        "domain": _infer_tool_domain(name),
        "error_code": error_code,
        "message": _truncate(message),
        "risk": _normalize_risk(risk),
        "recovery": _truncate(recovery),
    }


def _content_text(result: dict[str, Any]) -> str:
    content = result.get("content", [])
    if not isinstance(content, list):
        return ""
    parts = [item.get("text", "") for item in content if isinstance(item, dict)]
    return "\n".join(part for part in parts if part)


def _parse_skill_result_text(text: str) -> dict[str, Any] | None:
    text = text.strip()
    if not text.startswith("{"):
        return None
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(parsed, dict) or "success" not in parsed:
        return None
    return parsed


def _looks_like_failure_text(text: str) -> bool:
    lowered = text.strip().casefold()
    return lowered.startswith("error running tool") or lowered.startswith("tool not found")


def _infer_tool_domain(name: str) -> str:
    return _TOOL_DOMAINS.get(name, "")


def _normalize_risk(risk: str) -> str:
    risk = risk.strip().casefold()
    if risk in {"low", "medium", "high", "unknown"}:
        return risk
    return "unknown"


def _recovery_text(metadata: dict[str, Any]) -> str:
    recovery = metadata.get("recovery")
    if isinstance(recovery, str):
        return recovery
    suggestions = metadata.get("recovery_suggestions")
    if isinstance(suggestions, list):
        return "; ".join(str(item) for item in suggestions)
    return ""


def _truncate(text: str, limit: int = 500) -> str:
    return text[:limit]
