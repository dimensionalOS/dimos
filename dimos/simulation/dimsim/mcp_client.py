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

"""DimSim-specific MCP client behavior."""

from queue import Empty
from threading import Event, Lock, Thread
from typing import Any

from langchain.agents import create_agent
from langchain.tools import ToolRuntime
from langchain_core.messages import HumanMessage, ToolMessage
from langchain_core.tools import StructuredTool
from langchain_openai import ChatOpenAI
from langgraph.types import Command
from reactivex.disposable import Disposable

from dimos.agents.mcp import tool_stream
from dimos.agents.mcp.mcp_client import McpClient, _init_model
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.core.transport_factory import make_transport
from dimos.simulation.dimsim.agent_turn_control import (
    CANCEL_ACTIVE_TURN,
    EVAL_TURN_CONTROL_TOPIC,
)
from dimos.simulation.dimsim.spatial_memory import DimSimSpatialMemorySpec

_EVAL_IDLE_HEARTBEAT_SEC = 0.25
_EVAL_TURN_ENDED_TEXT = "The DimSim evaluation has ended. End this agent turn now."


class DimSimMcpClient(McpClient):
    """Keep camera observations in the tool call that requested them."""

    _spatial_memory: DimSimSpatialMemorySpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._eval_idle_lock = Lock()
        self._eval_idle = True
        self._live_update_lock = Lock()
        self._processing_turn = False
        self._live_updates: list[HumanMessage] = []
        self._eval_turn_cancel = Event()
        self._eval_agent_tools: list[StructuredTool] = []
        self._eval_control_transport: Any | None = None
        self._eval_control_unsubscribe: Any | None = None
        self._eval_idle_stop = Event()
        self._eval_idle_thread = Thread(
            target=self._eval_idle_heartbeat,
            name=f"{self.__class__.__name__}-idle-heartbeat",
            daemon=True,
        )

    @rpc
    def start(self) -> None:
        Module.start(self)
        self.register_disposable(
            Disposable(self.human_input.subscribe(self._queue_human_input)),
        )
        self._tool_stream_cleanup = tool_stream.subscribe(self._on_tool_stream_message)
        self._eval_control_transport = make_transport(EVAL_TURN_CONTROL_TOPIC)
        self._eval_control_unsubscribe = self._eval_control_transport.subscribe(
            self._on_eval_turn_control,
        )
        if not self._eval_idle_thread.is_alive():
            self._eval_idle_thread.start()

    def _set_eval_idle(self, idle: bool) -> None:
        with self._eval_idle_lock:
            self._eval_idle = idle
        self.agent_idle.publish(idle)

    def _eval_idle_heartbeat(self) -> None:
        while not self._eval_idle_stop.wait(_EVAL_IDLE_HEARTBEAT_SEC):
            with self._eval_idle_lock:
                idle = self._eval_idle
            self.agent_idle.publish(idle)

    def _process_message(self, state_graph: Any, message: Any) -> None:
        with self._live_update_lock:
            self._eval_turn_cancel.clear()
            self._processing_turn = True
        self._set_eval_idle(False)
        try:
            super()._process_message(state_graph, message)
        finally:
            with self._live_update_lock:
                self._processing_turn = False
                self._eval_turn_cancel.clear()
                remaining_updates = self._live_updates
                self._live_updates = []
            for tool in self._eval_agent_tools:
                tool.return_direct = False
            for update in remaining_updates:
                self._message_queue.put(update)
            self._set_eval_idle(self._message_queue.empty())

    @rpc
    def stop(self) -> None:
        if self._eval_control_unsubscribe is not None:
            self._eval_control_unsubscribe()
            self._eval_control_unsubscribe = None
        if self._eval_control_transport is not None:
            self._eval_control_transport.stop()
            self._eval_control_transport = None
        self._eval_idle_stop.set()
        if self._eval_idle_thread.is_alive():
            self._eval_idle_thread.join(timeout=1.0)
        super().stop()

    def _on_eval_turn_control(self, message: Any) -> None:
        if (
            not isinstance(message, dict)
            or message.get("type") != CANCEL_ACTIVE_TURN
            or not isinstance(message.get("runId"), str)
            or not message["runId"]
        ):
            return
        with self._live_update_lock:
            # A late packet from a completed run must not poison the next task.
            if self._processing_turn:
                self._eval_turn_cancel.set()

    def _cancelled_tool_result(
        self,
        tool: StructuredTool,
        result: str = "",
    ) -> str | None:
        if not self._eval_turn_cancel.is_set():
            return None
        tool.return_direct = True
        return self._append_update_text(result, _EVAL_TURN_ENDED_TEXT)

    def _queue_human_input(self, string: str) -> None:
        # Each DimSim task follows an authoritative simulator reset and must
        # not inherit model messages, queued tool updates, or semantic-map
        # viewpoints from a previous run. The lock also prevents resetting
        # history while the prior agent turn is still streaming.
        with self._lock:
            self._spatial_memory.clear_eval_memory()
            self._history.clear()
            with self._live_update_lock:
                self._live_updates.clear()
            while True:
                try:
                    self._message_queue.get_nowait()
                except Empty:
                    break
            self._message_queue.put(HumanMessage(content=string))

    def _enqueue_agent_update(self, message: HumanMessage) -> None:
        """Deliver background events into the active tool loop when possible.

        LangGraph runs one agent turn until the model stops calling tools.
        Upstream background notifications are queued for the *next* turn, which
        means a moving robot can execute several more actions before learning
        that a lookout matched. A DimSim evaluation needs the match to be
        visible at the next tool boundary so perception and pose stay
        correlated.
        """
        with self._live_update_lock:
            if self._processing_turn:
                self._live_updates.append(message)
                return
        self._message_queue.put(message)

    def _take_live_update_text(self) -> str:
        with self._live_update_lock:
            updates = self._live_updates
            self._live_updates = []
        return "\n".join(str(update.content) for update in updates)

    @staticmethod
    def _append_update_text(text: str, update: str) -> str:
        if not update:
            return text
        if not text:
            return update
        return f"{text}\n\n{update}"

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
        if text:
            self._enqueue_agent_update(
                HumanMessage(content=f"[tool:{tool_name}] {text}"),
            )

    @rpc
    def dispatch_continuation(
        self,
        continuation: dict[str, Any],
        continuation_context: dict[str, Any],
    ) -> None:
        """Execute a lookout continuation and surface its result immediately."""
        tool_name = continuation.get("tool")
        if not tool_name:
            self._enqueue_agent_update(
                HumanMessage(
                    content=f"Continuation failed: missing 'tool' key in {continuation}",
                ),
            )
            return

        if tool_name not in self._tool_registry:
            self._enqueue_agent_update(
                HumanMessage(content=f"Continuation failed: tool '{tool_name}' not found"),
            )
            return

        tool_args: dict[str, Any] = dict(continuation.get("args", {}))
        for key, value in tool_args.items():
            if isinstance(value, str) and value.startswith("$"):
                context_key = value[1:]
                if context_key in continuation_context:
                    tool_args[key] = continuation_context[context_key]

        try:
            result = self._mcp_tool_call(tool_name, tool_args)
            content = result.get("content", [])
            parts = [item.get("text", "") for item in content if item.get("type") == "text"]
            text = "\n".join(parts)
        except Exception as exc:
            self._enqueue_agent_update(
                HumanMessage(
                    content=f"Continuation '{tool_name}' failed with error: {exc}",
                ),
            )
            return

        label = continuation_context.get("label", "unknown")
        self._enqueue_agent_update(
            HumanMessage(
                content=(
                    f"Automatically executed '{tool_name}' as a continuation of "
                    f"lookout detection (detected: {label}). Result: "
                    f"{text or 'started'}"
                ),
            ),
        )

    @rpc
    def on_system_modules(self, _modules: list[RPCClient]) -> None:
        self._eval_agent_tools = []
        tools = self._fetch_tools()
        model: Any

        if self.config.model_fixture is not None:
            from dimos.agents.testing.mock_model import MockModel

            model = MockModel(json_path=self.config.model_fixture)
        else:
            model = _init_model(self.config.model)
            # The configured coding-plan proxy exposes the Responses API as an
            # event stream. Its non-streaming response is not OpenAI-compatible,
            # while LangChain correctly merges the streamed text and tool-call
            # events into complete AIMessages. Keep this compatibility setting
            # local to the DimSim adapter.
            if isinstance(model, ChatOpenAI) and model.use_responses_api:
                model = model.model_copy(update={"streaming": True})

        with self._lock:
            self._state_graph = self._create_eval_agent(model, tools)
            if not self._thread.is_alive():
                self._thread.start()
            self._set_eval_idle(self._message_queue.empty())

    def _create_eval_agent(
        self,
        model: Any,
        tools: list[StructuredTool],
    ) -> Any:
        # create_agent only compiles its tools -> END destination when at least
        # one tool is direct-return at construction time. Include that dormant
        # edge, then restore normal looping before any turn can execute. A
        # cancellation request activates it for exactly one tool boundary.
        direct_edge_tool = tools[0] if tools else None
        if direct_edge_tool is not None:
            direct_edge_tool.return_direct = True
        try:
            return create_agent(
                model=model,
                tools=tools,
                system_prompt=self.config.system_prompt,
            )
        finally:
            if direct_edge_tool is not None:
                direct_edge_tool.return_direct = False

    def _mcp_tool_to_langchain(self, mcp_tool: dict[str, Any]) -> StructuredTool:
        name = mcp_tool["name"]
        description = mcp_tool.get("description", "")
        input_schema = mcp_tool.get(
            "inputSchema",
            {"type": "object", "properties": {}},
        )

        if name != "observe":
            upstream_tool = super()._mcp_tool_to_langchain(mcp_tool)

            def call_tool(**kwargs: Any) -> str:
                cancelled = self._cancelled_tool_result(wrapped_tool)
                if cancelled is not None:
                    return cancelled
                if upstream_tool.func is None:
                    raise RuntimeError(f"MCP tool '{name}' has no callable function")
                text = upstream_tool.func(**kwargs)
                text = self._append_update_text(
                    text,
                    self._take_live_update_text(),
                )
                return self._cancelled_tool_result(wrapped_tool, text) or text

            wrapped_tool = StructuredTool(
                name=name,
                description=description,
                func=call_tool,
                args_schema=input_schema,
            )
            self._eval_agent_tools.append(wrapped_tool)
            return wrapped_tool

        def call_observe(runtime: ToolRuntime, **kwargs: Any) -> str | Command[Any]:
            cancelled = self._cancelled_tool_result(wrapped_observe)
            if cancelled is not None:
                if runtime.tool_call_id is None:
                    raise RuntimeError("observe requires an active tool call")
                return Command(
                    update={
                        "messages": [
                            ToolMessage(
                                content=cancelled,
                                tool_call_id=runtime.tool_call_id,
                            )
                        ]
                    }
                )
            result = self._mcp_tool_call(name, kwargs)
            content = result.get("content", [])
            text = "\n".join(item.get("text", "") for item in content if item.get("type") == "text")
            images = [item for item in content if item.get("type") != "text"]
            text = self._append_update_text(
                text,
                self._take_live_update_text(),
            )
            if runtime.tool_call_id is None:
                raise RuntimeError("observe requires an active tool call")

            cancelled = self._cancelled_tool_result(wrapped_observe, text)
            if cancelled is not None:
                return Command(
                    update={
                        "messages": [
                            ToolMessage(
                                content=cancelled,
                                tool_call_id=runtime.tool_call_id,
                            )
                        ]
                    }
                )

            if not images:
                return text

            return Command(
                update={
                    "messages": [
                        ToolMessage(
                            content=text or "Current camera frame attached.",
                            tool_call_id=runtime.tool_call_id,
                        ),
                        HumanMessage(
                            content=[
                                {
                                    "type": "text",
                                    "text": "This is the current camera frame returned by observe.",
                                },
                                *images,
                            ]
                        ),
                    ]
                }
            )

        wrapped_observe = StructuredTool(
            name=name,
            description=description,
            func=call_observe,
            args_schema=input_schema,
        )
        self._eval_agent_tools.append(wrapped_observe)
        return wrapped_observe
