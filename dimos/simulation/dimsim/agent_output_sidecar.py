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

"""Read-only NDJSON bridge from the DimOS ``/agent`` stream to DimSim evals."""

from __future__ import annotations

from collections.abc import Callable
import json
import signal
import sys
from threading import Event, RLock
import time
from typing import Any, Protocol, TextIO

from langchain_core.messages import AIMessage
from langchain_core.messages.base import BaseMessage

from dimos.core.transport_factory import apply_transport_arg, make_transport


class AgentTransport(Protocol):
    """Transport behavior used by the sidecar."""

    def start(self) -> None: ...

    def subscribe(self, callback: Callable[[Any], Any]) -> Callable[[], None]: ...

    def stop(self) -> None: ...


def _text_content(content: Any) -> str:
    if isinstance(content, str):
        return content
    if not isinstance(content, list):
        return ""

    parts: list[str] = []
    for block in content:
        if isinstance(block, str):
            parts.append(block)
            continue
        if not isinstance(block, dict):
            continue
        block_type = block.get("type")
        text = block.get("text")
        if block_type in {"text", "output_text"} and isinstance(text, str):
            parts.append(text)
    return "".join(parts)


def serialize_agent_message(
    message: BaseMessage,
    *,
    timestamp: float | None = None,
) -> dict[str, Any] | None:
    """Return the eval-safe representation of an AI message."""
    if not isinstance(message, AIMessage):
        return None
    return {
        "type": "agent_output",
        "text": _text_content(message.content),
        "hasToolCalls": bool(message.tool_calls),
        "timestampMs": round((time.time() if timestamp is None else timestamp) * 1000),
    }


def serialize_agent_idle(
    idle: Any,
    *,
    timestamp: float | None = None,
) -> dict[str, Any] | None:
    """Return a typed idle-state event for the eval lifecycle."""
    if not isinstance(idle, bool):
        return None
    return {
        "type": "agent_idle",
        "idle": idle,
        "timestampMs": round((time.time() if timestamp is None else timestamp) * 1000),
    }


def run_sidecar(
    transport: AgentTransport,
    output: TextIO,
    stop_event: Event,
    idle_transport: AgentTransport | None = None,
) -> None:
    """Stream AI messages and optional idle state, owning all cleanup."""
    lock = RLock()
    ready = False
    pending: list[dict[str, Any]] = []

    def emit(payload: dict[str, Any]) -> None:
        try:
            output.write(json.dumps(payload, separators=(",", ":")) + "\n")
            output.flush()
        except BrokenPipeError:
            stop_event.set()

    def on_message(message: Any) -> None:
        event = serialize_agent_message(message)
        if event is None:
            return
        with lock:
            if ready:
                emit(event)
            else:
                pending.append(event)

    def on_idle(idle: Any) -> None:
        event = serialize_agent_idle(idle)
        if event is None:
            return
        with lock:
            if ready:
                emit(event)
            else:
                pending.append(event)

    transport.start()
    unsubscribe: Callable[[], None] | None = None
    unsubscribe_idle: Callable[[], None] | None = None
    idle_started = False
    try:
        unsubscribe = transport.subscribe(on_message)
        if idle_transport is not None:
            idle_transport.start()
            idle_started = True
            unsubscribe_idle = idle_transport.subscribe(on_idle)
        with lock:
            emit({"type": "ready"})
            ready = True
            for event in pending:
                emit(event)
            pending.clear()
        stop_event.wait()
    finally:
        if unsubscribe_idle is not None:
            unsubscribe_idle()
        if unsubscribe is not None:
            unsubscribe()
        if idle_started and idle_transport is not None:
            idle_transport.stop()
        transport.stop()


def main() -> None:
    apply_transport_arg(sys.argv)
    stop_event = Event()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    run_sidecar(
        make_transport("/agent"),
        sys.stdout,
        stop_event,
        make_transport("/agent_idle"),
    )


if __name__ == "__main__":
    main()
