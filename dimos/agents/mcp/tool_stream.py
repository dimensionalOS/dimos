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

"""Server-initiated tool-stream notifications.

A skill uses `ToolStream` to push text updates out to any connected MCP client
(Claude Code, our own `McpClient`, curl, ...) while the skill's background work is
still running.

Transport: each `ToolStream.send` publishes a ready-made JSON-RPC
`notifications/message` frame on the shared `/tool_streams` LCM topic. Skill
workers and the `McpServer` process typically live in different workers, so we
lean on LCM's local-multicast bus to cross that boundary. `McpServer` subscribes
to the topic once, forwards each frame to every connected `GET /mcp` SSE client,
and drops frames when nobody is listening.

Each `ToolStream` instance owns its own `pLCMTransport`, created lazily on the
first `send` and torn down by `stop`. There is no module-level or process-level
state. The stream's lifetime is exactly the owning skill's lifetime.
"""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Any
import uuid

from dimos.agents.annotation import current_skill_context
from dimos.core.transport import pLCMTransport
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TOOL_STREAM_TOPIC = "/tool_streams"
NOTIFICATIONS_MESSAGE_METHOD = "notifications/message"
NOTIFICATIONS_PROGRESS_METHOD = "notifications/progress"
TOOL_STREAM_STOPPED_METHOD = "dimos/tool_stopped"

ToolStreamCallback = Callable[[dict[str, Any]], Any]


def make_notification(tool_name: str, text: str) -> dict[str, Any]:
    """Build an MCP `notifications/message` frame (log-style fallback)."""
    return {
        "jsonrpc": "2.0",
        "method": NOTIFICATIONS_MESSAGE_METHOD,
        "params": {
            "level": "info",
            "logger": tool_name,
            "data": text,
        },
    }


def make_progress_notification(
    progress_token: str | int,
    progress: int,
    message: str,
    tool_name: str | None = None,
    total: int | None = None,
) -> dict[str, Any]:
    """Build an MCP `notifications/progress` frame bound to `progress_token`."""
    params: dict[str, Any] = {"progressToken": progress_token, "progress": progress}
    if total is not None:
        params["total"] = total
    if message:
        params["message"] = message
    if tool_name is not None:
        params["_meta"] = {"tool_name": tool_name}
    return {"jsonrpc": "2.0", "method": NOTIFICATIONS_PROGRESS_METHOD, "params": params}


def make_stopped_notification(tool_name: str, token: str | None = None) -> dict[str, Any]:
    params: dict[str, Any] = {"tool_name": tool_name}
    if token is not None:
        params["token"] = token
    return {
        "jsonrpc": "2.0",
        "method": TOOL_STREAM_STOPPED_METHOD,
        "params": params,
    }


def subscribe(callback: ToolStreamCallback) -> Callable[[], None]:
    """Subscribe to the tool-stream LCM topic and return a cleanup callable."""
    transport: pLCMTransport[dict[str, Any]] = pLCMTransport(TOOL_STREAM_TOPIC)
    transport.start()
    unsubscribe = transport.subscribe(callback)

    def cleanup() -> None:
        try:
            unsubscribe()
        except Exception:
            logger.exception("tool-stream unsubscribe failed")
        try:
            transport.stop()
        except Exception:
            logger.exception("tool-stream transport stop failed")

    return cleanup


class ToolStream:
    """A streaming channel for pushing updates from a skill to the agent.

    Each `ToolStream` is tied to a single logical tool invocation.  It **must**
    be constructed inside a `@skill` call on the skill's own thread. That's
    where the per-call context (including the caller's `progressToken`) lives
    and can be captured. Constructing a `ToolStream` outside a `@skill` call
    raises `RuntimeError`.

    Once constructed, the instance is free-threaded: background threads
    spawned by the skill can call `send` and `stop` safely because
    the progress token is already captured on the instance.

    If the client that made the `tools/call` request did not supply a
    `progressToken`, the stream still constructs successfully (the per-call
    context is an empty dict, not `None`) and `send` falls back to emitting
    `notifications/message` log frames.

    Callers should use`Module.start_tool` / `tool_update` / `stop_tool` instead
    of constructing `ToolStream` directly.
    """

    def __init__(self, tool_name: str) -> None:
        self.tool_name: str = tool_name
        self.id: str = str(uuid.uuid4())
        self._closed: threading.Event = threading.Event()
        self._lock = threading.Lock()
        self._transport: pLCMTransport[dict[str, Any]] | None = None
        context = current_skill_context()
        if context is None:
            raise RuntimeError(
                f"ToolStream({tool_name!r}) must be constructed inside a @skill "
                f"call so the caller's progress token can be captured. Construct "
                f"it in the skill method's main thread, not in __init__ or a "
                f"detached background thread."
            )
        self._progress_token: str | int | None = context.get("progress_token")
        # Capability hold token (set by the MCP server for capability-using
        # skills); stamped on the stop frame so release is invocation-scoped.
        self._acquire_token: str | None = context.get("acquire_token")
        self._progress: int = 0

    def rebind_acquire_token(self) -> None:
        """Re-stamp this live stream with the current invocation's acquire token.

        Called when `start_tool` reopens an already-active stream -- a same-tool
        capability takeover. The newer invocation's hold token must be the one
        carried on the eventual stop frame, otherwise the registry would release
        a stale token and leak the capability. No-op if there is no skill context
        or the stream is already closed.
        """
        context = current_skill_context()
        if context is None:
            return
        with self._lock:
            if self._closed.is_set():
                return
            self._acquire_token = context.get("acquire_token")

    def send(self, message: str) -> None:
        with self._lock:
            if self._closed.is_set():
                logger.warning("send on closed ToolStream", stream_id=self.id)
                return
            if self._transport is None:
                self._transport = pLCMTransport(TOOL_STREAM_TOPIC)
                self._transport.start()
            self._progress += 1
            progress = self._progress
            transport = self._transport
            progress_token = self._progress_token
        if progress_token is not None:
            frame = make_progress_notification(
                progress_token, progress, message, tool_name=self.tool_name
            )
        else:
            frame = make_notification(self.tool_name, message)
        transport.publish(frame)

    def stop(self) -> None:
        with self._lock:
            if self._closed.is_set():
                return
            self._closed.set()
            transport = self._transport
            self._transport = None
        # Publish a final "stopped" frame so subscribers (e.g. McpServer's
        # capability registry) know the background skill released its hold.
        # If no `send()` ever happened we spin up a transport here so the
        # lifecycle signal isn't lost.
        if transport is None:
            transport = pLCMTransport(TOOL_STREAM_TOPIC)
            transport.start()
        try:
            transport.publish(make_stopped_notification(self.tool_name, self._acquire_token))
        except Exception:
            logger.exception("tool-stream stopped publish failed", stream_id=self.id)
        try:
            transport.stop()
        except Exception:
            logger.exception("tool-stream transport stop failed", stream_id=self.id)

    @property
    def is_closed(self) -> bool:
        return self._closed.is_set()
