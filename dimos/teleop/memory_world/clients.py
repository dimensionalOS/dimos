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

"""The memory world's web clients: one queue per websocket, what a viewer's messages
mean, and static files the browser must revalidate."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any

from fastapi import WebSocket
from fastapi.staticfiles import StaticFiles

from dimos.teleop.memory_world.messages import encode_text
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _is_finite_number(value: Any) -> bool:
    """True for a real number the viewer can be at, False for anything else.

    Two different exceptions have escaped this guard and killed the websocket loop:
    `np.isfinite` raises TypeError on a python int wider than int64, and `math.isfinite`
    raises OverflowError converting one to a float. Swapping the first for the second
    fixed the first input and not the second. A guard whose whole job is to reject bad
    input must not raise on any of it.
    """
    if isinstance(value, bool) or not isinstance(value, int | float):
        return False
    try:
        return math.isfinite(float(value))
    except (OverflowError, ValueError, TypeError):
        return False


class RevalidatedStaticFiles(StaticFiles):
    """Static files the browser must revalidate (ETag) on every load.

    Without this a phone kept a stale scene.js beside a fresh main.js for
    hours and every new control on the page was a TypeError.
    """

    async def get_response(self, path: str, scope: Any) -> Any:
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-cache"
        return response


@dataclass(eq=False)
class ClientConn:
    """One connected memory-world client."""

    ws: WebSocket
    loop: asyncio.AbstractEventLoop
    queue: asyncio.Queue[bytes | str] = field(default_factory=lambda: asyncio.Queue(maxsize=512))

    def send_threadsafe(self, msg: bytes | str) -> None:
        try:
            self.loop.call_soon_threadsafe(self._enqueue, msg)
        except RuntimeError:
            pass

    def _enqueue(self, msg: bytes | str) -> None:
        try:
            self.queue.put_nowait(msg)
        except asyncio.QueueFull:  # a stalled client: the world already queued outranks this
            logger.warning("client send queue full, dropping a message")


class ClientMessages:
    """The router for what a viewer sends up the websocket.

    Mixed into MemoryWorldModule, and here rather than in `module.py` because that file
    sits on the repo's 75 KB per-file ceiling. It belongs beside `ClientConn` anyway:
    both halves of "what a client is" are now in one file.

    Needs, from the module: ``_clients_lock``, ``_viewer_position`` and ``_ask_in_chat``.
    """

    _clients_lock: Any
    _viewer_position: tuple[float, float, float] | None

    if TYPE_CHECKING:

        def _ask_in_chat(self, text: str) -> bool: ...

    def _on_client_message(self, conn: ClientConn, msg: dict[str, Any]) -> None:
        kind = msg.get("type")
        if kind == "ping":
            conn.send_threadsafe(encode_text("pong"))
        elif kind == "diag":
            logger.info(
                "[client/diag] %s %s",
                msg.get("event", "?"),
                {k: v for k, v in msg.items() if k not in ("type", "event")},
            )
        elif kind == "viewer_pose":
            position = msg.get("position")
            if (
                isinstance(position, list)
                and len(position) == 3
                and all(_is_finite_number(value) for value in position)
            ):
                with self._clients_lock:
                    self._viewer_position = (
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    )
        elif kind in (
            "locomote",
            "yaw",
            "teleport_aim",
            "teleport_commit",
            "teleport_cancel",
            "scale_delta",
            "reset_view",
            "toggle_images",
            "toggle_cloud",
            "voice_start",
            "voice_stop",
        ):
            # Gestures the client handles itself, echoed here only as telemetry.
            # Debug-level so they don't spam the console (scale_delta fires every frame).
            #
            # Every kind `main.js`'s `dispatchGesture` forwards belongs in this tuple, or
            # the warning below fires on ordinary traffic: push-to-talk sends
            # `voice_start`/`voice_stop` on every mic press, and each one was logged as an
            # unknown message -- which teaches an operator to ignore the warning that
            # exists to catch a genuinely unrecognised one. The test beside this reads the
            # list out of `main.js` so the two cannot drift apart again.
            logger.debug("[client] %s", kind)
        elif kind == "ask":
            # Fire and forget: the answer comes back on the `agent` stream like every
            # other row of the conversation, so there is nothing to correlate here and
            # nothing to wait for. `/ask` is the blocking HTTP path, for scripts.
            text = msg.get("text")
            if isinstance(text, str) and text.strip():
                if not self._ask_in_chat(text.strip()):
                    conn.send_threadsafe(
                        encode_text("error", message="no agent is connected to ask")
                    )
        else:
            logger.warning("[client] unknown msg kind=%r full=%r", kind, msg)
