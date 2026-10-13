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

"""How a channel meets its maxHz: sampling for state channels (the rate gate
drops frames inside the interval) and spacing for event channels (the paced
sender queues frames and sends one per interval, in order)."""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import Callable
from typing import Any

FrameMeta = dict[str, Any] | None
# (payload, meta, ts); ts is None for live frames (stamped at send) and the
# source arrival time for replays, so a stale replay is honest about its age.
Sender = Callable[[bytes, FrameMeta, float | None], None]


def passes_rate_gate(
    last_input: dict[str, float],
    ch: str,
    now: float,
    min_interval: float,
) -> bool:
    """Claim the current input when it is outside the channel's rate interval."""
    if now - last_input.get(ch, 0.0) < min_interval:
        return False
    last_input[ch] = now
    return True


# A producer sustaining more than maxHz for this many frames is pathological
# (the agent chats at a few messages a second); beyond it the oldest go.
_PACED_QUEUE_MAX = 256


class PacedSender:
    """Send cap by spacing, not sampling: frames queue on the loop and go out
    one per interval, in order, so an event channel (chat) never thins a
    burst. Wraps the channel's real sender."""

    def __init__(self, loop: asyncio.AbstractEventLoop, interval: float, send: Sender) -> None:
        self._loop = loop
        self._interval = interval
        self._send = send
        self._queue: deque[tuple[bytes, FrameMeta, float | None]] = deque()
        self._due = 0.0
        self._handle: asyncio.TimerHandle | None = None
        self.dropped = 0

    def __call__(self, payload: bytes, meta: FrameMeta, ts: float | None) -> None:
        self._queue.append((payload, meta, ts))
        if len(self._queue) > _PACED_QUEUE_MAX:
            self._queue.popleft()
            self.dropped += 1
        if self._handle is None:
            self._drain()

    def _drain(self) -> None:
        self._handle = None
        now = self._loop.time()
        if now < self._due:
            self._handle = self._loop.call_at(self._due, self._drain)
            return
        payload, meta, ts = self._queue.popleft()
        self._due = now + self._interval
        if self._queue:
            self._handle = self._loop.call_at(self._due, self._drain)
        try:
            self._send(payload, meta, ts)
        except Exception:
            return  # session mid-teardown, same as RelayBridgeModule._offer

    def close(self) -> None:
        if self._handle is not None:
            self._handle.cancel()
            self._handle = None
        self._queue.clear()
