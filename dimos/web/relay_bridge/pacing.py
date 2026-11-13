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

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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
    """Claim the current input when it is outside the channel's rate interval:
    a strict minimum spacing from the last accepted input (log throttling).
    Not for sampling a stream: see passes_sampling_gate."""
    if now - last_input.get(ch, 0.0) < min_interval:
        return False
    last_input[ch] = now
    return True


def passes_sampling_gate(
    next_due: dict[str, float],
    ch: str,
    now: float,
    interval: float,
) -> bool:
    """Sample a stream at one frame per `interval` on average, tolerant of
    arrival jitter.

    The schedule is anchored on the channel's own due times, not on the
    accepted arrivals: a frame up to half an interval early is accepted and
    the next due time advances from the schedule, so a 30 Hz camera through a
    30 Hz gate passes every frame (a last-arrival anchor rejects every frame
    that lands a few ms early and re-anchors on the late ones, which thins
    30 fps to 20). A frame arriving late advances the schedule from its own
    arrival, so the average never exceeds the rate; the only burst is two
    frames half an interval apart right after a late one.
    """
    due = next_due.get(ch, 0.0)
    if now < due - 0.5 * interval:
        return False
    next_due[ch] = max(due, now) + interval
    return True


# A producer sustaining more than maxHz for this many frames is pathological
# (the agent chats at a few messages a second); beyond it the oldest go,
# logged at most once per _PACED_DROP_LOG_S per sender.
_PACED_QUEUE_MAX = 256
_PACED_DROP_LOG_S = 5.0


class PacedSender:
    """Send cap by spacing, not sampling: frames queue on the loop and go out
    one per interval, in order, so an event channel (chat) never thins a
    burst. Wraps the channel's real sender."""

    def __init__(
        self, loop: asyncio.AbstractEventLoop, interval: float, send: Sender, name: str = ""
    ) -> None:
        self._loop = loop
        self._interval = interval
        self._send = send
        self._name = name
        self._queue: deque[tuple[bytes, FrameMeta, float | None]] = deque()
        self._due = 0.0
        self._handle: asyncio.TimerHandle | None = None
        self.dropped = 0
        self._drop_logged: dict[str, float] = {}

    def __call__(self, payload: bytes, meta: FrameMeta, ts: float | None) -> None:
        self._queue.append((payload, meta, ts))
        if len(self._queue) > _PACED_QUEUE_MAX:
            self._queue.popleft()
            self.dropped += 1
            # A chat flood losing its oldest lines must not be silent.
            if passes_rate_gate(self._drop_logged, "", self._loop.time(), _PACED_DROP_LOG_S):
                logger.warning(
                    f"paced channel {self._name or '?'}: {_PACED_QUEUE_MAX} frames queued, "
                    f"dropping the oldest ({self.dropped} dropped so far)"
                )
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
