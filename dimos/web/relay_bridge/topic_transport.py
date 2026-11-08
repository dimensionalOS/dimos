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

"""Bounded ordered provider-to-relay delivery, independent of robot modules."""

import asyncio
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import time

from dimos.web.relay_bridge._wt_session import SessionProtocol
from dimos.web.relay_bridge.protocol import FrameHeader, encode_data_frame
from dimos.web.relay_bridge.topic_protocol import (
    MAX_TOPIC_KEYS,
    MAX_TOPIC_PAYLOAD_BYTES,
    TOPIC_ROUTE_PREFIX,
)

ORDERED_MAX_BYTES = 16 * 1024 * 1024
ORDERED_MAX_FRAMES = 512


@dataclass
class TopicByteBudget:
    """Event-loop-owned budget shared across a provider's routes."""

    limit: int = 64 * 1024 * 1024
    used: int = 0
    peak: int = 0


class OrderedTopicWriter:
    """One persistent ordered stream with a bounded pipeline, not stop-and-wait.

    ``offer`` is event-loop-only and never schedules work per sample. Queue and
    QUIC-buffered bytes share the same budget until ACKed, including retransmits.
    Overflow is explicit; a reliable event must never be silently overwritten.
    The owner closes the writer on unsubscribe, source loss, or any failure.
    """

    def __init__(
        self,
        session: SessionProtocol,
        route: str,
        budget: TopicByteBudget,
        on_error: Callable[[str], None],
        *,
        max_bytes: int = ORDERED_MAX_BYTES,
        max_frames: int = ORDERED_MAX_FRAMES,
    ) -> None:
        if not route.startswith(TOPIC_ROUTE_PREFIX):
            raise ValueError("ordered streams require an authorized topic route")
        self._session = session
        self.route = route
        self._budget = budget
        self._on_error = on_error
        self._max_bytes = max_bytes
        self._max_frames = max_frames
        self._queue: deque[bytes] = deque()
        self._receipts: deque[tuple[int, int]] = deque()
        self._bytes = 0
        self._frames = 0
        self._stream: int | None = None
        self._closed = False
        self._changed = asyncio.Event()
        self._task: asyncio.Task[None] | None = None

    def start(self) -> None:
        if self._closed:
            raise ConnectionError("ordered topic writer closed")
        if self._task is None:
            self._session.add_delivery_listener(self._changed.set)
            self._task = asyncio.create_task(self._pump())

    @property
    def buffered_frames(self) -> int:
        self._release_acked()
        return self._frames

    def _release_acked(self) -> None:
        if self._stream is None or self._closed:
            return
        acked, _ = self._session.ordered_stream_offsets(self._stream)
        while self._receipts and self._receipts[0][0] <= acked:
            _, size = self._receipts.popleft()
            self._bytes -= size
            self._frames -= 1
            self._budget.used -= size

    def offer(self, header: FrameHeader, payload: bytes) -> None:
        if self._closed or self._task is None:
            raise ConnectionError("ordered topic writer closed")
        if header.ch != self.route or header.delivery != "reliable":
            raise ValueError("frame does not belong to this ordered route")
        if len(payload) > MAX_TOPIC_PAYLOAD_BYTES:
            raise ValueError("topic payload exceeds byte bound")
        frame = encode_data_frame(header, payload)
        size = len(frame)
        if (
            self._frames >= self._max_frames
            or self._bytes + size > self._max_bytes
            or self._budget.used + size > self._budget.limit
        ):
            # The pump releases credit on delivery notifications. Recheck here
            # only at a bound, in case its wake-up has not run yet. Reading the
            # same QUIC offset for every event in a burst adds no new credit.
            self._release_acked()
            if (
                self._frames >= self._max_frames
                or self._bytes + size > self._max_bytes
                or self._budget.used + size > self._budget.limit
            ):
                raise BufferError("ordered topic queue and in-flight budget exhausted")
        self._bytes += size
        self._frames += 1
        self._budget.used += size
        self._budget.peak = max(self._budget.peak, self._budget.used)
        self._queue.append(frame)
        self._changed.set()

    async def _pump(self) -> None:
        try:
            while not self._closed:
                await self._changed.wait()
                self._changed.clear()
                if self._session.closed.is_set():
                    raise ConnectionError("topic provider session closed")
                self._release_acked()
                if self._queue and self._stream is None:
                    self._stream = self._session.open_ordered_stream()
                wrote = bool(self._queue)
                while self._queue and not self._closed:
                    assert self._stream is not None
                    frame = self._queue.popleft()
                    offset = self._session.write_ordered_stream(self._stream, frame)
                    self._receipts.append((offset, len(frame)))
                # aioquic already transmits after processing an incoming ACK.
                # A credit-only wake must not repeat the whole packet builder.
                if wrote:
                    self._session.transmit()
        except (ConnectionError, RuntimeError):
            if not self._closed:
                self.stop()
                self._on_error("topic_stream_failed")

    def stop(self) -> None:
        """Release buffers, stream and delivery listener exactly once."""
        if self._closed:
            return
        self._closed = True
        self._session.remove_delivery_listener(self._changed.set)
        if self._task is not None:
            self._task.cancel()
        if self._stream is not None:
            self._session.reset_ordered_stream(self._stream)
        self._budget.used -= self._bytes
        self._bytes = 0
        self._frames = 0
        self._queue.clear()
        self._receipts.clear()

    async def aclose(self) -> None:
        self.stop()
        if self._task is not None:
            (result,) = await asyncio.gather(self._task, return_exceptions=True)
            if isinstance(result, Exception):
                raise result


@dataclass
class _LatestSend:
    key: str
    size: int
    at: float


class LatestTopicWriter:
    """Bounded latest slots per concrete key, with several frames in flight.

    Eight frames per key allow a 30 Hz view to pipeline across a 200 ms RTT.
    ACKed or expired streams release their bytes; pending replacements cost
    no extra tasks. All routes share the provider's aggregate byte budget.
    """

    def __init__(
        self,
        session: SessionProtocol,
        route: str,
        budget: TopicByteBudget,
        on_error: Callable[[str], None],
        *,
        stale_after: float = 0.5,
    ) -> None:
        self._session = session
        self.route = route
        self._budget = budget
        self._on_error = on_error
        self._stale_after = stale_after
        self._pending: dict[str, bytes] = {}
        self._inflight: dict[int, _LatestSend] = {}
        self._keys: set[str] = set()
        self._bytes = 0
        self._closed = False
        self._changed = asyncio.Event()
        self._task: asyncio.Task[None] | None = None
        self._timer: asyncio.TimerHandle | None = None
        self.dropped = 0

    def start(self) -> None:
        if self._closed:
            raise ConnectionError("latest topic writer closed")
        if self._task is None:
            self._session.add_delivery_listener(self._changed.set)
            self._task = asyncio.create_task(self._pump())

    def _release(self, size: int) -> None:
        self._bytes -= size
        self._budget.used -= size

    def _reap(self, now: float) -> None:
        for stream, pending in list(self._inflight.items()):
            if not self._session.stream_in_flight(stream):
                del self._inflight[stream]
                self._release(pending.size)
            elif now - pending.at >= self._stale_after:
                self._session.reset_ordered_stream(stream)
                del self._inflight[stream]
                self._release(pending.size)

    def offer(self, header: FrameHeader, payload: bytes) -> None:
        if self._closed or self._task is None:
            raise ConnectionError("latest topic writer closed")
        if header.ch != self.route or header.delivery != "latest":
            raise ValueError("frame does not belong to this latest route")
        key = (header.meta or {}).get("key")
        if not isinstance(key, str):
            raise ValueError("latest topic frame needs a concrete key")
        if key not in self._keys and len(self._keys) >= MAX_TOPIC_KEYS:
            raise BufferError("latest topic key bound exhausted")
        if len(payload) > MAX_TOPIC_PAYLOAD_BYTES:
            raise ValueError("topic payload exceeds byte bound")
        self._reap(time.monotonic())
        previous = self._pending.pop(key, None)
        if previous is not None:
            self._release(len(previous))
            self.dropped += 1
        frame = encode_data_frame(header, payload)
        size = len(frame)
        if (
            self._bytes + size > ORDERED_MAX_BYTES
            or self._budget.used + size > self._budget.limit
            or len(self._pending) + len(self._inflight) >= ORDERED_MAX_FRAMES
        ):
            raise BufferError("latest topic queue and in-flight budget exhausted")
        self._keys.add(key)
        self._pending[key] = frame
        self._bytes += size
        self._budget.used += size
        self._budget.peak = max(self._budget.peak, self._budget.used)
        self._changed.set()

    async def _pump(self) -> None:
        try:
            while not self._closed:
                await self._changed.wait()
                self._changed.clear()
                if self._timer is not None:
                    self._timer.cancel()
                    self._timer = None
                if self._session.closed.is_set():
                    raise ConnectionError("topic provider session closed")
                now = time.monotonic()
                self._reap(now)
                for key, frame in list(self._pending.items()):
                    if sum(p.key == key for p in self._inflight.values()) >= 8:
                        continue
                    stream = self._session.send_frame_bytes(frame)
                    self._inflight[stream] = _LatestSend(key, len(frame), now)
                    del self._pending[key]
                if self._inflight:
                    deadline = min(p.at for p in self._inflight.values()) + self._stale_after
                    self._timer = asyncio.get_running_loop().call_later(
                        max(0.0, deadline - time.monotonic()),
                        self._changed.set,
                    )
        except (ConnectionError, RuntimeError):
            if not self._closed:
                self.stop()
                self._on_error("topic_stream_failed")

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._session.remove_delivery_listener(self._changed.set)
        if self._timer is not None:
            self._timer.cancel()
        if self._task is not None:
            self._task.cancel()
        for stream in self._inflight:
            self._session.reset_ordered_stream(stream)
        self._budget.used -= self._bytes
        self._bytes = 0
        self._pending.clear()
        self._inflight.clear()
        self._keys.clear()

    async def aclose(self) -> None:
        self.stop()
        if self._task is not None:
            (result,) = await asyncio.gather(self._task, return_exceptions=True)
            if isinstance(result, Exception):
                raise result
