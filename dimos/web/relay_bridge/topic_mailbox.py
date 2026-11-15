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

"""Bound Zenoh callback work before bytes are copied or profiles transformed."""

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import cached_property
import threading

import zenoh

from dimos.web.relay_bridge.manifest import Delivery
from dimos.web.relay_bridge.topic_protocol import (
    MAX_TOPIC_KEY_BYTES,
    MAX_TOPIC_KEYS,
    MAX_TOPIC_PAYLOAD_BYTES,
)
from dimos.web.relay_bridge.topic_transport import ORDERED_MAX_BYTES, ORDERED_MAX_FRAMES


@dataclass
class TopicIngressBudget:
    """One lock owns all callback queues and their aggregate memory account."""

    limit: int = 64 * 1024 * 1024
    used: int = 0
    peak: int = 0
    lock: threading.RLock = field(default_factory=threading.RLock)


@dataclass(frozen=True)
class TopicSample:
    key: str
    encoding: str
    payload: bytes | zenoh.ZBytes
    ingress_ts: float
    source_ts: float | None = None
    kind: str = "put"

    @cached_property
    def size(self) -> int:
        # Include bounded metadata and object overhead, not just payloads.
        return len(self.payload) + len(self.key.encode()) + len(self.encoding.encode()) + 256


class TopicMailbox:
    """Reliable FIFO or one newest raw sample per key, sharing aggregate caps.

    The callback only retains ZBytes references. One pending wake-up covers
    any burst; rate selection happens before to_bytes/decoding/encoding.
    Consumers must release taken samples, including when a transform fails.
    """

    def __init__(
        self,
        delivery: Delivery,
        max_hz: float,
        budget: TopicIngressBudget,
        wake: Callable[[], None],
    ) -> None:
        self._delivery = delivery
        self._period = 1 / max_hz
        self._budget = budget
        self._wake = wake
        self._fifo: deque[TopicSample] = deque()
        self._latest: dict[str, TopicSample] = {}
        self._next_due: dict[str, float] = {}
        self._keys: set[str] = set()
        self._bytes = 0
        self._count = 0
        self._notified = False
        self._closed = False
        self._error: str | None = None
        self._dropped = 0
        self._last_arrival: float | None = None
        self._previous_gap_short = False
        self._burst_until = 0.0

    @property
    def error(self) -> str | None:
        with self._budget.lock:
            return self._error

    def set_rate(self, max_hz: float) -> None:
        """Keep the last selection time when changing a shared route's rate."""
        with self._budget.lock:
            period = 1 / max_hz
            self._next_due = {
                key: due + period - self._period for key, due in self._next_due.items()
            }
            self._period = period
            self._notify()

    @property
    def dropped(self) -> int:
        with self._budget.lock:
            return self._dropped

    @property
    def coalesce(self) -> bool:
        """Batch dense reliable arrivals; leave isolated telemetry prompt.

        Three successive arrivals with gaps below 5 ms mark the next 100 ms of
        arrivals. Ingress timestamps are captured locally by the callback,
        never supplied by a remote publisher. The route still bounds the
        actual hold to 90 ms, independently of wall-clock adjustments.
        """
        with self._budget.lock:
            return self._last_arrival is not None and self._last_arrival < self._burst_until

    def _notify(self) -> None:
        # Called under the lock; wake only schedules an event-loop callback.
        if not self._notified:
            self._notified = True
            self._wake()

    def _release(self, sample: TopicSample) -> None:
        self._bytes -= sample.size
        self._count -= 1
        self._budget.used -= sample.size

    def offer(self, sample: TopicSample) -> None:
        with self._budget.lock:
            if self._closed or self._error is not None:
                return
            if (
                len(sample.payload) > MAX_TOPIC_PAYLOAD_BYTES
                or len(sample.key.encode()) > MAX_TOPIC_KEY_BYTES
                or len(sample.encoding) > 256
            ):
                self._error = "topic_sample_too_large"
            elif sample.key not in self._keys and len(self._keys) >= MAX_TOPIC_KEYS:
                self._error = "topic_key_limit"
            else:
                if self._delivery == "latest":
                    old = self._latest.pop(sample.key, None)
                    if old is not None:
                        self._release(old)
                        self._dropped += 1
                if (
                    self._bytes + sample.size > ORDERED_MAX_BYTES
                    or self._budget.used + sample.size > self._budget.limit
                    or self._count >= ORDERED_MAX_FRAMES
                ):
                    self._error = "topic_overflow"
                else:
                    self._keys.add(sample.key)
                    self._bytes += sample.size
                    self._budget.used += sample.size
                    self._budget.peak = max(self._budget.peak, self._budget.used)
                    self._count += 1
                    if self._delivery == "latest":
                        self._latest[sample.key] = sample
                    else:
                        if self._last_arrival is not None:
                            interval = sample.ingress_ts - self._last_arrival
                            short_gap = 0 <= interval < 0.005
                            if interval < 0:
                                self._burst_until = 0.0
                            elif short_gap and self._previous_gap_short:
                                self._burst_until = sample.ingress_ts + 0.1
                            self._previous_gap_short = short_gap
                        self._last_arrival = sample.ingress_ts
                        self._fifo.append(sample)
            self._notify()

    def take(self, now: float, limit: int = 32) -> tuple[list[TopicSample], float | None]:
        """Eligible raw samples and delay until another queued sample is due."""
        with self._budget.lock:
            self._notified = False
            if self._closed or self._error is not None:
                return [], None
            batch: list[TopicSample] = []
            if self._delivery == "reliable":
                batch = [self._fifo.popleft() for _ in range(min(limit, len(self._fifo)))]
                return batch, 0.0 if self._fifo else None
            for key, sample in list(self._latest.items()):
                due = self._next_due.get(key)
                if (due is None or due <= now) and len(batch) < limit:
                    batch.append(sample)
                    del self._latest[key]
                    # Anchored to the schedule, not to this (late) take: a wake
                    # that runs a millisecond late must not push every later
                    # due time, or a same-rate source loses one frame per
                    # period of accumulated lateness. After a gap the schedule
                    # restarts from now.
                    self._next_due[key] = max(
                        now - self._period, (now if due is None else due) + self._period
                    )
            delay = min(
                (max(0.0, self._next_due.get(key, 0) - now) for key in self._latest), default=None
            )
            return batch, delay

    def refresh(self, key: str) -> TopicSample | None:
        """The sample of ``key`` that arrived since its take, past the rate gate.

        It replaces the taken one, not encoded yet, rather than adding a
        selection: the key's schedule stays where the take left it.
        """
        with self._budget.lock:
            if self._closed or self._error is not None:
                return None
            return self._latest.pop(key, None)

    def release(self, sample: TopicSample) -> None:
        with self._budget.lock:
            if not self._closed:
                self._release(sample)

    def close(self) -> None:
        with self._budget.lock:
            if self._closed:
                return
            self._closed = True
            self._budget.used -= self._bytes
            self._bytes = 0
            self._count = 0
            self._fifo.clear()
            self._latest.clear()
            self._keys.clear()
            self._next_due.clear()
