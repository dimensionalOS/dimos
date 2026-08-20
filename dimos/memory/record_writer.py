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

"""Micro-batched transaction coordinator for Recorder writes."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import queue
import threading
import time
from typing import Any, cast

from dimos.memory.backend import Backend, PreparedAppend
from dimos.memory.recorder_queue import RecorderFailedError


@dataclass(frozen=True)
class RecordWriterStatus:
    submitted: int
    committed: int
    transactions: int
    pending: int
    submitted_payload_bytes: int
    committed_payload_bytes: int
    oldest_age_s: float
    max_pending: int
    max_oldest_age_s: float
    failed: str | None
    mean_rows_per_transaction: float
    max_rows_per_transaction: int
    commit_p99_ms: float
    receive_to_commit_p50_ms: float
    receive_to_commit_p95_ms: float
    receive_to_commit_p99_ms: float
    receive_to_commit_max_ms: float
    streams: dict[str, RecordWriterStreamStatus]


@dataclass(frozen=True)
class RecordWriterStreamStatus:
    submitted: int
    committed: int
    submitted_payload_bytes: int
    committed_payload_bytes: int
    receive_to_commit_p99_ms: float
    receive_to_commit_max_ms: float


@dataclass(frozen=True)
class _Write:
    backend: Backend[Any]
    prepared: PreparedAppend[Any]
    accepted_monotonic: float
    stream_name: str
    payload_bytes: int


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, round(percentile * len(ordered)) - 1))
    return ordered[index]


class RecordWriter:
    """Commit prepared observations in bounded cross-stream micro-batches."""

    def __init__(self, *, max_rows: int = 64, max_delay_s: float = 0.010) -> None:
        self._max_rows = max_rows
        self._max_delay_s = max_delay_s
        self._queue: queue.Queue[_Write | None] = queue.Queue()
        self._lock = threading.Lock()
        self._submitted = 0
        self._committed = 0
        self._transactions = 0
        self._submitted_payload_bytes = 0
        self._committed_payload_bytes = 0
        self._max_pending = 0
        self._max_oldest_age_s = 0.0
        self._oldest_monotonic: float | None = None
        self._failure: BaseException | None = None
        self._transaction_rows: deque[int] = deque(maxlen=100_000)
        self._commit_durations_s: deque[float] = deque(maxlen=100_000)
        self._receive_to_commit_s: deque[float] = deque(maxlen=100_000)
        self._stream_submitted: dict[str, int] = defaultdict(int)
        self._stream_committed: dict[str, int] = defaultdict(int)
        self._stream_submitted_bytes: dict[str, int] = defaultdict(int)
        self._stream_committed_bytes: dict[str, int] = defaultdict(int)
        self._stream_latencies_s: dict[str, deque[float]] = defaultdict(
            lambda: deque(maxlen=100_000)
        )
        self._closed = False
        self._thread = threading.Thread(
            target=self._run, name="recorder-sqlite-writer", daemon=True
        )
        self._thread.start()

    def submit(
        self,
        backend: Backend[Any],
        prepared: PreparedAppend[Any],
        *,
        accepted_monotonic: float | None = None,
        stream_name: str | None = None,
    ) -> None:
        accepted = time.monotonic() if accepted_monotonic is None else accepted_monotonic
        name = backend.name if stream_name is None else stream_name
        payload_bytes = len(prepared.encoded) if prepared.encoded is not None else 0
        with self._lock:
            self._raise_if_failed_locked()
            if self._closed:
                raise RecorderFailedError("Recorder writer is closed")
            self._submitted += 1
            self._submitted_payload_bytes += payload_bytes
            self._stream_submitted[name] += 1
            self._stream_submitted_bytes[name] += payload_bytes
            self._max_pending = max(self._max_pending, self._submitted - self._committed)
            if self._oldest_monotonic is None:
                self._oldest_monotonic = accepted
        self._queue.put(_Write(backend, prepared, accepted, name, payload_bytes))

    def flush(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while self._queue.unfinished_tasks:
            with self._lock:
                self._raise_if_failed_locked()
            if time.monotonic() >= deadline:
                raise RecorderFailedError(f"Recorder writer did not drain within {timeout_s:.3f}s")
            time.sleep(0.005)
        with self._lock:
            self._raise_if_failed_locked()

    def close(self, timeout_s: float) -> None:
        with self._lock:
            if not self._closed:
                self._closed = True
                self._queue.put(None)
        self._thread.join(timeout=timeout_s)
        if self._thread.is_alive():
            raise RecorderFailedError("Recorder writer worker did not stop")
        with self._lock:
            self._raise_if_failed_locked()

    def status(self) -> RecordWriterStatus:
        now = time.monotonic()
        with self._lock:
            rows = list(self._transaction_rows)
            durations = list(self._commit_durations_s)
            latencies = list(self._receive_to_commit_s)
            oldest_age = 0.0 if self._oldest_monotonic is None else now - self._oldest_monotonic
            self._max_oldest_age_s = max(self._max_oldest_age_s, oldest_age)
            streams = {
                name: RecordWriterStreamStatus(
                    submitted=self._stream_submitted[name],
                    committed=self._stream_committed[name],
                    submitted_payload_bytes=self._stream_submitted_bytes[name],
                    committed_payload_bytes=self._stream_committed_bytes[name],
                    receive_to_commit_p99_ms=_percentile(list(self._stream_latencies_s[name]), 0.99)
                    * 1e3,
                    receive_to_commit_max_ms=max(self._stream_latencies_s[name], default=0.0) * 1e3,
                )
                for name in self._stream_submitted
            }
            return RecordWriterStatus(
                submitted=self._submitted,
                committed=self._committed,
                transactions=self._transactions,
                pending=self._submitted - self._committed,
                submitted_payload_bytes=self._submitted_payload_bytes,
                committed_payload_bytes=self._committed_payload_bytes,
                oldest_age_s=oldest_age,
                max_pending=self._max_pending,
                max_oldest_age_s=self._max_oldest_age_s,
                failed=str(self._failure) if self._failure is not None else None,
                mean_rows_per_transaction=sum(rows) / len(rows) if rows else 0.0,
                max_rows_per_transaction=max(rows, default=0),
                commit_p99_ms=_percentile(durations, 0.99) * 1e3,
                receive_to_commit_p50_ms=_percentile(latencies, 0.50) * 1e3,
                receive_to_commit_p95_ms=_percentile(latencies, 0.95) * 1e3,
                receive_to_commit_p99_ms=_percentile(latencies, 0.99) * 1e3,
                receive_to_commit_max_ms=max(latencies, default=0.0) * 1e3,
                streams=streams,
            )

    def live_status(self) -> dict[str, int | float | str | None]:
        """Return O(1) counters suitable for frequent benchmark sampling."""
        now = time.monotonic()
        with self._lock:
            oldest_age = 0.0 if self._oldest_monotonic is None else now - self._oldest_monotonic
            self._max_oldest_age_s = max(self._max_oldest_age_s, oldest_age)
            return {
                "submitted": self._submitted,
                "committed": self._committed,
                "pending": self._submitted - self._committed,
                "committed_payload_bytes": self._committed_payload_bytes,
                "oldest_age_s": oldest_age,
                "max_pending": self._max_pending,
                "max_oldest_age_s": self._max_oldest_age_s,
                "failed": str(self._failure) if self._failure is not None else None,
            }

    def _run(self) -> None:
        stop = False
        while not stop:
            first = self._queue.get()
            if first is None:
                self._queue.task_done()
                return
            batch = [first]
            deadline = time.monotonic() + self._max_delay_s
            while len(batch) < self._max_rows:
                timeout = deadline - time.monotonic()
                if timeout <= 0:
                    break
                try:
                    item = self._queue.get(timeout=timeout)
                except queue.Empty:
                    break
                if item is None:
                    self._queue.task_done()
                    stop = True
                    break
                batch.append(item)
            try:
                self._commit_batch(batch)
            except BaseException as error:
                with self._lock:
                    if self._failure is None:
                        self._failure = error
            finally:
                for _ in batch:
                    self._queue.task_done()

    def _commit_batch(self, batch: list[_Write]) -> None:
        grouped: dict[int, list[_Write]] = defaultdict(list)
        backends: dict[int, Backend[Any]] = {}
        for item in batch:
            key = id(item.backend)
            grouped[key].append(item)
            backends[key] = item.backend

        committed: list[_Write] = []
        for key, items in grouped.items():
            backend = backends[key]
            started = time.perf_counter()
            try:
                for item in items:
                    backend.persist_prepared(item.prepared)
                backend.commit()
            except BaseException:
                backend.rollback()
                raise
            for item in items:
                backend.notify(item.prepared.observation)
            committed.extend(items)
            committed_at = time.monotonic()
            with self._lock:
                self._transactions += 1
                self._transaction_rows.append(len(items))
                self._commit_durations_s.append(time.perf_counter() - started)
                for item in items:
                    latency = committed_at - item.accepted_monotonic
                    self._receive_to_commit_s.append(latency)
                    self._stream_latencies_s[item.stream_name].append(latency)

        with self._lock:
            self._committed += len(committed)
            self._committed_payload_bytes += sum(item.payload_bytes for item in committed)
            for item in committed:
                self._stream_committed[item.stream_name] += 1
                self._stream_committed_bytes[item.stream_name] += item.payload_bytes
            if self._oldest_monotonic is not None:
                self._max_oldest_age_s = max(
                    self._max_oldest_age_s, time.monotonic() - self._oldest_monotonic
                )
            self._oldest_monotonic = self._next_accepted_monotonic()

    def _next_accepted_monotonic(self) -> float | None:
        with self._queue.mutex:
            for queued in self._queue.queue:
                if queued is not None:
                    return cast("_Write", queued).accepted_monotonic
        return None

    def _raise_if_failed_locked(self) -> None:
        if self._failure is not None:
            raise RecorderFailedError("Recorder writer failed") from self._failure
