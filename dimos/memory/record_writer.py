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

from collections import defaultdict
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
    oldest_age_s: float
    max_pending: int
    max_oldest_age_s: float
    failed: str | None
    mean_rows_per_transaction: float
    max_rows_per_transaction: int


@dataclass(frozen=True)
class _Write:
    backend: Backend[Any]
    prepared: PreparedAppend[Any]
    accepted_monotonic: float


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
        self._max_pending = 0
        self._max_oldest_age_s = 0.0
        self._oldest_monotonic: float | None = None
        self._failure: BaseException | None = None
        self._transaction_rows = 0
        self._max_rows_per_transaction = 0
        self._closed = False
        self._thread = threading.Thread(
            target=self._run, name="recorder-sqlite-writer", daemon=True
        )
        self._thread.start()

    def submit(
        self,
        backend: Backend[Any],
        prepared: PreparedAppend[Any],
    ) -> None:
        accepted = time.monotonic()
        with self._lock:
            self._raise_if_failed_locked()
            if self._closed:
                raise RecorderFailedError("Recorder writer is closed")
            self._submitted += 1
            self._max_pending = max(self._max_pending, self._submitted - self._committed)
            if self._oldest_monotonic is None:
                self._oldest_monotonic = accepted
        self._queue.put(_Write(backend, prepared, accepted))

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
            oldest_age = 0.0 if self._oldest_monotonic is None else now - self._oldest_monotonic
            self._max_oldest_age_s = max(self._max_oldest_age_s, oldest_age)
            return RecordWriterStatus(
                submitted=self._submitted,
                committed=self._committed,
                transactions=self._transactions,
                pending=self._submitted - self._committed,
                oldest_age_s=oldest_age,
                max_pending=self._max_pending,
                max_oldest_age_s=self._max_oldest_age_s,
                failed=str(self._failure) if self._failure is not None else None,
                mean_rows_per_transaction=(
                    self._transaction_rows / self._transactions if self._transactions else 0.0
                ),
                max_rows_per_transaction=self._max_rows_per_transaction,
            )

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
            with self._lock:
                self._transactions += 1
                self._transaction_rows += len(items)
                self._max_rows_per_transaction = max(self._max_rows_per_transaction, len(items))

        with self._lock:
            self._committed += len(committed)
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
