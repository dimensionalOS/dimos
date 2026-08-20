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

"""Bounded, lossless ingress queues for the Recorder."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import queue
import threading
import time
from typing import Any


class RecorderFailedError(RuntimeError):
    """Raised when a recorder queue can no longer persist accepted messages."""


@dataclass(frozen=True)
class _QueuedMessage:
    value: Any


class RecorderQueue:
    """Run one stream's potentially slow preparation/write path in FIFO order.

    Subscriber callbacks only enqueue. A dedicated worker performs pose lookup,
    encoding, and persistence, so a busy stream never triggers latest-value
    coalescing in the module's general asynchronous dispatcher.
    """

    def __init__(
        self,
        name: str,
        process: Callable[[Any], None],
        *,
        max_pending: int = 65_536,
    ) -> None:
        self.name = name
        self._process = process
        self._queue: queue.Queue[_QueuedMessage | None] = queue.Queue(maxsize=max_pending)
        self._lock = threading.Lock()
        self._failure: BaseException | None = None
        self._accepting = True
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name=f"recorder-{name}",
            daemon=True,
        )
        self._thread.start()

    def submit(self, value: Any) -> None:
        with self._lock:
            self._raise_if_failed_locked()
            if not self._accepting:
                raise RecorderFailedError(f"Recorder queue {self.name!r} is closed")
        item = _QueuedMessage(value)
        try:
            self._queue.put_nowait(item)
        except queue.Full as error:
            with self._lock:
                self._fail_locked(
                    RecorderFailedError(
                        f"Recorder queue {self.name!r} reached its {self._queue.maxsize} "
                        "message capacity"
                    )
                )
            raise RecorderFailedError(f"Recorder queue {self.name!r} is full") from error

    def close_input(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._accepting = False
            self._closed = True
        self._queue.put(None)

    def fail(self, error: BaseException) -> None:
        """Enter the fatal state without discarding already accepted work."""
        with self._lock:
            self._fail_locked(error)

    def flush(self, timeout_s: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_s
        while self._queue.unfinished_tasks:
            with self._lock:
                self._raise_if_failed_locked()
            if time.monotonic() >= deadline:
                raise RecorderFailedError(
                    f"Recorder queue {self.name!r} did not drain within {timeout_s:.3f}s"
                )
            time.sleep(0.005)
        with self._lock:
            self._raise_if_failed_locked()

    def close(self, timeout_s: float = 10.0) -> None:
        self.close_input()
        deadline = time.monotonic() + timeout_s
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self._thread.is_alive():
            raise RecorderFailedError(f"Recorder queue {self.name!r} worker did not stop")
        with self._lock:
            self._raise_if_failed_locked()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                with self._lock:
                    failed = self._failure is not None
                if not failed:
                    self._process(item.value)
            except BaseException as error:
                with self._lock:
                    self._fail_locked(error)
            finally:
                self._queue.task_done()

    def _fail_locked(self, error: BaseException) -> None:
        if self._failure is None:
            self._failure = error
        self._accepting = False

    def _raise_if_failed_locked(self) -> None:
        if self._failure is not None:
            raise RecorderFailedError(f"Recorder queue {self.name!r} failed") from self._failure
