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

from collections.abc import Iterator
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import threading


class CallPool:
    """Lazy handler executor with cooperative, reentrant shutdown.

    Services share ``lock`` with their startup code and enclose their entire
    stop method in ``stop_scope``. Joining workers never holds that lock.
    """

    def __init__(self, max_workers: int) -> None:
        self._max_workers = max_workers
        self.lock = threading.RLock()
        self._executor: ThreadPoolExecutor | None = None
        self._worker = threading.local()
        self._stop_calls = 0
        self._stopped = False

    def __getstate__(self) -> dict[str, int]:
        # Only configuration crosses into a worker process.
        return {"max_workers": self._max_workers}

    def __setstate__(self, state: dict[str, int]) -> None:
        CallPool.__init__(self, state["max_workers"])

    def start(self) -> None:
        with self.lock:
            if self._stop_calls:
                raise RuntimeError("Service is still stopping; cannot restart yet")
            if self._stopped:
                self._executor = None
                self._stopped = False

    @contextmanager
    def stop_scope(self) -> Iterator[None]:
        """Reject restart and new handlers until every stop caller finishes."""
        with self.lock:
            self._stop_calls += 1
            self._stopped = True
        try:
            yield
        finally:
            with self.lock:
                self._stop_calls -= 1

    def _mark_worker(self) -> None:
        self._worker.active = True

    def get(self) -> ThreadPoolExecutor:
        with self.lock:
            if self._stopped:
                raise RuntimeError("Service is stopping or stopped; cannot accept calls")
            if self._executor is None:
                self._executor = ThreadPoolExecutor(
                    max_workers=self._max_workers, initializer=self._mark_worker
                )
            return self._executor

    def shutdown(self, *, wait: bool = True) -> None:
        with self.lock:
            self._stopped = True
            pool = self._executor
        if pool is None:
            return
        wait = wait and not getattr(self._worker, "active", False)
        # Workers may call stop() themselves, so never join with lock held.
        pool.shutdown(wait=wait)
        if wait:
            with self.lock:
                if self._executor is pool:
                    self._executor = None
        # A nonwaiting caller leaves the pool attached: a later external stop
        # must still drain those handlers before tearing down their transport.
