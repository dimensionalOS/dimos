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

"""Lossless preparation and persistence pipeline for Recorder."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable, Iterable
from dataclasses import dataclass
import threading
import time
from typing import Any

from dimos.memory.backend import Backend, PreparedAppend
from dimos.memory.buffer import ClosedError, DropNew


class RecordingFailedError(RuntimeError):
    """Raised when the pipeline cannot persist every accepted message."""


@dataclass(frozen=True)
class PreparedWrite:
    backend: Backend[Any]
    append: PreparedAppend[Any]


Processor = Callable[[Any], Iterable[PreparedWrite]]


class RecordingPipeline:
    """Prepare streams in parallel and serialize batched database writes."""

    def __init__(
        self,
        processors: dict[str, Processor],
        *,
        ingress_size: int = 256,
        writer_size: int = 4096,
        batch_rows: int = 64,
        batch_delay_s: float = 0.010,
    ) -> None:
        self._processors = processors
        self._ingress = {name: DropNew[Any](ingress_size) for name in processors}
        self._writer = DropNew[PreparedWrite](writer_size)
        self._batch_rows = batch_rows
        self._batch_delay_s = batch_delay_s
        self._failure: BaseException | None = None
        self._lock = threading.Lock()
        self._started = threading.Event()
        self._closed = threading.Event()
        self._preparation_threads: list[threading.Thread] = []
        self._writer_thread: threading.Thread | None = None

    def start(self) -> None:
        with self._lock:
            if self._closed.is_set():
                raise RecordingFailedError("Recording pipeline is closed")
            if self._started.is_set():
                return
            self._writer_thread = threading.Thread(
                target=self._write_loop,
                name="recorder-writer",
                daemon=True,
            )
            self._preparation_threads = [
                threading.Thread(
                    target=self._prepare_loop,
                    args=(name,),
                    name=f"recorder-prepare-{name}",
                    daemon=True,
                )
                for name in self._processors
            ]
            self._writer_thread.start()
            for thread in self._preparation_threads:
                thread.start()
            self._started.set()

    def submit(self, stream: str, value: Any) -> None:
        self._raise_if_unavailable()
        try:
            ingress = self._ingress[stream]
        except KeyError as unknown_stream:
            raise KeyError(f"Unknown recording stream {stream!r}") from unknown_stream
        if not ingress.put(value):
            capacity_error = RecordingFailedError(
                f"Recording ingress for {stream!r} reached capacity"
            )
            self.fail(capacity_error)
            raise capacity_error

    def fail(self, error: BaseException) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = error

    def close(self, timeout_s: float = 10.0) -> None:
        if not self._started.is_set():
            self._closed.set()
            self._raise_if_failed()
            return
        first_close = not self._closed.is_set()
        self._closed.set()
        deadline = time.monotonic() + timeout_s
        if first_close:
            for ingress in self._ingress.values():
                ingress.close()
        shutdown_failure: BaseException | None = None
        for thread in self._preparation_threads:
            try:
                self._join(thread, deadline)
            except BaseException as error:
                shutdown_failure = shutdown_failure or error
        self._writer.close()
        if self._writer_thread is not None:
            try:
                self._join(self._writer_thread, deadline)
            except BaseException as error:
                shutdown_failure = shutdown_failure or error
        if shutdown_failure is not None:
            raise RecordingFailedError("Recording pipeline did not stop") from shutdown_failure
        self._raise_if_failed()

    def _prepare_loop(self, stream: str) -> None:
        try:
            for value in self._ingress[stream]:
                if self._has_failed():
                    continue
                for write in self._processors[stream](value):
                    if not self._writer.put(write):
                        raise RecordingFailedError("Recording writer buffer reached capacity")
        except BaseException as error:
            self.fail(error)

    def _write_loop(self) -> None:
        while True:
            try:
                first = self._writer.take()
            except ClosedError:
                return
            batch = [first]
            deadline = time.monotonic() + self._batch_delay_s
            while len(batch) < self._batch_rows:
                try:
                    batch.append(self._writer.take(timeout=max(0.0, deadline - time.monotonic())))
                except TimeoutError:
                    break
                except ClosedError:
                    break
            try:
                self._persist(batch)
            except BaseException as error:
                self.fail(error)
                return

    @staticmethod
    def _persist(batch: list[PreparedWrite]) -> None:
        grouped: dict[int, list[PreparedAppend[Any]]] = defaultdict(list)
        backends: dict[int, Backend[Any]] = {}
        for write in batch:
            key = id(write.backend)
            backends[key] = write.backend
            grouped[key].append(write.append)
        for key, appends in grouped.items():
            backends[key].append_prepared(appends)

    def _raise_if_unavailable(self) -> None:
        if not self._started.is_set():
            raise RecordingFailedError("Recording pipeline is not started")
        if self._closed.is_set():
            raise RecordingFailedError("Recording pipeline is closed")
        self._raise_if_failed()

    def _has_failed(self) -> bool:
        with self._lock:
            return self._failure is not None

    def _raise_if_failed(self) -> None:
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise RecordingFailedError("Recording pipeline failed") from failure

    @staticmethod
    def _join(thread: threading.Thread, deadline: float) -> None:
        thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if thread.is_alive():
            raise RecordingFailedError(f"{thread.name} did not stop")
