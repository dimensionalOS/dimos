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

"""Bounded FIFO processing for Recorder."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import threading
import time
from typing import Any

from dimos.memory.buffer import DropNew
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_QUEUE_SIZE = 4096
_DEFAULT_DRAIN_TIMEOUT_S = 10.0


class RecordingFailedError(RuntimeError):
    """Raised when Recorder cannot persist every accepted message."""


Processor = Callable[[Any], None]


class RecordingPipeline:
    """Run stream processors in callback-arrival order on one worker."""

    def __init__(
        self,
        processors: dict[str, Processor],
        *,
        queue_size: int = _DEFAULT_QUEUE_SIZE,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        self._processors = processors
        self._ingress = DropNew[tuple[str, Any]](queue_size)
        self._failure: Exception | None = None
        self._lock = threading.Lock()
        self._callback_condition = threading.Condition(self._lock)
        self._active_callbacks = 0
        self._started = threading.Event()
        self._closed = threading.Event()
        self._worker: threading.Thread | None = None

    def start(self) -> None:
        with self._lock:
            if self._closed.is_set():
                raise RecordingFailedError("Recording pipeline is closed")
            if self._started.is_set():
                return
            self._worker = threading.Thread(
                target=self._work_loop,
                name="recorder-writer",
                daemon=True,
            )
            self._worker.start()
            self._started.set()

    @contextmanager
    def callback(self) -> Iterator[None]:
        """Admit one transport callback and keep shutdown behind it."""
        with self._callback_condition:
            self._raise_if_unavailable()
            self._active_callbacks += 1
        try:
            yield
        except Exception as error:
            self._fail(error)
            raise
        finally:
            with self._callback_condition:
                self._active_callbacks -= 1
                self._callback_condition.notify_all()

    def submit(self, stream: str, value: Any) -> None:
        with self._lock:
            self._raise_if_unavailable()
            if stream not in self._processors:
                raise KeyError(f"Unknown recording stream {stream!r}")
            if self._ingress.put((stream, value)):
                return
            error = RecordingFailedError("Recording queue reached capacity")
            self._failure = error
        logger.error("Recording pipeline failed", error=repr(error))
        raise error

    def close(self, timeout_s: float = _DEFAULT_DRAIN_TIMEOUT_S) -> None:
        deadline = time.monotonic() + timeout_s
        with self._callback_condition:
            if not self._started.is_set():
                self._closed.set()
                self._raise_if_failed()
                return
            if not self._closed.is_set():
                while self._active_callbacks:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise RecordingFailedError("Recorder callbacks did not stop")
                    self._callback_condition.wait(timeout=remaining)
                self._closed.set()
                self._ingress.close()
            worker = self._worker

        if worker is not None:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
            if worker.is_alive():
                raise RecordingFailedError("Recorder worker did not stop")
        with self._lock:
            self._raise_if_failed()

    def _work_loop(self) -> None:
        for stream, value in self._ingress:
            try:
                self._processors[stream](value)
            except Exception as error:
                self._fail(error)
                return

    def _fail(self, error: Exception) -> None:
        with self._lock:
            if self._failure is not None:
                return
            self._failure = error
        logger.error("Recording pipeline failed", error=repr(error), exc_info=error)

    def _raise_if_unavailable(self) -> None:
        if not self._started.is_set():
            raise RecordingFailedError("Recording pipeline is not started")
        if self._closed.is_set():
            raise RecordingFailedError("Recording pipeline is closed")
        self._raise_if_failed()

    def _raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RecordingFailedError("Recording pipeline failed") from self._failure
