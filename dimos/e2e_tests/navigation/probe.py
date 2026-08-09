# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import threading
import time
from typing import Generic, TypeVar, cast

from dimos.core.stream import Transport
from dimos.core.transport_factory import make_transport

T = TypeVar("T")


@dataclass(frozen=True)
class Observation(Generic[T]):
    """One received value and the marker immediately after it."""

    sequence: int
    value: T


class StreamProbe(Generic[T]):
    """Record one typed DimOS stream without depending on its transport."""

    def __init__(
        self,
        name: str,
        msg_type: type[T] | None = None,
        *,
        transport: Transport[T] | None = None,
    ) -> None:
        self._transport = transport or cast("Transport[T]", make_transport(name, msg_type))
        self._condition = threading.Condition()
        self._messages: list[T] = []
        self._unsubscribe: Callable[[], None] | None = None

    def start(self) -> None:
        if self._unsubscribe is not None:
            return
        self._unsubscribe = self._transport.subscribe(self._receive)

    def stop(self) -> None:
        unsubscribe, self._unsubscribe = self._unsubscribe, None
        if unsubscribe is not None:
            unsubscribe()
        self._transport.stop()

    def mark(self) -> int:
        """Return a marker that excludes every value received so far."""
        with self._condition:
            return len(self._messages)

    def wait_for(
        self,
        predicate: Callable[[T], bool],
        *,
        after: int = 0,
        timeout: float,
        failure_message: str,
    ) -> Observation[T]:
        """Wait for a matching value after a marker and return its next marker."""
        if after < 0:
            raise ValueError("after must not be negative")

        deadline = time.monotonic() + timeout
        cursor = after
        with self._condition:
            while True:
                for index in range(cursor, len(self._messages)):
                    value = self._messages[index]
                    if predicate(value):
                        return Observation(sequence=index + 1, value=value)
                cursor = len(self._messages)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(failure_message)
                self._condition.wait(remaining)

    def _receive(self, value: T) -> None:
        with self._condition:
            self._messages.append(value)
            self._condition.notify_all()


__all__ = ["Observation", "StreamProbe"]
