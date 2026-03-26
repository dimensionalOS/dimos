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

"""Callback collector with Event-based synchronization for async tests."""

import threading
from typing import Any


class CallbackCollector:
    """Callable that collects ``(msg, topic)`` pairs and signals when *n* arrive.

    Designed as a drop-in subscription callback for pubsub tests::

        collector = CallbackCollector(3)
        sub.subscribe(topic, collector)
        # ... publish 3 messages ...
        collector.wait()
        assert len(collector.results) == 3
    """

    def __init__(self, n: int, timeout: float = 2.0) -> None:
        self.results: list[tuple[Any, Any]] = []
        self._done = threading.Event()
        self._n = n
        self.timeout = timeout

    def __call__(self, msg: Any, topic: Any) -> None:
        self.results.append((msg, topic))
        if len(self.results) >= self._n:
            self._done.set()

    def wait(self) -> None:
        """Block until *n* items have been collected, or *timeout* expires."""
        if not self._done.wait(self.timeout):
            raise AssertionError(
                f"Timed out after {self.timeout}s waiting for {self._n} messages "
                f"(got {len(self.results)})"
            )
