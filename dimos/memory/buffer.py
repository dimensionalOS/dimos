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

"""Backpressure buffers — the bridge between push and pull.

Real-world data sources (cameras, LiDAR, ROS topics) and ReactiveX pipelines
are *push-based*: they emit items whenever they please. Databases, analysis
systems, and our memory store are *pull-based*: consumers iterate at their own
pace. A BackpressureBuffer sits between the two, absorbing push bursts so
that the pull side can drain items on its own schedule.

The choice of strategy controls what happens under load:

- **KeepLast** — single-slot, always overwrites; best for real-time sensor
  data where only the latest reading matters.
- **Bounded** — FIFO with a cap; drops the oldest item on overflow.
- **DropNew** — FIFO with a cap; rejects new items on overflow.
- **Unbounded** — unlimited FIFO; guarantees delivery at the cost of memory.

All four share the same ABC interface and are interchangeable wherever a
buffer is accepted (e.g. ``Stream.live(buffer=...)``).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
import threading
from typing import TYPE_CHECKING, Generic, TypeVar

if TYPE_CHECKING:
    from collections.abc import Iterator

T = TypeVar("T")


class ClosedError(Exception):
    """Raised when take() is called on a closed buffer."""


class BackpressureBuffer(ABC, Generic[T]):
    """Thread-safe buffer between push producers and pull consumers."""

    @abstractmethod
    def put(self, item: T) -> bool:
        """Push an item. Returns False if the item was dropped."""

    @abstractmethod
    def take(self, timeout: float | None = None) -> T:
        """Block until an item is available. Raises ClosedError if the buffer is closed."""

    @abstractmethod
    def try_take(self) -> T | None:
        """Non-blocking take. Returns None if empty."""

    @abstractmethod
    def close(self) -> None:
        """Signal no more items. Subsequent take() raises ClosedError."""

    @abstractmethod
    def __len__(self) -> int: ...

    def __iter__(self) -> Iterator[T]:
        """Yield items until the buffer is closed."""
        while True:
            try:
                yield self.take()
            except ClosedError:
                return


class KeepLast(BackpressureBuffer[T]):
    """Single-slot buffer. put() always overwrites. Default for live mode."""

    def __init__(self) -> None:
        self._item: T | None = None
        self._has_item = False
        self._closed = False
        self._cond = threading.Condition()

    def put(self, item: T) -> bool:
        with self._cond:
            if self._closed:
                return False
            self._item = item
            self._has_item = True
            self._cond.notify()
        return True

    def take(self, timeout: float | None = None) -> T:
        with self._cond:
            while not self._has_item:
                if self._closed:
                    raise ClosedError("Buffer is closed")
                if not self._cond.wait(timeout):
                    raise TimeoutError("take() timed out")
            item = self._item
            assert item is not None
            self._item = None
            self._has_item = False
            return item

    def try_take(self) -> T | None:
        with self._cond:
            if not self._has_item:
                return None
            item = self._item
            self._item = None
            self._has_item = False
            return item

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def __len__(self) -> int:
        with self._cond:
            return 1 if self._has_item else 0


class Bounded(BackpressureBuffer[T]):
    """FIFO queue with max size. Drops oldest when full."""

    def __init__(self, maxlen: int) -> None:
        self._buf: deque[T] = deque(maxlen=maxlen)
        self._closed = False
        self._cond = threading.Condition()

    def put(self, item: T) -> bool:
        with self._cond:
            if self._closed:
                return False
            self._buf.append(item)  # deque(maxlen) drops oldest automatically
            self._cond.notify()
        return True

    def take(self, timeout: float | None = None) -> T:
        with self._cond:
            while not self._buf:
                if self._closed:
                    raise ClosedError("Buffer is closed")
                if not self._cond.wait(timeout):
                    raise TimeoutError("take() timed out")
            return self._buf.popleft()

    def try_take(self) -> T | None:
        with self._cond:
            return self._buf.popleft() if self._buf else None

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def __len__(self) -> int:
        with self._cond:
            return len(self._buf)


class DropNew(BackpressureBuffer[T]):
    """FIFO queue. Rejects new items when full (put returns False)."""

    def __init__(self, maxlen: int) -> None:
        self._buf: deque[T] = deque()
        self._maxlen = maxlen
        self._closed = False
        self._cond = threading.Condition()

    def put(self, item: T) -> bool:
        with self._cond:
            if self._closed or len(self._buf) >= self._maxlen:
                return False
            self._buf.append(item)
            self._cond.notify()
        return True

    def take(self, timeout: float | None = None) -> T:
        with self._cond:
            while not self._buf:
                if self._closed:
                    raise ClosedError("Buffer is closed")
                if not self._cond.wait(timeout):
                    raise TimeoutError("take() timed out")
            return self._buf.popleft()

    def try_take(self) -> T | None:
        with self._cond:
            return self._buf.popleft() if self._buf else None

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def __len__(self) -> int:
        with self._cond:
            return len(self._buf)


class Unbounded(BackpressureBuffer[T]):
    """Unbounded FIFO queue. Use carefully — can grow without limit."""

    def __init__(self) -> None:
        self._buf: deque[T] = deque()
        self._closed = False
        self._cond = threading.Condition()

    def put(self, item: T) -> bool:
        with self._cond:
            if self._closed:
                return False
            self._buf.append(item)
            self._cond.notify()
        return True

    def take(self, timeout: float | None = None) -> T:
        with self._cond:
            while not self._buf:
                if self._closed:
                    raise ClosedError("Buffer is closed")
                if not self._cond.wait(timeout):
                    raise TimeoutError("take() timed out")
            return self._buf.popleft()

    def try_take(self) -> T | None:
        with self._cond:
            return self._buf.popleft() if self._buf else None

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def __len__(self) -> int:
        with self._cond:
            return len(self._buf)
