# Copyright 2025 Dimensional Inc.
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

from abc import ABC, abstractmethod
import threading
from typing import Generic, TypeVar

T = TypeVar("T")


class Accumulator(ABC, Generic[T]):
    """Base class for accumulating messages between rate-limited calls."""

    @abstractmethod
    def add(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """Add args and kwargs to the accumulator."""
        pass

    @abstractmethod
    def get(self) -> tuple[tuple, dict] | None:  # type: ignore[type-arg]
        """Get the accumulated args and kwargs and reset the accumulator."""
        pass

    @abstractmethod
    def __len__(self) -> int:
        """Return the number of accumulated items."""
        pass


class LatestAccumulator(Accumulator[T]):
    """Simple accumulator that remembers only the latest args and kwargs."""

    def __init__(self) -> None:
        self._latest: tuple[tuple, dict] | None = None  # type: ignore[type-arg]
        self._lock = threading.Lock()

    def add(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        with self._lock:
            self._latest = (args, kwargs)

    def get(self) -> tuple[tuple, dict] | None:  # type: ignore[type-arg]
        with self._lock:
            result = self._latest
            self._latest = None
            return result

    def __len__(self) -> int:
        with self._lock:
            return 1 if self._latest is not None else 0


class RollingAverageAccumulator(Accumulator[T]):
    """Accumulator that maintains a rolling average of the first argument.

    This accumulator expects the first argument to be numeric and maintains
    a rolling average without storing individual values.
    """

    def __init__(self) -> None:
        self._sum: float = 0.0
        self._count: int = 0
        self._latest_kwargs: dict = {}  # type: ignore[type-arg]
        self._lock = threading.Lock()

    def add(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        if not args:
            raise ValueError("RollingAverageAccumulator requires at least one argument")

        with self._lock:
            try:
                value = float(args[0])
                self._sum += value
                self._count += 1
                self._latest_kwargs = kwargs
            except (TypeError, ValueError):
                raise TypeError(f"First argument must be numeric, got {type(args[0])}")

    def get(self) -> tuple[tuple, dict] | None:  # type: ignore[type-arg]
        with self._lock:
            if self._count == 0:
                return None

            average = self._sum / self._count
            result = ((average,), self._latest_kwargs)

            # Reset accumulator
            self._sum = 0.0
            self._count = 0
            self._latest_kwargs = {}

            return result

    def __len__(self) -> int:
        with self._lock:
            return self._count
