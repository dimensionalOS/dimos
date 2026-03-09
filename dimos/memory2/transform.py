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

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Generic, TypeVar

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.memory2.type import Observation

T = TypeVar("T")
R = TypeVar("R")


class Transformer(ABC, Generic[T, R]):
    """Transforms a stream of observations lazily via iterator -> iterator.

    Pull from upstream, yield transformed observations. Naturally supports
    batching, windowing, fan-out. The generator cleans
    up when upstream exhausts.
    """

    @abstractmethod
    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]: ...


class FnTransformer(Transformer[T, R]):
    """Wraps a callable that receives an Observation and returns a new one (or None to skip)."""

    def __init__(self, fn: Callable[[Observation[T]], Observation[R] | None]) -> None:
        self._fn = fn

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
        fn = self._fn
        for obs in upstream:
            result = fn(obs)
            if result is not None:
                yield result


class QualityWindow(Transformer[T, T]):
    """Keeps the highest-quality item per time window.

    Emits the best observation when the window advances. The last window
    is emitted when the upstream iterator exhausts — no flush needed.
    """

    def __init__(self, quality_fn: Callable[[Any], float], window: float) -> None:
        self._quality_fn = quality_fn
        self._window = window

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        quality_fn = self._quality_fn
        window = self._window
        best: Observation[T] | None = None
        best_score: float = -1.0
        window_start: float | None = None

        for obs in upstream:
            if window_start is not None and (obs.ts - window_start) >= window:
                if best is not None:
                    yield best
                best = None
                best_score = -1.0
                window_start = obs.ts

            score = quality_fn(obs.data)
            if score > best_score:
                best = obs
                best_score = score
            if window_start is None:
                window_start = obs.ts

        if best is not None:
            yield best
