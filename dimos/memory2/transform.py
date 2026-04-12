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
import inspect
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.utils.formatting import FilterRepr

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator, Sequence

    from dimos.memory2.type.observation import Observation

T = TypeVar("T")
R = TypeVar("R")


class Transformer(FilterRepr, ABC, Generic[T, R]):
    """Transforms a stream of observations lazily via iterator -> iterator.

    Pull from upstream, yield transformed observations. Naturally supports
    batching, windowing, fan-out. The generator cleans
    up when upstream exhausts.
    """

    @abstractmethod
    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]: ...

    def __str__(self) -> str:
        parts: list[str] = []
        for name in inspect.signature(self.__init__).parameters:  # type: ignore[misc]
            for attr in (name, f"_{name}"):
                if hasattr(self, attr):
                    val = getattr(self, attr)
                    if callable(val):
                        parts.append(f"{name}={getattr(val, '__name__', '...')}")
                    else:
                        parts.append(f"{name}={val!r}")
                    break
        return f"{self.__class__.__name__}({', '.join(parts)})"


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


class FnIterTransformer(Transformer[T, R]):
    """Wraps a bare ``Iterator → Iterator`` callable (e.g. a generator function)."""

    def __init__(self, fn: Callable[[Iterator[Observation[T]]], Iterator[Observation[R]]]) -> None:
        self._fn = fn

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
        return self._fn(upstream)


class Batch(Transformer[T, R]):
    """Batched transform: collects observations, applies a batch function, derives new data.

    The ``fn`` receives a list of data items and returns a list of results,
    one per input (e.g. ``model.caption_batch``, ``model.embed``).
    """

    def __init__(self, fn: Callable[[list[T]], Sequence[R]], batch_size: int = 16) -> None:
        self._fn = fn
        self._batch_size = batch_size

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
        fn = self._fn
        batch: list[Observation[T]] = []
        for obs in upstream:
            batch.append(obs)
            if len(batch) >= self._batch_size:
                results = fn([o.data for o in batch])
                for o, r in zip(batch, results, strict=True):
                    yield o.derive(data=r)
                batch = []
        if batch:
            results = fn([o.data for o in batch])
            for o, r in zip(batch, results, strict=True):
                yield o.derive(data=r)


def stride(n: int) -> FnIterTransformer[T, T]:
    """Yield every *n*-th observation, skipping the rest."""
    if n < 1:
        raise ValueError(f"stride(n) requires n >= 1, got {n}")

    def _stride(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        for i, obs in enumerate(upstream):
            if i % n == 0:
                yield obs

    return FnIterTransformer(_stride)


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
