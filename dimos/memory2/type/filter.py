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
from dataclasses import dataclass, field, fields
from itertools import islice
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.type.observation import Observation
    from dimos.models.embedding.base import Embedding


@dataclass(frozen=True)
class Filter(ABC):
    """Any object with a .matches(obs) -> bool method can be a filter."""

    @abstractmethod
    def matches(self, obs: Observation[Any]) -> bool: ...

    def __str__(self) -> str:
        args = ", ".join(f"{f.name}={getattr(self, f.name)!r}" for f in fields(self))
        return f"{self.__class__.__name__}({args})"


@dataclass(frozen=True)
class AfterFilter(Filter):
    t: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts > self.t


@dataclass(frozen=True)
class BeforeFilter(Filter):
    t: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts < self.t


@dataclass(frozen=True)
class TimeRangeFilter(Filter):
    t1: float
    t2: float

    def matches(self, obs: Observation[Any]) -> bool:
        return self.t1 <= obs.ts <= self.t2


@dataclass(frozen=True)
class AtFilter(Filter):
    t: float
    tolerance: float = 1.0

    def matches(self, obs: Observation[Any]) -> bool:
        return abs(obs.ts - self.t) <= self.tolerance


@dataclass(frozen=True)
class NearFilter(Filter):
    pose: Any = field(hash=False)
    radius: float = 0.0

    def matches(self, obs: Observation[Any]) -> bool:
        if obs.pose is None or self.pose is None:
            return False
        p1 = self.pose
        p2 = obs.pose
        # Support both raw (x,y,z) tuples and PoseStamped objects
        if hasattr(p1, "position"):
            p1 = p1.position
        if hasattr(p2, "position"):
            p2 = p2.position
        x1, y1, z1 = _xyz(p1)
        x2, y2, z2 = _xyz(p2)
        dist_sq = (x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2
        return dist_sq <= self.radius**2


def _xyz(p: Any) -> tuple[float, float, float]:
    """Extract (x, y, z) from various pose representations."""
    if isinstance(p, (list, tuple)):
        return (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)
    return (float(p.x), float(p.y), float(getattr(p, "z", 0.0)))


@dataclass(frozen=True)
class TagsFilter(Filter):
    tags: dict[str, Any] = field(default_factory=dict, hash=False)

    def matches(self, obs: Observation[Any]) -> bool:
        for k, v in self.tags.items():
            if obs.tags.get(k) != v:
                return False
        return True


@dataclass(frozen=True)
class PredicateFilter(Filter):
    """Wraps an arbitrary predicate function for use with .filter()."""

    fn: Callable[[Observation[Any]], bool] = field(hash=False)

    def matches(self, obs: Observation[Any]) -> bool:
        return bool(self.fn(obs))


@dataclass(frozen=True)
class StreamQuery:
    filters: tuple[Filter, ...] = ()
    order_field: str | None = None
    order_desc: bool = False
    limit_val: int | None = None
    offset_val: int | None = None
    live_buffer: BackpressureBuffer[Any] | None = None
    # Vector search (embedding similarity)
    search_vec: Embedding | None = field(default=None, hash=False, compare=False)
    search_k: int | None = None
    # Full-text search (substring / FTS5)
    search_text: str | None = None

    def __str__(self) -> str:
        parts: list[str] = [str(f) for f in self.filters]
        if self.search_text is not None:
            parts.append(f"search({self.search_text!r})")
        if self.search_vec is not None:
            k = f", k={self.search_k}" if self.search_k is not None else ""
            parts.append(f"vector_search({k.lstrip(', ')})" if k else "vector_search()")
        if self.order_field:
            direction = " DESC" if self.order_desc else ""
            parts.append(f"order_by({self.order_field}{direction})")
        if self.offset_val:
            parts.append(f"offset({self.offset_val})")
        if self.limit_val is not None:
            parts.append(f"limit({self.limit_val})")
        return " | ".join(parts)

    def apply(
        self, it: Iterator[Observation[Any]], *, live: bool = False
    ) -> Iterator[Observation[Any]]:
        """Apply all query operations to an iterator in Python.

        Used as the fallback execution path for transform-sourced streams
        and in-memory backends. Backends with native query support (SQL,
        ANN indexes) should push down operations instead.
        """
        # Filters
        if self.filters:
            it = (obs for obs in it if all(f.matches(obs) for f in self.filters))

        # Text search — substring match
        if self.search_text is not None:
            needle = self.search_text.lower()
            it = (obs for obs in it if needle in str(obs.data).lower())

        # Vector search — brute-force cosine (materializes)
        if self.search_vec is not None:
            if live:
                raise TypeError(
                    ".search() requires finite data — cannot rank an infinite live stream."
                )
            query_emb = self.search_vec
            scored = []
            for obs in it:
                emb = getattr(obs, "embedding", None)
                if emb is not None:
                    sim = float(emb @ query_emb)
                    scored.append(obs.derive(data=obs.data, similarity=sim))
            scored.sort(key=lambda o: getattr(o, "similarity", 0.0) or 0.0, reverse=True)
            if self.search_k is not None:
                scored = scored[: self.search_k]
            it = iter(scored)

        # Sort (materializes)
        if self.order_field:
            if live:
                raise TypeError(
                    ".order_by() requires finite data — cannot sort an infinite live stream."
                )
            key = self.order_field
            desc = self.order_desc
            items = sorted(
                list(it),
                key=lambda obs: getattr(obs, key) if getattr(obs, key, None) is not None else 0,
                reverse=desc,
            )
            it = iter(items)

        # Offset + limit
        if self.offset_val:
            it = islice(it, self.offset_val, None)
        if self.limit_val is not None:
            it = islice(it, self.limit_val)

        return it
