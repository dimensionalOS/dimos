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

from itertools import islice
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.backend import Backend
from dimos.memory2.buffer import BackpressureBuffer, ClosedError, KeepLast
from dimos.memory2.filter import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    Filter,
    NearFilter,
    PredicateFilter,
    StreamQuery,
    TagsFilter,
    TimeRangeFilter,
)
from dimos.memory2.transform import FnTransformer, Transformer

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.type import Observation

T = TypeVar("T")
R = TypeVar("R")


class Stream(Generic[T]):
    """Lazy, pull-based stream over observations.

    Every filter/transform method returns a new Stream — no computation
    happens until iteration. Backends handle query application for stored
    data; transform sources apply filters as Python predicates.
    """

    def __init__(
        self,
        source: Backend[T] | Stream[Any],
        *,
        xf: Transformer[Any, T] | None = None,
        query: StreamQuery = StreamQuery(),
        _live_buf: BackpressureBuffer[Observation[Any]] | None = None,
        _live_sub: DisposableBase | None = None,
    ) -> None:
        self._source = source
        self._xf = xf
        self._query = query
        self._live_buf = _live_buf
        self._live_sub = _live_sub  # kept alive for lifetime of stream

    # ── Iteration ───────────────────────────────────────────────────

    def __iter__(self) -> Iterator[Observation[T]]:
        return self._build_iter()

    def _build_iter(self) -> Iterator[Observation[T]]:
        if isinstance(self._source, Stream):
            it = self._iter_transform()
        else:
            # Backend handles all query application
            it = self._source.iterate(self._query)

        # Live tail: after backfill exhausts, yield from live buffer
        if self._live_buf is not None:
            it = self._iter_with_live(it)

        return it

    def _iter_transform(self) -> Iterator[Observation[T]]:
        """Iterate a transform source, applying query filters in Python."""
        assert isinstance(self._source, Stream) and self._xf is not None
        it: Iterator[Observation[T]] = self._xf(iter(self._source))

        # Apply filters as Python predicates
        filters = self._query.filters
        if filters:
            it = (obs for obs in it if all(f.matches(obs) for f in filters))

        # Sort if needed (materializes — only for finite streams)
        if self._query.order_field:
            key = self._query.order_field
            desc = self._query.order_desc
            items = sorted(
                list(it),
                key=lambda obs: getattr(obs, key) if getattr(obs, key, None) is not None else 0,
                reverse=desc,
            )
            it = iter(items)

        # Offset + limit
        if self._query.offset_val:
            it = islice(it, self._query.offset_val, None)
        if self._query.limit_val is not None:
            it = islice(it, self._query.limit_val)

        return it

    def _iter_with_live(self, backfill: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        """Yield backfill, then switch to live tail."""
        last_id = -1
        for obs in backfill:
            last_id = max(last_id, obs.id)
            yield obs

        # Live phase
        buf = self._live_buf
        assert buf is not None
        filters = self._query.filters
        try:
            while True:
                obs = buf.take()
                if obs.id <= last_id:
                    continue
                last_id = obs.id
                if filters and not all(f.matches(obs) for f in filters):
                    continue
                yield obs
        except (ClosedError, StopIteration):
            return

    # ── Query builders ──────────────────────────────────────────────

    def _replace_query(self, **overrides: Any) -> Stream[T]:
        q = self._query
        new_q = StreamQuery(
            filters=overrides.get("filters", q.filters),
            order_field=overrides.get("order_field", q.order_field),
            order_desc=overrides.get("order_desc", q.order_desc),
            limit_val=overrides.get("limit_val", q.limit_val),
            offset_val=overrides.get("offset_val", q.offset_val),
        )
        return Stream(
            self._source,
            xf=self._xf,
            query=new_q,
            _live_buf=self._live_buf,
            _live_sub=self._live_sub,
        )

    def _with_filter(self, f: Filter) -> Stream[T]:
        return self._replace_query(filters=(*self._query.filters, f))

    def after(self, t: float) -> Stream[T]:
        return self._with_filter(AfterFilter(t))

    def before(self, t: float) -> Stream[T]:
        return self._with_filter(BeforeFilter(t))

    def time_range(self, t1: float, t2: float) -> Stream[T]:
        return self._with_filter(TimeRangeFilter(t1, t2))

    def at(self, t: float, tolerance: float = 1.0) -> Stream[T]:
        return self._with_filter(AtFilter(t, tolerance))

    def near(self, pose: Any, radius: float) -> Stream[T]:
        return self._with_filter(NearFilter(pose, radius))

    def filter_tags(self, **tags: Any) -> Stream[T]:
        return self._with_filter(TagsFilter(tags))

    def order_by(self, field: str, desc: bool = False) -> Stream[T]:
        return self._replace_query(order_field=field, order_desc=desc)

    def limit(self, k: int) -> Stream[T]:
        return self._replace_query(limit_val=k)

    def offset(self, n: int) -> Stream[T]:
        return self._replace_query(offset_val=n)

    # ── Functional API ──────────────────────────────────────────────

    def filter(self, pred: Callable[[Observation[T]], bool]) -> Stream[T]:
        """Filter by arbitrary predicate on the full Observation."""
        return self._with_filter(PredicateFilter(pred))

    def map(self, fn: Callable[[Observation[T]], Observation[R]]) -> Stream[Any]:
        """Transform each observation's data via callable."""
        return self.transform(FnTransformer(lambda obs: fn(obs)))

    # ── Transform ───────────────────────────────────────────────────

    def transform(self, xf: Transformer[T, R]) -> Stream[R]:
        """Wrap this stream with a transformer. Returns a new lazy Stream.

        When iterated, calls xf(iter(self)) — pulls lazily through the chain.
        """
        return Stream(source=self, xf=xf, query=StreamQuery())

    # ── Live mode ───────────────────────────────────────────────────

    def live(self, buffer: BackpressureBuffer[Observation[Any]] | None = None) -> Stream[T]:
        """Return a stream that yields backfill then live data (infinite iterator).

        Only valid on backend-backed streams. Transforms downstream of a live
        stream just see an infinite iterator — they don't need to know about
        liveness. Call .live() before .transform(), not after.

        Default buffer: KeepLast(). Subscribes to the backend BEFORE backfill
        starts and deduplicates by observation id.
        """
        if isinstance(self._source, Stream):
            raise TypeError(
                "Cannot call .live() on a transform stream. "
                "Call .live() on the source stream, then .transform()."
            )
        buf = buffer if buffer is not None else KeepLast()
        sub = self._source.subscribe(buf)
        return Stream(self._source, query=self._query, _live_buf=buf, _live_sub=sub)

    # ── Terminals ───────────────────────────────────────────────────

    def fetch(self) -> list[Observation[T]]:
        """Materialize all observations into a list."""
        return list(self)

    def first(self) -> Observation[T]:
        """Return the first matching observation."""
        it = iter(self.limit(1))
        try:
            return next(it)
        except StopIteration:
            raise LookupError("No matching observation") from None

    def last(self) -> Observation[T]:
        """Return the last matching observation (by timestamp)."""
        return self.order_by("ts", desc=True).first()

    def count(self) -> int:
        """Count matching observations."""
        if isinstance(self._source, Backend):
            return self._source.count(self._query)
        return sum(1 for _ in self)

    def exists(self) -> bool:
        """Check if any matching observation exists."""
        return next(iter(self.limit(1)), None) is not None

    # ── Write ───────────────────────────────────────────────────────

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
    ) -> Observation[T]:
        """Append to the backing store. Only works if source is a Backend."""
        if isinstance(self._source, Stream):
            raise TypeError("Cannot append to a transform stream. Append to the source stream.")
        return self._source.append(payload, ts=ts, pose=pose, tags=tags)
