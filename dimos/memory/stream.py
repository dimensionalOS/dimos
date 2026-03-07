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

from typing import (
    TYPE_CHECKING,
    Any,
    Generic,
    Protocol,
    TypeVar,
    cast,
    overload,
)

import numpy as np
import reactivex.operators as ops

from dimos.types.timestamped import Timestamped

from .types import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    EmbeddingObservation,
    EmbeddingSearchFilter,
    Filter,
    LineageFilter,
    NearFilter,
    Observation,
    StreamQuery,
    TagsFilter,
    TextSearchFilter,
    TimeRangeFilter,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from reactivex import Observable
    from reactivex.abc import DisposableBase as Disposable
    from reactivex.subject import Subject

    from dimos.models.embedding.base import Embedding, EmbeddingModel
    from dimos.msgs.geometry_msgs.Pose import PoseLike

    from .store import Session
    from .transformer import Transformer

T = TypeVar("T")
R = TypeVar("R")


class StreamBackend(Protocol):
    """Backend protocol — implemented by SqliteStreamBackend etc."""

    def execute_fetch(self, query: StreamQuery) -> list[Observation[Any]]: ...
    def execute_count(self, query: StreamQuery) -> int: ...
    def do_append(
        self,
        payload: Any,
        ts: float | None,
        pose: Any | None,
        tags: dict[str, Any] | None,
        parent_id: int | None = None,
    ) -> Observation[Any]: ...
    @property
    def appended_subject(self) -> Subject[Observation[Any]]: ...  # type: ignore[type-arg]
    @property
    def stream_name(self) -> str: ...


class Stream(Generic[T]):
    """Lazy, chainable stream over stored observations.

    Created by Session.stream(). Filter methods return new Stream instances.
    Terminals (.fetch(), .count(), etc.) execute the query.
    """

    def __init__(
        self,
        backend: StreamBackend | None = None,
        *,
        query: StreamQuery | None = None,
        session: Session | None = None,
        payload_type: type | None = None,
    ) -> None:
        self._backend = backend
        self._query = query or StreamQuery()
        self._session: Session | None = session
        self._payload_type: type | None = payload_type

    def _clone(self, **overrides: Any) -> Stream[T]:
        """Return a new Stream with updated query fields."""
        q = self._query
        new_query = StreamQuery(
            filters=overrides.get("filters", q.filters),
            order_field=overrides.get("order_field", q.order_field),
            order_desc=overrides.get("order_desc", q.order_desc),
            limit_val=overrides.get("limit_val", q.limit_val),
            offset_val=overrides.get("offset_val", q.offset_val),
        )
        clone: Stream[T] = self.__class__.__new__(self.__class__)
        clone._backend = self._backend
        clone._query = new_query
        clone._session = self._session
        clone._payload_type = self._payload_type
        return clone

    def __repr__(self) -> str:
        cls = type(self).__name__
        type_name = self._payload_type.__name__ if self._payload_type else "?"
        name = self._backend.stream_name if self._backend else "unbound"
        head = f'{cls}[{type_name}]("{name}")'
        query_str = str(self._query)
        return f"{head} | {query_str}" if query_str else head

    def _with_filter(self, f: Filter) -> Stream[T]:
        return self._clone(filters=(*self._query.filters, f))

    def _require_backend(self) -> StreamBackend:
        if self._backend is None:
            raise TypeError(
                "Operation requires a stored stream. Call .store() first or use session.stream()."
            )
        return self._backend

    # ── Write ─────────────────────────────────────────────────────────

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: PoseLike | None = None,
        tags: dict[str, Any] | None = None,
        parent_id: int | None = None,
    ) -> Observation[T]:
        if ts is None and isinstance(payload, Timestamped):
            ts = payload.ts
        backend = self._require_backend()
        return cast("Observation[T]", backend.do_append(payload, ts, pose, tags, parent_id))

    # ── Temporal filters ──────────────────────────────────────────────

    def after(self, t: float) -> Stream[T]:
        return self._with_filter(AfterFilter(t))

    def before(self, t: float) -> Stream[T]:
        return self._with_filter(BeforeFilter(t))

    def time_range(self, t1: float, t2: float) -> Stream[T]:
        return self._with_filter(TimeRangeFilter(t1, t2))

    def at(self, t: float, *, tolerance: float = 1.0) -> Stream[T]:
        return self._with_filter(AtFilter(t, tolerance))

    # ── Spatial filter ────────────────────────────────────────────────

    def near(self, pose: PoseLike, radius: float) -> Stream[T]:
        return self._with_filter(NearFilter(pose, radius))

    # ── Tag filter ────────────────────────────────────────────────────

    def filter_tags(self, **tags: Any) -> Stream[T]:
        return self._with_filter(TagsFilter(tuple(tags.items())))

    # ── Ordering / pagination ─────────────────────────────────────────

    def order_by(self, field: str, *, desc: bool = False) -> Stream[T]:
        return self._clone(order_field=field, order_desc=desc)

    def limit(self, k: int) -> Stream[T]:
        return self._clone(limit_val=k)

    def offset(self, n: int) -> Stream[T]:
        return self._clone(offset_val=n)

    # ── Transform ─────────────────────────────────────────────────────

    @overload
    def transform(
        self,
        xf: Transformer[T, R],
        *,
        live: bool = ...,
        backfill_only: bool = ...,
    ) -> Stream[R]: ...

    @overload
    def transform(
        self,
        xf: Callable[[T], Any],
        *,
        live: bool = ...,
        backfill_only: bool = ...,
    ) -> Stream[Any]: ...

    def transform(
        self,
        xf: Transformer[Any, Any] | Callable[..., Any],
        *,
        live: bool = False,
        backfill_only: bool = False,
    ) -> Stream[Any]:
        from .transformer import PerItemTransformer, Transformer as TransformerABC

        transformer: TransformerABC[Any, Any]
        if not isinstance(xf, TransformerABC):
            transformer = PerItemTransformer(xf)
        else:
            transformer = xf

        return TransformStream(
            source=self,
            transformer=transformer,
            live=live,
            backfill_only=backfill_only,
        )

    # ── Materialize ───────────────────────────────────────────────────

    def store(
        self,
        name: str | None = None,
        payload_type: type | None = None,
        session: Session | None = None,
    ) -> Stream[T]:
        # Already stored streams are a no-op
        if self._backend is not None and name is None:
            return self
        raise TypeError(
            "store() requires a session context. This stream is not associated with a session."
        )

    # ── Cross-stream lineage ──────────────────────────────────────────

    def project_to(self, target: Stream[R]) -> Stream[R]:
        """Follow parent_id lineage to project observations onto the target stream.

        Returns a filtered *target* Stream containing only observations that are
        ancestors of the current (source) query results.  The result is a normal
        Stream — all chaining, pagination, and lazy loading work as usual.
        """
        backend = self._require_backend()
        target_backend = target._require_backend()
        session = self._session
        if session is None:
            raise TypeError("project_to requires a session-backed stream")

        source_table = backend.stream_name
        target_table = target_backend.stream_name

        if source_table == target_table:
            return self  # type: ignore[return-value]

        hops = session.resolve_lineage_chain(source_table, target_table)

        return target._with_filter(
            LineageFilter(
                source_table=source_table,
                source_query=self._query,
                hops=hops,
            )
        )

    # ── Iteration ─────────────────────────────────────────────────────

    def __iter__(self) -> Iterator[Observation[T]]:
        for page in self.fetch_pages():
            yield from page

    # ── Terminals ─────────────────────────────────────────────────────

    def fetch(self) -> ObservationSet[T]:
        backend = self._require_backend()
        results = backend.execute_fetch(self._query)
        return ObservationSet(cast("list[Observation[T]]", results), session=self._session)

    def fetch_pages(self, batch_size: int = 128) -> Iterator[list[Observation[T]]]:
        offset = self._query.offset_val or 0
        total_limit = self._query.limit_val
        emitted = 0
        while True:
            page_size = batch_size
            if total_limit is not None:
                remaining = total_limit - emitted
                if remaining <= 0:
                    break
                page_size = min(batch_size, remaining)
            q = StreamQuery(
                filters=self._query.filters,
                order_field=self._query.order_field or "id",
                order_desc=self._query.order_desc,
                limit_val=page_size,
                offset_val=offset,
            )
            backend = self._require_backend()
            page = backend.execute_fetch(q)
            if not page:
                break
            yield cast("list[Observation[T]]", page)
            emitted += len(page)
            if len(page) < page_size:
                break
            offset += len(page)

    def one(self) -> Observation[T]:
        results = self.limit(1).fetch()
        if not results:
            raise LookupError("No matching observation")
        return results[0]

    def last(self) -> Observation[T]:
        results = self.order_by("ts", desc=True).limit(1).fetch()
        if not results:
            raise LookupError("No matching observation")
        return results[0]

    def count(self) -> int:
        backend = self._require_backend()
        return backend.execute_count(self._query)

    # ── Reactive ──────────────────────────────────────────────────────

    def observable(self) -> Observable[Observation[T]]:  # type: ignore[type-arg]
        backend = self._require_backend()
        raw: Observable[Observation[T]] = backend.appended_subject  # type: ignore[assignment]
        if not self._query.filters:
            return raw
        active = [
            f
            for f in self._query.filters
            if not isinstance(f, (EmbeddingSearchFilter, LineageFilter))
        ]

        def _check(o: Observation[T]) -> bool:
            return all(f.matches(o) for f in active)

        return raw.pipe(ops.filter(_check))

    def subscribe(self, on_next: Callable[[Observation[T]], None]) -> Disposable:
        return self.observable().subscribe(on_next=on_next)


class EmbeddingStream(Stream[T]):
    """Stream with a vector index. Adds search_embedding()."""

    _embedding_model: EmbeddingModel | None

    def __init__(
        self,
        backend: StreamBackend | None = None,
        *,
        query: StreamQuery | None = None,
        session: Session | None = None,
        embedding_model: EmbeddingModel | None = None,
        payload_type: type | None = None,
    ) -> None:
        super().__init__(backend=backend, query=query, session=session, payload_type=payload_type)
        self._embedding_model = embedding_model

    def _require_model(self) -> EmbeddingModel:
        if self._embedding_model is None:
            raise TypeError(
                "This embedding stream has no model reference. "
                "Pass a str/image only on streams created via EmbeddingTransformer, "
                "or search with a pre-computed Embedding / list[float]."
            )
        return self._embedding_model

    def _clone(self, **overrides: Any) -> Stream[T]:
        clone = super()._clone(**overrides)
        if isinstance(clone, EmbeddingStream):
            clone._embedding_model = self._embedding_model
        return clone

    def search_embedding(
        self,
        query: Embedding | list[float] | str | Any,
        *,
        k: int,
    ) -> EmbeddingStream[T]:
        """Search by vector similarity.

        Accepts pre-computed embeddings, raw float lists, text strings, or
        images/other objects.  Text and non-vector inputs are auto-embedded
        using the model that created this stream.

        Returns an EmbeddingStream — use ``.project_to(source)`` to get
        results in the source stream's type, or ``.fetch()`` for
        ``EmbeddingObservation`` with ``.similarity`` scores.
        """
        from dimos.models.embedding.base import Embedding as EmbeddingCls

        if isinstance(query, str):
            emb = self._require_model().embed_text(query)
            if isinstance(emb, list):
                emb = emb[0]
            return self.search_embedding(emb, k=k)

        if isinstance(query, EmbeddingCls):
            vec = query.to_numpy().tolist()
        elif isinstance(query, list):
            vec = list(query)
        else:
            # Assume embeddable object (Image, etc.)
            emb = self._require_model().embed(query)
            if isinstance(emb, list):
                emb = emb[0]
            return self.search_embedding(emb, k=k)

        clone = self._with_filter(EmbeddingSearchFilter(vec, k))
        return EmbeddingStream(
            backend=clone._backend,
            query=clone._query,
            session=clone._session,
            embedding_model=self._embedding_model,
            payload_type=clone._payload_type,
        )

    def fetch(self) -> ObservationSet[T]:  # type: ignore[override]
        backend = self._require_backend()
        results = backend.execute_fetch(self._query)
        return ObservationSet(cast("list[Observation[T]]", results), session=self._session)

    def one(self) -> EmbeddingObservation:  # type: ignore[override]
        results = self.limit(1).fetch()
        if not results:
            raise LookupError("No matching observation")
        return results[0]  # type: ignore[return-value]

    def last(self) -> EmbeddingObservation:  # type: ignore[override]
        results = self.order_by("ts", desc=True).limit(1).fetch()
        if not results:
            raise LookupError("No matching observation")
        return results[0]  # type: ignore[return-value]


class TextStream(Stream[T]):
    """Stream with an FTS5 index. Adds search_text()."""

    def search_text(self, text: str, *, k: int | None = None) -> TextStream[T]:
        clone = self._with_filter(TextSearchFilter(text, k))
        ts: TextStream[T] = TextStream(
            backend=clone._backend,
            query=clone._query,
            session=clone._session,
            payload_type=clone._payload_type,
        )
        return ts


class TransformStream(Stream[R]):
    """In-memory stream produced by .transform(). Not yet stored."""

    def __init__(
        self,
        source: Stream[Any],
        transformer: Transformer[Any, R],
        *,
        live: bool = False,
        backfill_only: bool = False,
    ) -> None:
        super().__init__(backend=None)
        self._source = source
        self._transformer = transformer
        self._live = live
        self._backfill_only = backfill_only

    def _clone(self, **overrides: Any) -> Stream[R]:
        clone = super()._clone(**overrides)
        if isinstance(clone, TransformStream):
            clone._source = self._source
            clone._transformer = self._transformer
            clone._live = self._live
            clone._backfill_only = self._backfill_only
        return clone

    def __repr__(self) -> str:
        type_name = self._transformer.output_type.__name__ if self._transformer.output_type else "?"
        xf_name = type(self._transformer).__name__
        flags: list[str] = []
        if self._live:
            flags.append("live=True")
        if self._backfill_only:
            flags.append("backfill_only=True")
        flag_str = ", " + ", ".join(flags) if flags else ""
        head = f"TransformStream[{type_name}]({self._source!r} -> {xf_name}{flag_str})"
        query_str = str(self._query)
        return f"{head} | {query_str}" if query_str else head

    def fetch(self) -> ObservationSet[R]:
        """Execute transform in memory, collecting results."""
        collector = _CollectorStream[R]()
        if self._transformer.supports_backfill and not self._live:
            self._transformer.process(self._source, collector)
        return ObservationSet(collector.results, session=self._source._session)

    def store(
        self,
        name: str | None = None,
        payload_type: type | None = None,
        session: Session | None = None,
    ) -> Stream[R]:
        resolved = session or self._source._session
        if resolved is None:
            raise TypeError(
                "Cannot store: no session available. "
                "Either use session.stream() to create the source, "
                "or pass session= to store()."
            )
        if name is None:
            raise TypeError("store() requires a name for transform outputs")
        resolved_type = payload_type or self._transformer.output_type
        return resolved.materialize_transform(
            name=name,
            source=self._source,
            transformer=self._transformer,
            payload_type=resolved_type,
            live=self._live,
            backfill_only=self._backfill_only,
        )


class _CollectorStream(Stream[R]):
    """Ephemeral stream that collects appended observations in a list."""

    def __init__(self) -> None:
        super().__init__(backend=None)
        self.results: list[Observation[R]] = []
        self._next_id = 0

    def append(
        self,
        payload: R,
        *,
        ts: float | None = None,
        pose: PoseLike | None = None,
        tags: dict[str, Any] | None = None,
        parent_id: int | None = None,
    ) -> Observation[R]:
        obs: Observation[R] = Observation(
            id=self._next_id,
            ts=ts,
            tags=tags or {},
            parent_id=parent_id,
            _data=payload,
        )
        self._next_id += 1
        self.results.append(obs)
        return obs


class ListBackend:
    """In-memory backend that evaluates StreamQuery filters in Python."""

    def __init__(self, observations: list[Observation[Any]], name: str = "<memory>") -> None:
        self._observations = observations
        self._name = name
        from reactivex.subject import Subject

        self._subject: Subject[Observation[Any]] = Subject()  # type: ignore[type-arg]

    def execute_fetch(self, query: StreamQuery) -> list[Observation[Any]]:
        results = list(self._observations)

        # Apply non-embedding filters
        for f in query.filters:
            if isinstance(f, (EmbeddingSearchFilter, LineageFilter)):
                continue
            results = [obs for obs in results if f.matches(obs)]

        # Embedding top-k pass (cosine similarity)
        emb_filters = [f for f in query.filters if isinstance(f, EmbeddingSearchFilter)]
        if emb_filters:
            ef = emb_filters[0]
            query_vec = np.array(ef.query, dtype=np.float32)
            query_norm = np.linalg.norm(query_vec)
            if query_norm > 0:
                scored = []
                for obs in results:
                    if isinstance(obs, EmbeddingObservation):
                        obs_vec = obs.embedding.to_numpy()
                    else:
                        continue
                    obs_norm = np.linalg.norm(obs_vec)
                    if obs_norm > 0:
                        sim = float(np.dot(query_vec, obs_vec) / (query_norm * obs_norm))
                    else:
                        sim = 0.0
                    scored.append((sim, obs))
                scored.sort(key=lambda x: x[0], reverse=True)
                results = [obs for _, obs in scored[: ef.k]]

        # Ordering
        if query.order_field:
            key = query.order_field
            results.sort(
                key=lambda obs: getattr(obs, key) if getattr(obs, key, None) is not None else 0,
                reverse=query.order_desc,
            )

        # Offset / limit
        if query.offset_val:
            results = results[query.offset_val :]
        if query.limit_val is not None:
            results = results[: query.limit_val]

        return results

    def execute_count(self, query: StreamQuery) -> int:
        return len(self.execute_fetch(query))

    def do_append(
        self,
        payload: Any,
        ts: float | None,
        pose: Any | None,
        tags: dict[str, Any] | None,
        parent_id: int | None = None,
    ) -> Observation[Any]:
        raise TypeError("ObservationSet is read-only")

    @property
    def appended_subject(self) -> Subject[Observation[Any]]:  # type: ignore[type-arg]
        return self._subject

    @property
    def stream_name(self) -> str:
        return self._name


class ObservationSet(Stream[T]):
    """Materialized result set — list-like + stream-like.

    Holds Observation objects with lazy _data_loader closures.
    Metadata is in memory, payload BLOBs stay in DB until .data access.
    """

    def __init__(
        self,
        observations: list[Observation[T]],
        *,
        session: Session | None = None,
    ) -> None:
        self._observations = observations
        backend = ListBackend(cast("list[Observation[Any]]", observations))
        super().__init__(backend=backend, session=session)

    def _clone(self, **overrides: Any) -> Stream[T]:
        """Return a plain Stream backed by same ListBackend (preserves lazy filter chaining)."""
        q = self._query
        new_query = StreamQuery(
            filters=overrides.get("filters", q.filters),
            order_field=overrides.get("order_field", q.order_field),
            order_desc=overrides.get("order_desc", q.order_desc),
            limit_val=overrides.get("limit_val", q.limit_val),
            offset_val=overrides.get("offset_val", q.offset_val),
        )
        return Stream(
            backend=self._backend,
            query=new_query,
            session=self._session,
            payload_type=self._payload_type,
        )

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: PoseLike | None = None,
        tags: dict[str, Any] | None = None,
        parent_id: int | None = None,
    ) -> Observation[T]:
        raise TypeError("ObservationSet is read-only")

    # ── List-like interface ──────────────────────────────────────────

    def __len__(self) -> int:
        return len(self._observations)

    @overload
    def __getitem__(self, index: int) -> Observation[T]: ...

    @overload
    def __getitem__(self, index: slice) -> list[Observation[T]]: ...

    def __getitem__(self, index: int | slice) -> Observation[T] | list[Observation[T]]:
        return self._observations[index]

    def __iter__(self) -> Iterator[Observation[T]]:
        return iter(self._observations)

    def __bool__(self) -> bool:
        return len(self._observations) > 0
