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

"""SQLite-backed memory store implementation.

Schema per stream ``{name}``:

    {name}          — id, ts, pose columns (x/y/z + quaternion), tags, parent_id
    {name}_payload  — id, data BLOB (loaded lazily)
    {name}_rtree    — R*Tree spatial index on position
    {name}_fts      — FTS5 full-text index (TextStream only)
    {name}_vec      — vec0 vector index (EmbeddingStream only)

Payloads use Codec (LCM for DimosMsg types, pickle otherwise).
Poses are decomposed into columns. Tags are JSON.
"""

from __future__ import annotations

import json
import re
import sqlite3
import time
from typing import TYPE_CHECKING, Any

from reactivex.subject import Subject

from dimos.memory.codec import (
    Codec,
    PickleCodec,
    codec_for_type,
    module_path_to_type,
    type_to_module_path,
)
from dimos.memory.store import Session, Store
from dimos.memory.stream import EmbeddingStream, Stream, TextStream
from dimos.memory.transformer import (
    CaptionTransformer,
    EmbeddingTransformer,
    TextEmbeddingTransformer,
    Transformer,
)
from dimos.memory.types import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    EmbeddingObservation,
    EmbeddingSearchFilter,
    Filter,
    LineageFilter,
    NearFilter,
    Observation,
    StreamInfo,
    StreamQuery,
    TagsFilter,
    TextSearchFilter,
    TimeRangeFilter,
)

if TYPE_CHECKING:
    from dimos.memory.types import PoseProvider
    from dimos.models.embedding.base import EmbeddingModel

_IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_ALLOWED_ORDER_FIELDS = frozenset({"id", "ts"})


def _validate_identifier(name: str) -> str:
    """Validate that *name* is a safe SQL identifier (alphanumeric + underscore)."""
    if not _IDENTIFIER_RE.match(name):
        raise ValueError(f"Invalid identifier: {name!r}")
    return name


# ── Pose helpers (column-based) ──────────────────────────────────────


def _decompose_pose(pose: Any) -> tuple[float, float, float, float, float, float, float] | None:
    """Extract (x, y, z, qx, qy, qz, qw) from a PoseStamped."""
    if pose is None:
        return None
    p = pose.position
    q = pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def _reconstruct_pose(
    x: float | None,
    y: float | None,
    z: float | None,
    qx: float | None,
    qy: float | None,
    qz: float | None,
    qw: float | None,
) -> Any | None:
    """Rebuild a PoseStamped from column values."""
    if x is None:
        return None
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    return PoseStamped(
        position=[x, y if y is not None else 0.0, z if z is not None else 0.0],
        orientation=[
            qx if qx is not None else 0.0,
            qy if qy is not None else 0.0,
            qz if qz is not None else 0.0,
            qw if qw is not None else 1.0,
        ],
    )


def _serialize_tags(tags: dict[str, Any] | None) -> str:
    if not tags:
        return "{}"
    return json.dumps(tags, separators=(",", ":"))


def _deserialize_tags(text: str) -> dict[str, Any]:
    if not text:
        return {}
    return json.loads(text)  # type: ignore[no-any-return]


# ── SQL building ──────────────────────────────────────────────────────

# Columns selected from the meta table (no payload).
_META_COLS = "id, ts, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, tags, parent_id"


def _compile_filter(f: Filter, table: str) -> tuple[str, list[Any]]:
    """Compile a single filter to (SQL fragment, params)."""
    if isinstance(f, AfterFilter):
        return f"{table}.ts > ?", [f.t]
    if isinstance(f, BeforeFilter):
        return f"{table}.ts < ?", [f.t]
    if isinstance(f, TimeRangeFilter):
        return f"{table}.ts >= ? AND {table}.ts <= ?", [f.t1, f.t2]
    if isinstance(f, AtFilter):
        return f"ABS({table}.ts - ?) <= ?", [f.t, f.tolerance]
    if isinstance(f, TagsFilter):
        clauses: list[str] = []
        params: list[Any] = []
        for key, val in f.tags:
            _validate_identifier(key)
            clauses.append(f"json_extract({table}.tags, '$.{key}') = ?")
            params.append(val)
        return " AND ".join(clauses), params
    if isinstance(f, NearFilter):
        # Handled via R*Tree JOIN — see _compile_query
        return "1=1", []
    if isinstance(f, EmbeddingSearchFilter):
        return "1=1", []
    if isinstance(f, TextSearchFilter):
        return "1=1", []
    if isinstance(f, LineageFilter):
        inner_sql, params = _compile_ids(f.source_query, f.source_table, select_col="parent_id")
        for hop in f.hops:
            inner_sql = f"SELECT parent_id FROM {hop} WHERE id IN ({inner_sql})"
        return f"{table}.id IN ({inner_sql})", params
    raise TypeError(f"Unknown filter type: {type(f)}")


def _compile_ids(
    query: StreamQuery, table: str, *, select_col: str = "id"
) -> tuple[str, list[Any]]:
    """Compile a StreamQuery to ``SELECT {col} FROM {table} WHERE ...``.

    Unlike ``_compile_query``, this handles *all* filter types as SQL — including
    EmbeddingSearchFilter and TextSearchFilter as inline subqueries — so that the
    result can be nested inside another query (used by LineageFilter).
    """
    where_parts: list[str] = []
    params: list[Any] = []
    joins: list[str] = []

    for f in query.filters:
        if isinstance(f, EmbeddingSearchFilter):
            where_parts.append(
                f"{table}.id IN (SELECT rowid FROM {table}_vec WHERE embedding MATCH ? AND k = ?)"
            )
            params.extend([json.dumps(f.query), f.k])
        elif isinstance(f, TextSearchFilter):
            fts_sub = f"SELECT rowid FROM {table}_fts WHERE content MATCH ?"
            fts_params: list[Any] = [f.text]
            if f.k is not None:
                fts_sub += " LIMIT ?"
                fts_params.append(f.k)
            where_parts.append(f"{table}.id IN ({fts_sub})")
            params.extend(fts_params)
        elif isinstance(f, NearFilter):
            joins.append(f"JOIN {table}_rtree AS r ON r.id = {table}.id")
            p = f.pose.position
            x, y, z = p.x, p.y, p.z
            where_parts.append(
                "r.min_x >= ? AND r.max_x <= ? AND "
                "r.min_y >= ? AND r.max_y <= ? AND "
                "r.min_z >= ? AND r.max_z <= ?"
            )
            params.extend(
                [x - f.radius, x + f.radius, y - f.radius, y + f.radius, z - f.radius, z + f.radius]
            )
        else:
            # Simple filters + LineageFilter → delegate to _compile_filter
            sql_frag, p = _compile_filter(f, table)
            where_parts.append(sql_frag)
            params.extend(p)

    where = " AND ".join(where_parts) if where_parts else "1=1"
    join_clause = " ".join(joins)

    sql = f"SELECT {table}.{select_col} FROM {table}"
    if join_clause:
        sql += f" {join_clause}"
    sql += f" WHERE {where}"

    if query.order_field:
        if query.order_field not in _ALLOWED_ORDER_FIELDS:
            raise ValueError(f"Invalid order field: {query.order_field!r}")
        sql += f" ORDER BY {query.order_field}"
        if query.order_desc:
            sql += " DESC"
    if query.limit_val is not None:
        sql += f" LIMIT {query.limit_val}"
    if query.offset_val is not None:
        sql += f" OFFSET {query.offset_val}"

    return sql, params


def _has_near_filter(query: StreamQuery) -> NearFilter | None:
    for f in query.filters:
        if isinstance(f, NearFilter):
            return f
    return None


def _compile_query(query: StreamQuery, table: str) -> tuple[str, list[Any]]:
    """Compile a StreamQuery to (SQL, params) for a metadata SELECT."""
    where_parts: list[str] = []
    params: list[Any] = []
    joins: list[str] = []

    for f in query.filters:
        if isinstance(f, NearFilter):
            # R*Tree bounding-box join
            joins.append(f"JOIN {table}_rtree AS r ON r.id = {table}.id")
            where_parts.append(
                "r.min_x >= ? AND r.max_x <= ? AND "
                "r.min_y >= ? AND r.max_y <= ? AND "
                "r.min_z >= ? AND r.max_z <= ?"
            )
            p = f.pose.position
            x, y, z = p.x, p.y, p.z
            params.extend(
                [
                    x - f.radius,
                    x + f.radius,
                    y - f.radius,
                    y + f.radius,
                    z - f.radius,
                    z + f.radius,
                ]
            )
        else:
            sql_frag, p = _compile_filter(f, table)
            where_parts.append(sql_frag)
            params.extend(p)

    where = " AND ".join(where_parts) if where_parts else "1=1"
    join_clause = " ".join(joins)

    if query.order_field:
        if query.order_field not in _ALLOWED_ORDER_FIELDS:
            raise ValueError(f"Invalid order field: {query.order_field!r}")
        order = f"ORDER BY {table}.{query.order_field}"
        if query.order_desc:
            order += " DESC"
    else:
        order = f"ORDER BY {table}.id"

    sql = f"SELECT {table}.{_META_COLS.replace(', ', f', {table}.')} FROM {table}"
    if join_clause:
        sql += f" {join_clause}"
    sql += f" WHERE {where} {order}"
    if query.limit_val is not None:
        sql += f" LIMIT {query.limit_val}"
    if query.offset_val is not None:
        sql += f" OFFSET {query.offset_val}"
    return sql, params


def _compile_count(query: StreamQuery, table: str) -> tuple[str, list[Any]]:
    where_parts: list[str] = []
    params: list[Any] = []
    joins: list[str] = []

    for f in query.filters:
        if isinstance(f, NearFilter):
            joins.append(f"JOIN {table}_rtree AS r ON r.id = {table}.id")
            p = f.pose.position
            x, y, z = p.x, p.y, p.z
            where_parts.append(
                "r.min_x >= ? AND r.max_x <= ? AND "
                "r.min_y >= ? AND r.max_y <= ? AND "
                "r.min_z >= ? AND r.max_z <= ?"
            )
            params.extend(
                [
                    x - f.radius,
                    x + f.radius,
                    y - f.radius,
                    y + f.radius,
                    z - f.radius,
                    z + f.radius,
                ]
            )
        else:
            sql_frag, p = _compile_filter(f, table)
            where_parts.append(sql_frag)
            params.extend(p)

    where = " AND ".join(where_parts) if where_parts else "1=1"
    join_clause = " ".join(joins)
    sql = f"SELECT COUNT(*) FROM {table}"
    if join_clause:
        sql += f" {join_clause}"
    sql += f" WHERE {where}"
    return sql, params


# ── Near-filter post-processing (exact distance after R*Tree bbox) ───


def _apply_near_post_filter(
    rows: list[Observation[Any]], near: NearFilter
) -> list[Observation[Any]]:
    """Post-filter R*Tree candidates by exact Euclidean distance."""
    tp = near.pose.position
    result: list[Observation[Any]] = []
    for obs in rows:
        if obs.pose is None:
            continue
        op = obs.pose.position
        dist = ((op.x - tp.x) ** 2 + (op.y - tp.y) ** 2 + (op.z - tp.z) ** 2) ** 0.5
        if dist <= near.radius:
            result.append(obs)
    return result


# ── Backend ───────────────────────────────────────────────────────────


class SqliteStreamBackend:
    """StreamBackend implementation for a single SQLite-backed stream."""

    def __init__(
        self,
        conn: sqlite3.Connection,
        table: str,
        *,
        pose_provider: PoseProvider | None = None,
        codec: Codec[Any] | None = None,
    ) -> None:
        _validate_identifier(table)
        self._conn = conn
        self._table = table
        self._pose_provider = pose_provider
        self._codec = codec or PickleCodec()
        self._subject: Subject[Observation[Any]] = Subject()  # type: ignore[type-arg]

    @property
    def appended_subject(self) -> Subject[Observation[Any]]:  # type: ignore[type-arg]
        return self._subject

    @property
    def stream_name(self) -> str:
        return self._table

    def _post_insert(self, row_id: int, payload: Any) -> None:
        """Hook for subclasses to add extra inserts inside the transaction."""

    def do_append(
        self,
        payload: Any,
        ts: float | None,
        pose: Any | None,
        tags: dict[str, Any] | None,
        parent_id: int | None = None,
    ) -> Observation[Any]:
        if ts is None:
            ts = time.time()
        if pose is None and self._pose_provider is not None:
            pose = self._pose_provider()

        pose_cols = _decompose_pose(pose)
        tags_json = _serialize_tags(tags)

        # Encode payload before touching the DB so a codec error can't leave
        # a metadata row without a matching payload row.
        payload_blob = self._codec.encode(payload)

        # 1. Insert into meta table
        if pose_cols is not None:
            cur = self._conn.execute(
                f"INSERT INTO {self._table} "
                "(ts, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, tags, parent_id) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (ts, *pose_cols, tags_json, parent_id),
            )
        else:
            cur = self._conn.execute(
                f"INSERT INTO {self._table} (ts, tags, parent_id) VALUES (?, ?, ?)",
                (ts, tags_json, parent_id),
            )
        row_id = cur.lastrowid
        assert row_id is not None

        # 2. Insert into payload table
        self._conn.execute(
            f"INSERT INTO {self._table}_payload (id, data) VALUES (?, ?)",
            (row_id, payload_blob),
        )

        # 3. Insert into R*Tree (if pose)
        if pose_cols is not None:
            x, y, z = pose_cols[0], pose_cols[1], pose_cols[2]
            self._conn.execute(
                f"INSERT INTO {self._table}_rtree (id, min_x, max_x, min_y, max_y, min_z, max_z) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (row_id, x, x, y, y, z, z),
            )

        # 4. Subclass hook (vec0, FTS, etc.)
        self._post_insert(row_id, payload)

        self._conn.commit()

        obs = Observation(
            id=row_id,
            ts=ts,
            pose=pose,
            tags=tags or {},
            parent_id=parent_id,
            _data=payload,
        )
        self._subject.on_next(obs)
        return obs

    def execute_fetch(self, query: StreamQuery) -> list[Observation[Any]]:
        sql, params = _compile_query(query, self._table)
        rows = self._conn.execute(sql, params).fetchall()
        observations = [self._row_to_obs(r) for r in rows]

        near = _has_near_filter(query)
        if near is not None:
            observations = _apply_near_post_filter(observations, near)

        return observations

    def execute_count(self, query: StreamQuery) -> int:
        sql, params = _compile_count(query, self._table)
        result = self._conn.execute(sql, params).fetchone()
        return result[0] if result else 0  # type: ignore[no-any-return]

    def _row_to_obs(self, row: Any) -> Observation[Any]:
        row_id, ts, px, py, pz, qx, qy, qz, qw, tags_json, pid = row
        pose = _reconstruct_pose(px, py, pz, qx, qy, qz, qw)
        conn = self._conn
        table = self._table
        codec = self._codec

        def loader() -> Any:
            r = conn.execute(f"SELECT data FROM {table}_payload WHERE id = ?", (row_id,)).fetchone()
            if r is None:
                raise LookupError(f"No payload for id={row_id}")
            return codec.decode(r[0])

        return Observation(
            id=row_id,
            ts=ts,
            pose=pose,
            tags=_deserialize_tags(tags_json),
            parent_id=pid,
            _data_loader=loader,
        )


class SqliteEmbeddingBackend(SqliteStreamBackend):
    """Backend for EmbeddingStream — stores vectors in a vec0 virtual table."""

    def __init__(
        self,
        conn: sqlite3.Connection,
        table: str,
        *,
        vec_dimensions: int | None = None,
        pose_provider: PoseProvider | None = None,
        parent_table: str | None = None,
        codec: Codec[Any] | None = None,
    ) -> None:
        super().__init__(conn, table, pose_provider=pose_provider, codec=codec)
        self._vec_dimensions = vec_dimensions
        self._parent_table = parent_table

    def _post_insert(self, row_id: int, payload: Any) -> None:
        from dimos.models.embedding.base import Embedding

        if isinstance(payload, Embedding):
            vec = payload.to_numpy().tolist()
            if self._vec_dimensions is None:
                self._vec_dimensions = len(vec)
                self._ensure_vec_table()
            self._conn.execute(
                f"INSERT INTO {self._table}_vec (rowid, embedding) VALUES (?, ?)",
                (row_id, json.dumps(vec)),
            )

    def _ensure_vec_table(self) -> None:
        if self._vec_dimensions is None:
            return
        self._conn.execute(
            f"CREATE VIRTUAL TABLE IF NOT EXISTS {self._table}_vec "
            f"USING vec0(embedding float[{self._vec_dimensions}] distance_metric=cosine)"
        )
        self._conn.commit()

    def execute_fetch(self, query: StreamQuery) -> list[Observation[Any]]:
        emb_filter = None
        for f in query.filters:
            if isinstance(f, EmbeddingSearchFilter):
                emb_filter = f
                break

        if emb_filter is not None:
            return self._fetch_by_vector(query, emb_filter)

        return super().execute_fetch(query)

    def execute_count(self, query: StreamQuery) -> int:
        if any(isinstance(f, EmbeddingSearchFilter) for f in query.filters):
            return len(self.execute_fetch(query))
        return super().execute_count(query)

    def _fetch_by_vector(
        self, query: StreamQuery, emb_filter: EmbeddingSearchFilter
    ) -> list[Observation[Any]]:
        """Fetch using vec0 similarity search, then apply remaining filters."""
        vec_sql = (
            f"SELECT rowid, distance FROM {self._table}_vec "
            f"WHERE embedding MATCH ? ORDER BY distance LIMIT ?"
        )
        vec_rows = self._conn.execute(
            vec_sql, (json.dumps(emb_filter.query), emb_filter.k)
        ).fetchall()

        if not vec_rows:
            return []

        dist_map = {r[0]: r[1] for r in vec_rows}
        rowids = list(dist_map.keys())
        placeholders = ",".join("?" * len(rowids))

        where_parts: list[str] = [f"{self._table}.id IN ({placeholders})"]
        params: list[Any] = list(rowids)

        for f in query.filters:
            if isinstance(f, EmbeddingSearchFilter):
                continue
            sql_frag, p = _compile_filter(f, self._table)
            where_parts.append(sql_frag)
            params.extend(p)

        where = " AND ".join(where_parts)
        sql = (
            f"SELECT {self._table}.{_META_COLS.replace(', ', f', {self._table}.')} "
            f"FROM {self._table} WHERE {where}"
        )
        rows = self._conn.execute(sql, params).fetchall()

        observations: list[Observation[Any]] = [self._row_to_obs(r) for r in rows]

        # Populate similarity scores from vec0 cosine distance (0=identical, 2=opposite)
        for obs in observations:
            if isinstance(obs, EmbeddingObservation):
                obs.similarity = max(0.0, min(1.0, 1.0 - dist_map.get(obs.id, 0.0)))

        # Re-sort by distance rank (IN clause doesn't preserve vec0 ordering)
        rank = {rid: i for i, rid in enumerate(rowids)}
        observations.sort(key=lambda o: rank.get(o.id, len(rank)))

        near = _has_near_filter(query)
        if near is not None:
            observations = _apply_near_post_filter(observations, near)

        return observations

    def _row_to_obs(self, row: Any) -> EmbeddingObservation:
        row_id, ts, px, py, pz, qx, qy, qz, qw, tags_json, pid = row
        pose = _reconstruct_pose(px, py, pz, qx, qy, qz, qw)
        conn = self._conn
        table = self._table
        codec = self._codec

        def loader() -> Any:
            r = conn.execute(f"SELECT data FROM {table}_payload WHERE id = ?", (row_id,)).fetchone()
            if r is None:
                raise LookupError(f"No payload for id={row_id}")
            return codec.decode(r[0])

        return EmbeddingObservation(
            id=row_id,
            ts=ts,
            pose=pose,
            tags=_deserialize_tags(tags_json),
            parent_id=pid,
            _data_loader=loader,
        )


class SqliteTextBackend(SqliteStreamBackend):
    """Backend for TextStream — maintains an FTS5 index."""

    def __init__(
        self,
        conn: sqlite3.Connection,
        table: str,
        *,
        tokenizer: str = "unicode61",
        pose_provider: PoseProvider | None = None,
        codec: Codec[Any] | None = None,
    ) -> None:
        super().__init__(conn, table, pose_provider=pose_provider, codec=codec)
        self._tokenizer = tokenizer

    def _post_insert(self, row_id: int, payload: Any) -> None:
        text = str(payload) if payload is not None else ""
        self._conn.execute(
            f"INSERT INTO {self._table}_fts (rowid, content) VALUES (?, ?)",
            (row_id, text),
        )

    def execute_fetch(self, query: StreamQuery) -> list[Observation[Any]]:
        text_filter = None
        for f in query.filters:
            if isinstance(f, TextSearchFilter):
                text_filter = f
                break

        if text_filter is not None:
            return self._fetch_by_text(query, text_filter)

        return super().execute_fetch(query)

    def _fetch_by_text(
        self, query: StreamQuery, text_filter: TextSearchFilter
    ) -> list[Observation[Any]]:
        fts_sql = f"SELECT rowid, rank FROM {self._table}_fts WHERE content MATCH ? ORDER BY rank"
        fts_params: list[Any] = [text_filter.text]
        if text_filter.k is not None:
            fts_sql += " LIMIT ?"
            fts_params.append(text_filter.k)

        fts_rows = self._conn.execute(fts_sql, fts_params).fetchall()
        if not fts_rows:
            return []

        rowids = [r[0] for r in fts_rows]
        placeholders = ",".join("?" * len(rowids))

        where_parts: list[str] = [f"{self._table}.id IN ({placeholders})"]
        params: list[Any] = list(rowids)

        for f in query.filters:
            if isinstance(f, TextSearchFilter):
                continue
            sql_frag, p = _compile_filter(f, self._table)
            where_parts.append(sql_frag)
            params.extend(p)

        where = " AND ".join(where_parts)
        sql = (
            f"SELECT {self._table}.{_META_COLS.replace(', ', f', {self._table}.')} "
            f"FROM {self._table} WHERE {where}"
        )
        rows = self._conn.execute(sql, params).fetchall()

        observations = [self._row_to_obs(r) for r in rows]

        # Re-sort by FTS rank (IN clause doesn't preserve FTS5 ordering)
        rank = {rid: i for i, rid in enumerate(rowids)}
        observations.sort(key=lambda o: rank.get(o.id, len(rank)))

        near = _has_near_filter(query)
        if near is not None:
            observations = _apply_near_post_filter(observations, near)

        return observations


# ── Session ───────────────────────────────────────────────────────────


class SqliteSession(Session):
    """Session against a SQLite database."""

    def __init__(self, conn: sqlite3.Connection) -> None:
        self._conn = conn
        self._streams: dict[str, Stream[Any]] = {}
        self._ensure_meta_table()

    def resolve_parent_stream(self, name: str) -> str | None:
        row = self._conn.execute(
            "SELECT parent_stream FROM _streams WHERE name = ?", (name,)
        ).fetchone()
        return row[0] if row and row[0] else None

    def resolve_lineage_chain(self, source: str, target: str) -> tuple[str, ...]:
        """Walk ``_streams.parent_stream`` from *source* toward *target*.

        Returns intermediate table names (empty tuple for direct parent).
        """
        current = source
        intermediates: list[str] = []
        visited = {source}

        while True:
            row = self._conn.execute(
                "SELECT parent_stream FROM _streams WHERE name = ?", (current,)
            ).fetchone()
            if not row or not row[0]:
                raise ValueError(f"No lineage path from {source!r} to {target!r}")

            parent_name: str = row[0]
            if parent_name == target:
                return tuple(intermediates)

            if parent_name in visited:
                raise ValueError(f"Cycle detected in lineage chain at {parent_name!r}")

            visited.add(parent_name)
            intermediates.append(parent_name)
            current = parent_name

    def _ensure_meta_table(self) -> None:
        self._conn.execute(
            "CREATE TABLE IF NOT EXISTS _streams ("
            "  name TEXT PRIMARY KEY,"
            "  payload_module TEXT,"
            "  stream_kind TEXT DEFAULT 'stream',"
            "  parent_stream TEXT,"
            "  embedding_dim INTEGER"
            ")"
        )
        self._conn.commit()

    def stream(
        self,
        name: str,
        payload_type: type,
        *,
        pose_provider: PoseProvider | None = None,
    ) -> Stream[Any]:
        _validate_identifier(name)
        if name in self._streams:
            return self._streams[name]

        self._ensure_stream_tables(name)
        self._register_stream(name, payload_type, "stream")

        codec = codec_for_type(payload_type)
        backend = SqliteStreamBackend(self._conn, name, pose_provider=pose_provider, codec=codec)
        s: Stream[Any] = Stream(backend=backend, session=self, payload_type=payload_type)
        self._streams[name] = s
        return s

    def text_stream(
        self,
        name: str,
        *,
        tokenizer: str = "unicode61",
        pose_provider: PoseProvider | None = None,
    ) -> TextStream[Any]:
        _validate_identifier(name)
        if name in self._streams:
            return self._streams[name]  # type: ignore[return-value]

        self._ensure_stream_tables(name)
        self._ensure_fts_table(name, tokenizer)
        self._register_stream(name, str, "text")

        codec = codec_for_type(str)
        backend = SqliteTextBackend(
            self._conn, name, tokenizer=tokenizer, pose_provider=pose_provider, codec=codec
        )
        ts: TextStream[Any] = TextStream(backend=backend, session=self, payload_type=str)
        self._streams[name] = ts
        return ts

    def embedding_stream(
        self,
        name: str,
        *,
        vec_dimensions: int | None = None,
        pose_provider: PoseProvider | None = None,
        parent_table: str | None = None,
        embedding_model: EmbeddingModel | None = None,
    ) -> EmbeddingStream[Any]:
        from dimos.models.embedding.base import Embedding

        _validate_identifier(name)
        if name in self._streams:
            existing = self._streams[name]
            if embedding_model is not None and isinstance(existing, EmbeddingStream):
                existing._embedding_model = embedding_model
            return existing  # type: ignore[return-value]

        self._ensure_stream_tables(name)
        self._register_stream(name, Embedding, "embedding", embedding_dim=vec_dimensions)

        codec = codec_for_type(Embedding)
        backend = SqliteEmbeddingBackend(
            self._conn,
            name,
            vec_dimensions=vec_dimensions,
            pose_provider=pose_provider,
            parent_table=parent_table,
            codec=codec,
        )
        if vec_dimensions is not None:
            backend._ensure_vec_table()

        es: EmbeddingStream[Any] = EmbeddingStream(
            backend=backend,
            session=self,
            embedding_model=embedding_model,
            payload_type=Embedding,
        )
        self._streams[name] = es
        return es

    def list_streams(self) -> list[StreamInfo]:
        rows = self._conn.execute(
            "SELECT name, payload_module, stream_kind FROM _streams"
        ).fetchall()
        result: list[StreamInfo] = []
        for name, pmodule, kind in rows:
            _validate_identifier(name)
            count_row = self._conn.execute(f"SELECT COUNT(*) FROM {name}").fetchone()
            count = count_row[0] if count_row else 0
            result.append(
                StreamInfo(
                    name=name, payload_type=pmodule, count=count, stream_kind=kind or "stream"
                )
            )
        return result

    def materialize_transform(
        self,
        name: str,
        source: Stream[Any],
        transformer: Transformer[Any, Any],
        *,
        payload_type: type | None = None,
        live: bool = False,
        backfill_only: bool = False,
    ) -> Stream[Any]:
        # Resolve source table name for parent lineage
        source_table = None
        if source._backend is not None:
            source_table = source._backend.stream_name

        target: Stream[Any]
        if isinstance(transformer, (EmbeddingTransformer, TextEmbeddingTransformer)):
            target = self.embedding_stream(name, parent_table=source_table)
            target._embedding_model = transformer.model
        elif isinstance(transformer, CaptionTransformer):
            target = self.text_stream(name)
        else:
            if payload_type is None:
                raise TypeError("materialize_transform requires payload_type for plain streams")
            target = self.stream(name, payload_type)

        # Record parent lineage in _streams registry
        if source_table is not None:
            self._conn.execute(
                "UPDATE _streams SET parent_stream = ? WHERE name = ?",
                (source_table, name),
            )
            self._conn.commit()

        # Backfill existing data
        if transformer.supports_backfill and not live:
            transformer.process(source, target)

        # Subscribe to live updates
        if transformer.supports_live and not backfill_only:
            source.subscribe(lambda obs: transformer.on_append(obs, target))

        return target

    def stop(self) -> None:
        for s in self._streams.values():
            if s._backend is not None:
                s._backend.appended_subject.on_completed()
        self._streams.clear()
        self._conn.close()

    # ── Internal helpers ──────────────────────────────────────────────

    def _ensure_stream_tables(self, name: str) -> None:
        """Create the meta table, payload table, and R*Tree for a stream."""
        self._conn.execute(
            f"CREATE TABLE IF NOT EXISTS {name} ("
            "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "  ts REAL,"
            "  pose_x REAL,"
            "  pose_y REAL,"
            "  pose_z REAL,"
            "  pose_qx REAL,"
            "  pose_qy REAL,"
            "  pose_qz REAL,"
            "  pose_qw REAL,"
            "  tags TEXT DEFAULT '{}',"
            "  parent_id INTEGER"
            ")"
        )
        self._conn.execute(f"CREATE INDEX IF NOT EXISTS idx_{name}_ts ON {name}(ts)")
        self._conn.execute(
            f"CREATE TABLE IF NOT EXISTS {name}_payload (  id INTEGER PRIMARY KEY,  data BLOB)"
        )
        self._conn.execute(
            f"CREATE VIRTUAL TABLE IF NOT EXISTS {name}_rtree USING rtree("
            "  id,"
            "  min_x, max_x,"
            "  min_y, max_y,"
            "  min_z, max_z"
            ")"
        )
        self._conn.commit()

    def _ensure_fts_table(self, name: str, tokenizer: str) -> None:
        self._conn.execute(
            f"CREATE VIRTUAL TABLE IF NOT EXISTS {name}_fts "
            f"USING fts5(content, tokenize='{tokenizer}')"
        )
        self._conn.commit()

    def _register_stream(
        self,
        name: str,
        payload_type: type | None,
        kind: str,
        *,
        embedding_dim: int | None = None,
    ) -> None:
        module_path = type_to_module_path(payload_type) if payload_type else None
        self._conn.execute(
            "INSERT OR IGNORE INTO _streams (name, payload_module, stream_kind, embedding_dim) "
            "VALUES (?, ?, ?, ?)",
            (name, module_path, kind, embedding_dim),
        )
        self._conn.commit()

    def _resolve_payload_type(self, name: str) -> type | None:
        """Look up payload type from _streams metadata (for restart case)."""
        row = self._conn.execute(
            "SELECT payload_module FROM _streams WHERE name = ?", (name,)
        ).fetchone()
        if row is None or row[0] is None:
            return None
        return module_path_to_type(row[0])


# ── Store ─────────────────────────────────────────────────────────────


class SqliteStore(Store):
    """SQLite-backed memory store (lightweight factory).

    Each :meth:`session` call opens a new ``sqlite3.Connection`` with WAL mode
    and extensions loaded.  Sessions are safe to use from different threads.
    """

    def __init__(self, path: str) -> None:
        self._path = path
        self._closed = False

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        self._load_extensions(conn)
        return conn

    def session(self) -> SqliteSession:
        if self._closed:
            raise RuntimeError("Store is closed")
        return SqliteSession(self._connect())

    def _load_extensions(self, conn: sqlite3.Connection) -> None:
        try:
            import sqlite_vec

            conn.enable_load_extension(True)
            sqlite_vec.load(conn)
            conn.enable_load_extension(False)
        except ImportError:
            pass

    def stop(self) -> None:
        self._closed = True
