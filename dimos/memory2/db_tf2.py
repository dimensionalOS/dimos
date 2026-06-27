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

"""Graph-stream transform lookups (the multi-robot-friendly successor to DbTf).

Two pieces of recorded state:

* a **graph stream** (table ``tf_graph``): one row per *topology* change — i.e.
  whenever the set of frames or any frame's parent / static-ness changes. Each row
  is the full structure at that instant: ``{child_frame: {parent, static}}``.
  Topology changes rarely (a robot joins/leaves, a relocalization re-parents), so
  this table is tiny.
* the existing ``tf`` stream, with each row tagged by its ``child_frame`` (an
  indexed json tag) so a frame's samples can be range-queried by time.

``GraphTf.get`` then: gets the graph as-of the query time (from RAM if there are
few graph changes, else one query), walks it to the source->target chain (in
memory; the graph may be DISJOINT for unrelated robots), and resolves *only* the
chain's frames — a static frame is one cached constant, a dynamic frame is its two
samples bracketing the time, interpolated. Composition + interpolation reuse
:class:`MultiTBuffer`.
"""

from __future__ import annotations

import bisect
import json
import sqlite3
from typing import TYPE_CHECKING, Any, cast

from dimos.memory2.db_tf import _connect, _safe_table
from dimos.memory2.store.sqlite import SqliteStoreConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store

logger = setup_logger()

DEFAULT_TF_STREAM = "tf"
GRAPH_TABLE = "tf_graph"
# If a recording has fewer than this many topology changes, load them all into RAM
# so a lookup needs no graph query (the common single-robot / stable-tree case).
# At or above it, fall back to one graph query per lookup (many-robot churn).
DEFAULT_MAX_GRAPH_CHANGES_IN_RAM = 20
_NO_PRUNE = 1.0e15


def _graph_table(stream: str) -> str:
    return f"{_safe_table(stream)}_graph"


def ensure_graph_table(conn: sqlite3.Connection, stream: str) -> None:
    """Create the topology change-log table (safe to call before the tf table
    exists — it doesn't touch the tf table)."""
    table = _graph_table(stream)
    conn.execute(
        f'CREATE TABLE IF NOT EXISTS "{table}" '
        "(id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, structure TEXT NOT NULL)"
    )
    conn.execute(f'CREATE INDEX IF NOT EXISTS "{table}_ts_idx" ON "{table}"(ts)')
    conn.commit()


def _ensure_child_index(conn: sqlite3.Connection, stream: str) -> None:
    """Index the child_frame json tag on the tf rows so per-frame time queries
    seek. The live recorder gets this for free (the store auto-indexes tag keys on
    tagged appends); this is for migrated recordings and the read side. Requires
    the tf table to exist."""
    safe = _safe_table(stream)
    conn.execute(
        f'CREATE INDEX IF NOT EXISTS "{safe}_child_idx" '
        f"ON \"{safe}\"(json_extract(tags, '$.child_frame'))"
    )
    conn.commit()


class TfGraphWriter:
    """Recorder helper: tracks the running topology and appends a ``tf_graph`` row
    only when the structure changes."""

    def __init__(self, db_path: str, stream: str = DEFAULT_TF_STREAM) -> None:
        self._stream = _safe_table(stream)
        self._table = _graph_table(stream)
        self._conn = _connect(db_path)
        self._structure: dict[str, dict[str, Any]] = {}
        ensure_graph_table(self._conn, self._stream)

    def record(self, child_frame: str, parent_frame: str, is_static: bool, ts: float) -> None:
        entry = {"parent": parent_frame, "static": bool(is_static)}
        if self._structure.get(child_frame) == entry:
            return  # no structural change -> no new graph row
        self._structure[child_frame] = entry
        self._conn.execute(
            f'INSERT INTO "{self._table}" (ts, structure) VALUES (?, ?)',
            (ts, json.dumps(self._structure)),
        )
        self._conn.commit()

    def close(self) -> None:
        self._conn.close()


def build_graph_stream(store: Store, stream: str = DEFAULT_TF_STREAM) -> int:
    """One-time migration for a recording that predates the graph stream: tag every
    tf row with its ``child_frame`` and build ``tf_graph`` chronologically. A frame
    is treated as static if its pose never changes across the recording. Returns the
    number of topology-change rows written."""
    config = store.config
    if not isinstance(config, SqliteStoreConfig):
        raise TypeError("build_graph_stream needs a SqliteStore")
    table = _graph_table(stream)
    safe = _safe_table(stream)

    # one decode pass: collect (id, ts, child, parent, pose-key) per row
    rows: list[tuple[int, float, str, str, tuple[float, ...]]] = []
    poses_per_child: dict[str, set[tuple[float, ...]]] = {}
    for obs in store.stream(safe, TFMessage).order_by("ts"):
        for transform in getattr(obs.data, "transforms", None) or [obs.data]:
            pose_key = (
                round(transform.translation.x, 9),
                round(transform.translation.y, 9),
                round(transform.translation.z, 9),
                round(transform.rotation.x, 9),
                round(transform.rotation.y, 9),
                round(transform.rotation.z, 9),
                round(transform.rotation.w, 9),
            )
            rows.append((obs.id, obs.ts, transform.child_frame_id, transform.frame_id, pose_key))
            poses_per_child.setdefault(transform.child_frame_id, set()).add(pose_key)
    static_frames = {child for child, poses in poses_per_child.items() if len(poses) == 1}

    conn = _connect(config.path)
    try:
        ensure_graph_table(conn, safe)
        conn.execute(f'DELETE FROM "{table}"')
        # tag each tf row with its child_frame (json_set keeps any existing tags)
        for row_id, _ts, child, _parent, _pose in rows:
            conn.execute(
                f"UPDATE \"{safe}\" SET tags = json_set(tags, '$.child_frame', ?) WHERE id = ?",
                (child, row_id),
            )
        _ensure_child_index(conn, safe)
        # build the topology change-log
        structure: dict[str, dict[str, Any]] = {}
        written = 0
        for _row_id, ts, child, parent, _pose in rows:
            entry = {"parent": parent, "static": child in static_frames}
            if structure.get(child) == entry:
                continue
            structure[child] = entry
            conn.execute(
                f'INSERT INTO "{table}" (ts, structure) VALUES (?, ?)', (ts, json.dumps(structure))
            )
            written += 1
        conn.commit()
        return written
    finally:
        conn.close()


class DbTf2:
    """Graph-stream transform lookups with an in-RAM graph cache for the common case.

    Drop-in successor to :class:`dimos.memory2.db_tf.DbTf` — same
    ``get(target, source, time_point, time_tolerance)`` / ``has_transforms()``
    surface, so the store's ``tf`` property can swap to it in one line."""

    def __init__(
        self,
        store: Store,
        stream: str = DEFAULT_TF_STREAM,
        max_graph_changes_in_ram: int = DEFAULT_MAX_GRAPH_CHANGES_IN_RAM,
    ) -> None:
        self._store = store
        self._stream = _safe_table(stream)
        self._table = _graph_table(stream)
        self._max_in_ram = max_graph_changes_in_ram
        self._conn: sqlite3.Connection | None = None
        self._built = False
        # graph cache: either the whole change-log in RAM, or None (query per lookup)
        self._graph_in_ram: list[tuple[float, dict[str, Any]]] | None = None
        self._graph_loaded = False
        self._static_cache: dict[str, Transform] = {}
        self.rows_fetched = 0
        self.graph_queries = 0

    def _connection(self) -> sqlite3.Connection:
        conn = self._conn
        if conn is None:
            config = self._store.config
            assert isinstance(config, SqliteStoreConfig)
            conn = _connect(config.path)
            self._conn = conn
        return conn

    def has_transforms(self) -> bool:
        conn = self._connection()
        if self._stream not in set(self._store.list_streams()):
            return False
        (n_rows,) = conn.execute(f'SELECT count(*) FROM "{self._stream}"').fetchone()
        return bool(n_rows)

    def _ensure_built(self) -> None:
        if self._built:
            return
        conn = self._connection()
        ensure_graph_table(conn, self._stream)
        (n_graph,) = conn.execute(f'SELECT count(*) FROM "{self._table}"').fetchone()
        (n_rows,) = conn.execute(f'SELECT count(*) FROM "{self._stream}"').fetchone()
        if n_rows and n_graph == 0:
            logger.warning(
                "\n========================================================================\n"
                "  tf graph stream MISSING for %r. Building it (one-time): tagging tf rows\n"
                "  with child_frame and writing the topology change-log.\n"
                "========================================================================",
                self._stream,
            )
            built = build_graph_stream(self._store, self._stream)
            logger.warning("tf graph built: %d topology changes for %r.", built, self._stream)
        if n_rows:
            _ensure_child_index(conn, self._stream)  # tf table exists now
        self._built = True

    def _load_graph_if_small(self) -> None:
        if self._graph_loaded:
            return
        conn = self._connection()
        (n_graph,) = conn.execute(f'SELECT count(*) FROM "{self._table}"').fetchone()
        if n_graph < self._max_in_ram:
            self._graph_in_ram = [
                (ts, json.loads(structure))
                for ts, structure in conn.execute(
                    f'SELECT ts, structure FROM "{self._table}" ORDER BY ts ASC'
                )
            ]
        else:
            self._graph_in_ram = None  # too many -> query per lookup
        self._graph_loaded = True

    def _graph_at(self, query_time: float) -> dict[str, Any] | None:
        if self._graph_in_ram is not None:
            # in-RAM: binary search the latest change at-or-before query_time
            stamps = [ts for ts, _ in self._graph_in_ram]
            index = bisect.bisect_right(stamps, query_time) - 1
            if index < 0:
                return self._graph_in_ram[0][1]  # before first -> earliest
            return self._graph_in_ram[index][1]
        # fallback: one query
        self.graph_queries += 1
        conn = self._connection()
        row = conn.execute(
            f'SELECT structure FROM "{self._table}" WHERE ts <= ? ORDER BY ts DESC LIMIT 1',
            (query_time,),
        ).fetchone()
        if row is None:
            row = conn.execute(
                f'SELECT structure FROM "{self._table}" ORDER BY ts ASC LIMIT 1'
            ).fetchone()
        return json.loads(row[0]) if row else None

    def _chain_frames(self, graph: dict[str, Any], source: str, target: str) -> list[str] | None:
        def to_root(frame: str) -> list[str]:
            path = [frame]
            seen = {frame}
            while (
                frame in graph
                and graph[frame].get("parent") in graph
                and graph[frame]["parent"] not in seen
            ):
                frame = graph[frame]["parent"]
                path.append(frame)
                seen.add(frame)
            # include a final parent that is itself a root (not a key in graph)
            if frame in graph and graph[frame].get("parent") and graph[frame]["parent"] not in seen:
                path.append(graph[frame]["parent"])
            return path

        source_path = to_root(source)
        target_path = to_root(target)
        common = next((f for f in source_path if f in set(target_path)), None)
        if common is None:
            return None  # disjoint graph: no transform between them
        frames = source_path[: source_path.index(common) + 1]
        frames += target_path[: target_path.index(common)]
        return frames

    def _static_transform(self, frame: str) -> Transform | None:
        if frame in self._static_cache:
            return self._static_cache[frame]
        conn = self._connection()
        row = conn.execute(
            f"SELECT id FROM \"{self._stream}\" WHERE json_extract(tags, '$.child_frame') = ? "
            "ORDER BY ts DESC LIMIT 1",
            (frame,),
        ).fetchone()
        if row is None:
            return None
        transform = self._decode(int(row[0]), frame)
        self._static_cache[frame] = transform
        return transform

    def _bracket_rows(self, frame: str, query_time: float) -> tuple[int | None, int | None]:
        conn = self._connection()
        earlier = conn.execute(
            f"SELECT id FROM \"{self._stream}\" WHERE json_extract(tags, '$.child_frame') = ? "
            "AND ts <= ? ORDER BY ts DESC LIMIT 1",
            (frame, query_time),
        ).fetchone()
        later = conn.execute(
            f"SELECT id FROM \"{self._stream}\" WHERE json_extract(tags, '$.child_frame') = ? "
            "AND ts >= ? ORDER BY ts ASC LIMIT 1",
            (frame, query_time),
        ).fetchone()
        return (int(earlier[0]) if earlier else None, int(later[0]) if later else None)

    def _decode(self, row_id: int, frame: str) -> Transform:
        # A row normally holds one transform (the live recorder appends one per row);
        # pick the one for `frame` defensively in case a legacy row packs several.
        backend: Any = self._store.stream(self._stream, TFMessage)._source
        self.rows_fetched += 1
        message = backend._make_loader(row_id)()
        transforms = getattr(message, "transforms", None) or [message]
        for transform in transforms:
            if transform.child_frame_id == frame:
                return cast("Transform", transform)
        return cast("Transform", transforms[0])

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        self._ensure_built()
        self._load_graph_if_small()
        query_time = time_point if time_point is not None else 0.0
        graph = self._graph_at(query_time)
        if graph is None:
            return None
        frames = self._chain_frames(graph, source_frame, target_frame)
        if frames is None:
            return None

        buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
        for frame in frames:
            if frame not in graph:
                continue  # a root frame has no incoming edge
            if graph[frame].get("static"):
                transform = self._static_transform(frame)
                if transform is None:
                    return None
                # restamp the constant to the query time so the buffer's tolerance
                # (statics may have been recorded long ago) never rejects it.
                buffer.receive_transform(_restamp(transform, query_time))
            else:
                earlier_id, later_id = self._bracket_rows(frame, query_time)
                if earlier_id is None and later_id is None:
                    return None
                lo = earlier_id if earlier_id is not None else later_id
                hi = later_id if later_id is not None else earlier_id
                assert lo is not None and hi is not None  # at least one existed
                buffer.receive_transform(self._decode(lo, frame))
                if hi != lo:
                    buffer.receive_transform(self._decode(hi, frame))
        return buffer.lookup(target_frame, source_frame, time_point, time_tolerance)


def _restamp(transform: Transform, ts: float) -> Transform:
    return Transform(
        translation=transform.translation,
        rotation=transform.rotation,
        frame_id=transform.frame_id,
        child_frame_id=transform.child_frame_id,
        ts=ts,
    )
