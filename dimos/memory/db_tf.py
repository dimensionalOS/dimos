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

"""
A tf class for memory2
"""

from __future__ import annotations

import bisect
import re
import sqlite3
import threading
from typing import TYPE_CHECKING, Any, cast

from dimos.memory.store.sqlite import SqliteStoreConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

logger = setup_logger()

DEFAULT_TF_STREAM = "tf"
# The topology change-log is a single companion stream (like tf_static).
GRAPH_STREAM = "tf_graph"
# Streams the RAM fallback (non-sqlite stores) reads.
TF_STREAMS = ("tf", "tf_static")
# Cache the whole change-log in RAM when there are at most this many topology
# changes (a stable tree — even a many-frame sensor rig — is a one-time setup, not
# churn); above it, fall back to one indexed graph query per lookup (multi-robot).
DEFAULT_MAX_GRAPH_CHANGES_IN_RAM = 64
# MultiTBuffer drops samples older than buffer_size seconds; we feed it exactly the
# bracketing samples and want them all kept, so use a span no recording exceeds.
_NO_PRUNE = 1.0e15
# A frame is "static" if its pose never changes; poses are compared rounded to this
# many decimals (~nanometre / nanoradian) so float noise doesn't read as motion.
POSE_EQUALITY_DECIMALS = 9
# enforce safe identifiers for SQL
_VAR_NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class DbTf:
    """Transform lookups backed by a store's recorded transforms.

    On a SQLite store this uses the graph stream + an in-RAM graph cache; other
    stores fall back to loading the tf streams into a :class:`MultiTBuffer`. Surface
    is ``get(target, source, time_point, time_tolerance)`` / ``has_transforms()``.
    """

    def __init__(
        self,
        store: Store,
        stream: str = DEFAULT_TF_STREAM,
        max_graph_changes_in_ram: int = DEFAULT_MAX_GRAPH_CHANGES_IN_RAM,
        stream_names: tuple[str, ...] = TF_STREAMS,
    ) -> None:
        self._store = store
        self._stream = _safe_table(stream)
        self._max_in_ram = max_graph_changes_in_ram
        self._stream_names = stream_names  # RAM fallback only
        self._lock = threading.Lock()
        self._conn: sqlite3.Connection | None = None
        self._built = False
        # graph cache: either the whole change-log in RAM, or None (query per lookup)
        self._graph_in_ram: list[tuple[float, dict[str, Any]]] | None = None
        self._graph_loaded = False
        self._static_cache: dict[str, Transform] = {}
        self._buffer: MultiTBuffer | None = None  # RAM fallback (non-sqlite)
        self.rows_fetched = 0
        self.graph_queries = 0

    @property
    def _is_sqlite(self) -> bool:
        return isinstance(self._store.config, SqliteStoreConfig)

    def _connection(self) -> sqlite3.Connection:
        conn = self._conn
        if conn is None:
            config = self._store.config
            assert isinstance(config, SqliteStoreConfig)  # guarded by _is_sqlite
            conn = _connect(config.path)
            self._conn = conn
        return conn

    # --- RAM fallback (non-sqlite stores) --------------------------------------

    def _ensure_loaded(self) -> MultiTBuffer:
        if self._buffer is not None:
            return self._buffer
        with self._lock:
            if self._buffer is not None:
                return self._buffer
            buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
            available = set(self._store.list_streams())
            for name in self._stream_names:
                if name not in available:
                    continue
                for observation in self._store.stream(name, TFMessage):
                    transforms = getattr(observation.data, "transforms", None) or [observation.data]
                    buffer.receive_transform(*transforms)
            self._buffer = buffer
            return buffer

    # --- graph stream (sqlite) -------------------------------------------------

    def has_transforms(self) -> bool:
        if not self._is_sqlite:
            return bool(self._ensure_loaded().buffers)
        conn = self._connection()
        if self._stream not in set(self._store.list_streams()):
            return False
        (n_rows,) = conn.execute(f'SELECT count(*) FROM "{self._stream}"').fetchone()
        return bool(n_rows)

    def _graph_stream(self) -> Stream[TfGraph]:
        return self._store.stream(GRAPH_STREAM, TfGraph)

    def _graph_count(self) -> int:
        if GRAPH_STREAM not in set(self._store.list_streams()):
            return 0
        return self._graph_stream().count()

    def _ensure_built(self) -> None:
        """First sqlite use: if the recording has tf rows but no graph stream (a
        recording that predates it / wasn't written by the recorder), build the graph
        stream once by replaying the tf rows, then make sure the seek index exists."""
        if self._built:
            return
        conn = self._connection()
        (n_rows,) = conn.execute(f'SELECT count(*) FROM "{self._stream}"').fetchone()
        if n_rows and self._graph_count() == 0:
            logger.warning(
                "\n========================================================================\n"
                "  tf graph stream MISSING for %r. Building it (one-time): tagging tf rows\n"
                "  with child_frame and writing the topology change-log.\n"
                "========================================================================",
                self._stream,
            )
            self._build_graph_stream()
        if n_rows:
            _ensure_child_index(conn, self._stream)  # tf table exists now
        self._built = True

    def _build_graph_stream(self) -> None:
        """One-time migration: decode every tf row, tag it with its child_frame, and
        append a ``TfGraph`` snapshot whenever the topology changes. A frame counts as
        static if its pose never varies across the whole recording."""
        safe = self._stream
        # one decode pass: collect (id, ts, child, parent, pose-key) + per-child poses
        rows: list[tuple[int, float, str, str]] = []
        poses_per_child: dict[str, set[tuple[float, ...]]] = {}
        for obs in self._store.stream(safe, TFMessage).order_by("ts"):
            for transform in getattr(obs.data, "transforms", None) or [obs.data]:
                pose_key = tuple(
                    round(value, POSE_EQUALITY_DECIMALS)
                    for value in (
                        transform.translation.x,
                        transform.translation.y,
                        transform.translation.z,
                        transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w,
                    )
                )
                rows.append((obs.id, obs.ts, transform.child_frame_id, transform.frame_id))
                poses_per_child.setdefault(transform.child_frame_id, set()).add(pose_key)
        static_frames = {child for child, poses in poses_per_child.items() if len(poses) == 1}

        # tag each tf row with its child_frame (raw UPDATE on the tf table)
        conn = self._connection()
        for row_id, _ts, child, _parent in rows:
            conn.execute(
                f"UPDATE \"{safe}\" SET tags = json_set(tags, '$.child_frame', ?) WHERE id = ?",
                (child, row_id),
            )
        conn.commit()

        # build the change-log as a first-class stream: one snapshot per change
        graph_stream = self._store.stream(GRAPH_STREAM, TfGraph)
        structure: dict[str, dict[str, Any]] = {}
        written = 0
        for _row_id, ts, child, parent in rows:
            entry = {"parent": parent, "static": child in static_frames}
            if structure.get(child) == entry:
                continue
            structure[child] = entry
            graph_stream.append(TfGraph(structure), ts=ts)
            written += 1
        logger.warning("tf graph built: %d topology changes for %r.", written, self._stream)

    def _graph_codec(self) -> Any:
        source = self._store.stream(GRAPH_STREAM, TfGraph)._source
        return cast("Any", source).codec

    def _load_graph_if_small(self) -> None:
        if self._graph_loaded:
            return
        if self._graph_count() < self._max_in_ram:
            # Sort by (ts, id): several topology changes can share a timestamp (e.g.
            # every static frame latched at t0), and the LAST-inserted of those is the
            # complete snapshot — a plain ts sort leaves same-ts order undefined.
            snapshots = sorted(
                ((obs.ts, obs.id, obs.data.structure) for obs in self._graph_stream()),
                key=lambda row: (row[0], row[1]),
            )
            self._graph_in_ram = [(ts, structure) for ts, _id, structure in snapshots]
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
        # fallback: one indexed query for the latest snapshot at or before query_time.
        # Tie-break by id (DESC) so same-timestamp changes resolve to the complete one.
        self.graph_queries += 1
        conn = self._connection()
        graph, blob = f'"{GRAPH_STREAM}"', f'"{GRAPH_STREAM}_blob"'
        row = conn.execute(
            f"SELECT x.data FROM {graph} g JOIN {blob} x ON x.id = g.id "
            "WHERE g.ts <= ? ORDER BY g.ts DESC, g.id DESC LIMIT 1",
            (query_time,),
        ).fetchone()
        if row is None:  # before the first snapshot -> earliest
            row = conn.execute(
                f"SELECT x.data FROM {graph} g JOIN {blob} x ON x.id = g.id "
                "ORDER BY g.ts ASC, g.id ASC LIMIT 1"
            ).fetchone()
        if row is None:
            return None
        return cast("TfGraph", self._graph_codec().decode(row[0])).structure

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

    def _codec(self) -> Any:
        source = self._store.stream(self._stream, TFMessage)._source
        return cast("Any", source).codec

    def _decode_blob(self, data: bytes, frame: str) -> Transform:
        # The blob is the codec-encoded message; pick the transform for `frame`
        # (rows normally hold one; legacy rows may pack several).
        message = self._codec().decode(data)
        transforms = getattr(message, "transforms", None) or [message]
        for transform in transforms:
            if transform.child_frame_id == frame:
                return cast("Transform", transform)
        return cast("Transform", transforms[0])

    def _fetch_rows(
        self, dynamic: list[str], static: list[str], query_time: float
    ) -> dict[tuple[str, str], bytes]:
        """ONE query: for each dynamic frame the bracketing rows ('lo' = latest at
        or before query_time, 'hi' = earliest at or after), and for each (uncached)
        static frame its latest row ('st') — all joined to the blob data. Keyed by
        (frame, kind) -> blob bytes."""
        cf = "json_extract(tags, '$.child_frame')"
        tf, blob = f'"{self._stream}"', f'"{self._stream}_blob"'
        # One UNION of per-frame, index-served LIMIT-1 subqueries: each is a direct
        # (child_frame, ts) range seek — far cheaper than a window-function scan, and
        # still a single round-trip.
        parts: list[str] = []
        params: list[Any] = []

        def pick(frame: str, kind: str, where_ts: str, order: str) -> None:
            parts.append(
                f"SELECT ? AS cf, ? AS kind, "
                f"(SELECT id FROM {tf} WHERE {cf} = ?{where_ts} ORDER BY ts {order} LIMIT 1) AS id"
            )
            params.extend([frame, kind, frame])

        for frame in dynamic:
            pick(frame, "lo", " AND ts <= ?", "DESC")
            params.append(query_time)
            pick(frame, "hi", " AND ts >= ?", "ASC")
            params.append(query_time)
        for frame in static:
            pick(frame, "st", "", "DESC")
        if not parts:
            return {}
        union = " UNION ALL ".join(parts)
        sql = f"SELECT t.cf, t.kind, b.data FROM ({union}) t JOIN {blob} b ON b.id = t.id"
        rows: dict[tuple[str, str], bytes] = {}
        for cf_val, kind, data in self._connection().execute(sql, params):
            rows[(cf_val, kind)] = data
            self.rows_fetched += 1
        return rows

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        """Transform that maps a point in ``source_frame`` into ``target_frame``,
        or ``None`` if no chain connects them at the requested time."""
        if not self._is_sqlite:
            return self._ensure_loaded().lookup(
                target_frame, source_frame, time_point, time_tolerance
            )
        self._ensure_built()
        self._load_graph_if_small()
        query_time = time_point if time_point is not None else 0.0
        graph = self._graph_at(query_time)  # 0 queries when the graph is in RAM
        if graph is None:
            return None
        frames = self._chain_frames(graph, source_frame, target_frame)
        if frames is None:
            return None

        edges = [f for f in frames if f in graph]  # roots have no incoming edge
        dynamic = [f for f in edges if not graph[f].get("static")]
        static = [f for f in edges if graph[f].get("static")]
        uncached_static = [f for f in static if f not in self._static_cache]

        rows = self._fetch_rows(dynamic, uncached_static, query_time)  # ONE detail query

        buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
        for frame in static:
            transform = self._static_cache.get(frame)
            if transform is None:
                data = rows.get((frame, "st"))
                if data is None:
                    return None
                transform = self._decode_blob(data, frame)
                self._static_cache[frame] = transform
            # restamp the constant to query_time so the buffer's tolerance never
            # rejects a static that was recorded long ago (latched once).
            buffer.receive_transform(_restamp(transform, query_time))
        for frame in dynamic:
            lo = rows.get((frame, "lo"))
            hi = rows.get((frame, "hi"))
            chosen = lo if lo is not None else hi
            if chosen is None:
                return None
            buffer.receive_transform(self._decode_blob(chosen, frame))
            other = hi if hi is not None else lo
            if other is not None and other is not chosen:
                buffer.receive_transform(self._decode_blob(other, frame))
        return buffer.lookup(target_frame, source_frame, time_point, time_tolerance)


class TfGraph:
    """A tf topology snapshot, recorded one per structure change.

    ``structure`` maps each child frame to ``{"parent": str, "static": bool}`` —
    the full tf tree as of this message's timestamp. The stream of these snapshots
    (the ``<tf>_graph`` stream) is the topology change-log that transform lookups
    walk to resolve a source->target chain at any past time. Defined here (not under
    ``dimos/msgs``) because it is a recording-internal payload, not a wire message;
    it is stored via the pickle codec."""

    structure: dict[str, dict[str, Any]]
    msg_name = "tf2_msgs.TfGraph"

    def __init__(self, structure: dict[str, dict[str, Any]]) -> None:
        # copy so later mutations of the writer's running structure don't alter an
        # already-recorded snapshot
        self.structure = {child: dict(entry) for child, entry in structure.items()}

    def __repr__(self) -> str:
        return f"TfGraph({len(self.structure)} frames)"


def _restamp(transform: Transform, ts: float) -> Transform:
    return Transform(
        translation=transform.translation,
        rotation=transform.rotation,
        frame_id=transform.frame_id,
        child_frame_id=transform.child_frame_id,
        ts=ts,
    )


def _safe_table(name: str) -> str:
    if not _VAR_NAME_PATTERN.match(name):
        raise ValueError(f"unsafe stream/table name: {name!r}")
    return name


def _connect(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.execute("PRAGMA busy_timeout=5000")
    return conn


def _ensure_child_index(conn: sqlite3.Connection, stream: str) -> None:
    """Index the child_frame json tag on the tf rows so per-frame time queries
    seek. The live recorder gets this for free (the store auto-indexes tag keys on
    tagged appends); this is for migrated recordings and the read side. Requires
    the tf table to exist."""
    safe = _safe_table(stream)
    # Composite (child_frame, ts) so a per-frame "latest at/before T" is a direct
    # index range seek, not a scan+sort. Index names share SQLite's global namespace
    # with tables, so the name is double-underscore-namespaced to keep it clear of any
    # real stream/table name (no stream would contain "__dbtf_").
    index_name = f"{safe}__dbtf_child_ts_idx"
    conn.execute(
        f'CREATE INDEX IF NOT EXISTS "{index_name}" '
        f"ON \"{safe}\"(json_extract(tags, '$.child_frame'), ts)"
    )
    conn.commit()
