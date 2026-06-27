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

"""Transform lookups over the transforms recorded in a store (multi-robot friendly).

Two pieces of recorded state, written by the recorder:

* a **graph stream** (table ``tf_graph``): one row per *topology* change — i.e.
  whenever the set of frames or any frame's parent / static-ness changes. Each row
  is the full structure at that instant: ``{child_frame: {parent, static}}``.
  Topology changes rarely (a robot joins/leaves, a relocalization re-parents), so
  this table is tiny and naturally supports time-varying / multi-robot trees.
* the ``tf`` stream, with each row tagged by its ``child_frame`` (an indexed json
  tag) so a frame's samples can be range-queried by time.

``store.tf.get(target, source, ts)`` then: reads the graph as-of the query time
(from RAM if there are few graph changes, else one query), walks it to the
source->target chain (in memory; the graph may be DISJOINT for unrelated robots),
and resolves *only* that chain's frames — a static frame is one cached constant, a
dynamic frame is its two bracketing samples, interpolated. Composition +
interpolation reuse :class:`MultiTBuffer`. Non-sqlite stores fall back to loading
the tf streams into a buffer.

``write_tf_tree`` populates the tf stream for a recording that lacks one.
"""

from __future__ import annotations

import bisect
import re
import sqlite3
import threading
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory2.store.sqlite import SqliteStoreConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream

logger = setup_logger()

DEFAULT_TF_STREAM = "tf"
# The topology change-log is a first-class stream named "<tf stream>_graph".
GRAPH_STREAM_SUFFIX = "_graph"
# Streams the RAM fallback (non-sqlite stores) reads.
TF_STREAMS = ("tf", "tf_static")
# If a recording has fewer than this many topology changes, load them all into RAM
# so a lookup needs no graph query (the common single-robot / stable-tree case).
# At or above it, fall back to one graph query per lookup (many-robot churn).
DEFAULT_MAX_GRAPH_CHANGES_IN_RAM = 20
# Larger than any single recording's span so the fallback buffer never prunes.
_NO_PRUNE = 1.0e15
# Lower bound for "latest topology at or before T" range queries.
_TS_MIN = -1.0e18
# SQLite can't parameterize table names, so caller-supplied stream names are
# interpolated; allow only safe identifiers to keep that injection-free.
_SAFE_TABLE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _graph_stream_name(stream: str) -> str:
    return f"{_safe_table(stream)}{GRAPH_STREAM_SUFFIX}"


def _safe_table(name: str) -> str:
    if not _SAFE_TABLE.match(name):
        raise ValueError(f"unsafe stream/table name: {name!r}")
    return name


def _connect(db_path: str) -> sqlite3.Connection:
    """A connection that waits on the WAL write-lock instead of erroring — the
    store keeps its own connection to the same db open while we read/write."""
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
    # index range seek, not a scan+sort.
    conn.execute(
        f'CREATE INDEX IF NOT EXISTS "{safe}_child_ts_idx" '
        f"ON \"{safe}\"(json_extract(tags, '$.child_frame'), ts)"
    )
    conn.commit()


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


class TfGraphWriter:
    """Recorder helper: tracks the running topology and appends a ``TfGraph``
    snapshot to the graph stream only when the structure changes."""

    def __init__(self, store: Store, stream: str = DEFAULT_TF_STREAM) -> None:
        self._stream: Stream[TfGraph] = store.stream(_graph_stream_name(stream), TfGraph)
        self._structure: dict[str, dict[str, Any]] = {}

    def record(self, child_frame: str, parent_frame: str, is_static: bool, ts: float) -> None:
        entry = {"parent": parent_frame, "static": bool(is_static)}
        if self._structure.get(child_frame) == entry:
            return  # no structural change -> no new snapshot
        self._structure[child_frame] = entry
        self._stream.append(TfGraph(self._structure), ts=ts)

    def close(self) -> None:
        # the stream is owned by the store; nothing to close here
        pass


def build_graph_stream(store: Store, stream: str = DEFAULT_TF_STREAM) -> int:
    """One-time migration for a recording that predates the graph stream: tag every
    tf row with its ``child_frame`` and build the ``<tf>_graph`` stream
    chronologically. A frame is treated as static if its pose never changes across
    the recording. Returns the number of topology-change snapshots written."""
    config = store.config
    if not isinstance(config, SqliteStoreConfig):
        raise TypeError("build_graph_stream needs a SqliteStore")
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

    # tag each tf row with its child_frame + add the seek index (raw, on the tf table)
    conn = _connect(config.path)
    try:
        for row_id, _ts, child, _parent, _pose in rows:
            conn.execute(
                f"UPDATE \"{safe}\" SET tags = json_set(tags, '$.child_frame', ?) WHERE id = ?",
                (child, row_id),
            )
        conn.commit()
        _ensure_child_index(conn, safe)
    finally:
        conn.close()

    # build the topology change-log as a first-class stream
    graph_stream = store.stream(_graph_stream_name(safe), TfGraph)
    structure: dict[str, dict[str, Any]] = {}
    written = 0
    for _row_id, ts, child, parent, _pose in rows:
        entry = {"parent": parent, "static": child in static_frames}
        if structure.get(child) == entry:
            continue
        structure[child] = entry
        graph_stream.append(TfGraph(structure), ts=ts)
        written += 1
    return written


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
        self._graph_name = _graph_stream_name(stream)
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
        return self._store.stream(self._graph_name, TfGraph)

    def _graph_count(self) -> int:
        if self._graph_name not in set(self._store.list_streams()):
            return 0
        return self._graph_stream().count()

    def _ensure_built(self) -> None:
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
            built = build_graph_stream(self._store, self._stream)
            logger.warning("tf graph built: %d topology changes for %r.", built, self._stream)
        if n_rows:
            _ensure_child_index(conn, self._stream)  # tf table exists now
        self._built = True

    def _load_graph_if_small(self) -> None:
        if self._graph_loaded:
            return
        if self._graph_count() < self._max_in_ram:
            self._graph_in_ram = [
                (obs.ts, obs.data.structure) for obs in self._graph_stream().order_by("ts")
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
        # fallback: one query for the latest snapshot at or before query_time
        self.graph_queries += 1
        stream = self._graph_stream()
        latest = stream.time_range(_TS_MIN, query_time).order_by("ts", desc=True).limit(1)
        for obs in latest:
            return obs.data.structure
        for obs in stream.order_by("ts").limit(1):  # before first -> earliest
            return obs.data.structure
        return None

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


def _restamp(transform: Transform, ts: float) -> Transform:
    return Transform(
        translation=transform.translation,
        rotation=transform.rotation,
        frame_id=transform.frame_id,
        child_frame_id=transform.child_frame_id,
        ts=ts,
    )


def transform_matrix(transform: Transform) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(R, t)`` (3x3, 3) for ``transform`` so ``p_target = p_source @ R.T + t``."""
    rotation = transform.rotation
    rotation_matrix = np.asarray(rotation.to_rotation_matrix(), float).reshape(3, 3)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z], float
    )
    return rotation_matrix, translation


def write_tf_tree(
    store: Store,
    *,
    odom_stream: str,
    odom_parent: str = "odom",
    odom_child: str = "base_link",
    root_links: tuple[tuple[str, str], ...] = (("world", "map"), ("map", "odom")),
    sensor_child: str = "mid360_link",
    sensor_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    sensor_rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    static_period: float = 0.45,
    stream_name: str = "tf",
) -> int:
    """Populate ``store``'s tf stream from an odometry stream.

    - ``root_links`` and ``odom_child -> sensor_child`` are emitted as identity /
      fixed transforms every ``static_period`` seconds across the recording span.
    - ``odom_parent -> odom_child`` is emitted once per odometry sample, taken
      from each observation's pose.

    Each transform is written as its own row (one transform per ``TFMessage``) so
    the graph-stream reader can range-query it by ``child_frame``. Returns the
    number of tf observations written.
    """
    config = store.config
    if not isinstance(config, SqliteStoreConfig):
        raise TypeError("write_tf_tree reads the db directly and needs a SqliteStore")
    db_path = config.path
    connection = sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True)
    odom = np.array(
        list(
            connection.execute(
                "select ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
                f"from {_safe_table(odom_stream)} order by ts"
            )
        ),
        float,
    )
    connection.close()
    if not len(odom):
        raise ValueError(f"odom stream {odom_stream!r} is empty; cannot build tf tree")

    tf_stream = store.stream(stream_name, TFMessage)
    written = 0

    def _append(transform: Transform, ts: float) -> None:
        nonlocal written
        tf_stream.append(
            TFMessage(transform), ts=ts, tags={"child_frame": transform.child_frame_id}
        )
        written += 1

    # dynamic: odom_parent -> odom_child, one per odometry sample
    for row in odom:
        ts = float(row[0])
        _append(
            Transform(
                translation=Vector3(row[1], row[2], row[3]),
                rotation=Quaternion(row[4], row[5], row[6], row[7]),
                frame_id=odom_parent,
                child_frame_id=odom_child,
                ts=ts,
            ),
            ts,
        )

    # static: root links + sensor mount, resampled every static_period
    t0 = float(odom[0, 0])
    t1 = float(odom[-1, 0])

    def statics_at(ts: float) -> list[Transform]:
        links = [
            Transform(frame_id=parent, child_frame_id=child, ts=ts) for parent, child in root_links
        ]
        links.append(
            Transform(
                translation=Vector3(*sensor_translation),
                rotation=Quaternion(*sensor_rotation),
                frame_id=odom_child,
                child_frame_id=sensor_child,
                ts=ts,
            )
        )
        return links

    for static_ts in np.arange(t0, t1 + static_period, static_period):
        for transform in statics_at(float(static_ts)):
            _append(transform, float(static_ts))

    return written
