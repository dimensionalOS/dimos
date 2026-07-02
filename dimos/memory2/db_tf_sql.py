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

"""SQLite transform lookups — the sqlite graph-stream path (indexed per-lookup queries
plus loop-closure deformation correction). Used only by the sqlite store."""

from __future__ import annotations

import bisect
import math
import sqlite3
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory2.db_tf import TfGraph
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.DeformationNode import DeformationNode, tf_id_for
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.deformation import apply_delta, blend_se3
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.store.sqlite import SqliteStoreConfig
    from dimos.memory2.stream import Stream

logger = setup_logger()

GRAPH_STREAM = "tf_graph"  # changes to the tf tree (with room for non-tree structures)
DEFORMATION_STREAM = "tf_deformation_nodes"  # loop closure and other deformations
# 99% of the time there are less than 10 tree changes
# this cache allows us to avoid a DB query in the offline/map-load case
DEFAULT_MAX_GRAPH_CHANGES_IN_RAM = 64
# A frame is "static" if its pose never changes; poses are compared rounded to this
# many decimals (~nanometre / nanoradian) so float noise doesn't read as motion.
POSE_EQUALITY_DECIMALS = 9


class DbTfSql:
    """Transform lookups over the sqlite graph-stream: a topology change-log plus
    per-lookup indexed queries for the bracketing samples, with loop-closure
    deformation correction. Used only by the sqlite store."""

    def __init__(
        self,
        store: Store,
        max_graph_changes_in_ram: int = DEFAULT_MAX_GRAPH_CHANGES_IN_RAM,
    ) -> None:
        self._store = store
        self._max_in_ram = max_graph_changes_in_ram
        self._conn: sqlite3.Connection | None = None
        self._built = False
        # graph cache: either the whole change-log in RAM, or None (query per lookup)
        self._graph_in_ram: list[tuple[float, dict[str, Any]]] | None = None
        self._graph_loaded = False
        self._static_cache: dict[str, Transform] = {}
        # The set of tf_ids present on the deformation stream (the edges loop closure
        # corrects). Cached once so a recording with no deformation nodes pays nothing.
        self._deformation_tf_ids: set[int] | None = None
        self.rows_fetched = 0
        self.graph_queries = 0

    def _connection(self) -> sqlite3.Connection:
        conn = self._conn
        if conn is None:
            conn = _connect(cast("SqliteStoreConfig", self._store.config).path)
            self._conn = conn
        return conn

    def has_transforms(self) -> bool:
        if "tf" not in set(self._store.list_streams()):
            return False
        (n_rows,) = self._connection().execute('SELECT count(*) FROM "tf"').fetchone()
        return bool(n_rows)

    def _graph_stream(self) -> Stream[TfGraph]:
        return self._store.stream(GRAPH_STREAM, TfGraph)

    def _graph_count(self) -> int:
        if GRAPH_STREAM not in set(self._store.list_streams()):
            return 0
        return self._graph_stream().count()

    def _ensure_built(self) -> None:
        """First use: if the recording has tf rows but no graph stream (a recording
        that predates it / wasn't written by the recorder), build the graph stream once
        by replaying the tf rows, then make sure the seek index exists."""
        if self._built:
            return
        conn = self._connection()
        (n_rows,) = conn.execute('SELECT count(*) FROM "tf"').fetchone()
        if n_rows and self._graph_count() == 0:
            logger.warning(
                "\n========================================================================\n"
                "  tf graph stream MISSING for %r. Building it (one-time): tagging tf rows\n"
                "  with child_frame and writing the topology change-log.\n"
                "========================================================================",
                "tf",
            )
            self._build_graph_stream()
        if n_rows:
            _ensure_child_index(conn)  # tf table exists now
        self._built = True

    def _build_graph_stream(self) -> None:
        """One-time migration: decode every tf row, tag it with its child_frame, and
        append a ``TfGraph`` snapshot whenever the topology changes. A frame counts as
        static if its pose never varies across the whole recording."""
        # one decode pass: collect (id, ts, child, parent, pose-key) + per-child poses
        rows: list[tuple[int, float, str, str]] = []
        poses_per_child: dict[str, set[tuple[float, ...]]] = {}
        for obs in self._store.stream("tf", TFMessage).order_by("ts"):
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
                "UPDATE \"tf\" SET tags = json_set(tags, '$.child_frame', ?) WHERE id = ?",
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
        logger.warning("tf graph built: %d topology changes for %r.", written, "tf")

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
        codec = cast("Any", self._store.stream(GRAPH_STREAM, TfGraph)._source).codec
        return cast("TfGraph", codec.decode(row[0])).structure

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

    def _decode_blob(self, data: bytes, frame: str) -> Transform:
        # The blob is the codec-encoded message; pick the transform for `frame`
        # (rows normally hold one; legacy rows may pack several).
        codec = cast("Any", self._store.stream("tf", TFMessage)._source).codec
        message = codec.decode(data)
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
        # One UNION of per-frame, index-served LIMIT-1 subqueries: each is a direct
        # (child_frame, ts) range seek — far cheaper than a window-function scan, and
        # still a single round-trip.
        parts: list[str] = []
        params: list[Any] = []

        def pick(frame: str, kind: str, where_ts: str, order: str) -> None:
            parts.append(
                f"SELECT ? AS cf, ? AS kind, "
                f'(SELECT id FROM "tf" WHERE {cf} = ?{where_ts} ORDER BY ts {order} LIMIT 1) AS id'
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
        sql = f'SELECT t.cf, t.kind, b.data FROM ({union}) t JOIN "tf_blob" b ON b.id = t.id'
        rows: dict[tuple[str, str], bytes] = {}
        for cf_val, kind, data in self._connection().execute(sql, params):
            rows[(cf_val, kind)] = data
            self.rows_fetched += 1
        return rows

    def _load_deformation_ids(self) -> set[int]:
        """The distinct tf_ids present on the deformation stream, cached once. Empty
        when there's no such stream — so the correction path is skipped at zero cost
        for recordings without loop closure."""
        if self._deformation_tf_ids is not None:
            return self._deformation_tf_ids
        ids: set[int] = set()
        if DEFORMATION_STREAM in set(self._store.list_streams()):
            for (tf_id,) in self._connection().execute(
                f"SELECT DISTINCT json_extract(tags, '$.tf_id') FROM \"{DEFORMATION_STREAM}\""
            ):
                if tf_id is not None:
                    ids.add(int(tf_id))
        self._deformation_tf_ids = ids
        return ids

    def _node_pose_matrix(self, order: str, tf_id: int, node_id: str) -> np.ndarray | None:
        """The 4x4 pose of one keyframe's first (``ASC``) or latest (``DESC``) recorded
        version. Versions of a node share the keyframe ts, so they're ordered by row id
        (insertion order), not ts."""
        stream, blob = f'"{DEFORMATION_STREAM}"', f'"{DEFORMATION_STREAM}_blob"'
        row = (
            self._connection()
            .execute(
                f"SELECT b.data FROM {stream} s JOIN {blob} b ON b.id = s.id "
                f"WHERE json_extract(s.tags, '$.tf_id') = ? AND json_extract(s.tags, '$.id') = ? "
                f"ORDER BY s.id {order} LIMIT 1",
                (str(tf_id), node_id),
            )
            .fetchone()
        )
        if row is None:
            return None
        codec = cast("Any", self._store.stream(DEFORMATION_STREAM, DeformationNode)._source).codec
        node = cast("DeformationNode", codec.decode(row[0]))
        return Transform.from_pose(node.pose.frame_id, node.pose).to_matrix()

    def _node_delta(self, tf_id: int, node_id: str) -> np.ndarray | None:
        """How far the optimizer has moved a keyframe since it was first recorded:
        ``current ∘ inv(original)`` — the SE(3) correction this node contributes."""
        original = self._node_pose_matrix("ASC", tf_id, node_id)
        current = self._node_pose_matrix("DESC", tf_id, node_id)
        if original is None or current is None:
            return None
        return current @ np.linalg.inv(original)

    def _edge_delta(self, tf_id: int, query_time: float) -> np.ndarray | None:
        """The blended correction for an edge at ``query_time``: take the keyframes
        bracketing the time (latest at-or-before, earliest at-or-after), each node's
        ``current ∘ inv(original)`` delta, and linear-blend-skin between them."""
        stream = f'"{DEFORMATION_STREAM}"'
        id_tag = "json_extract(tags, '$.id')"
        tf_tag = "json_extract(tags, '$.tf_id')"
        conn = self._connection()
        lo = conn.execute(
            f"SELECT {id_tag}, ts FROM {stream} WHERE {tf_tag} = ? AND ts <= ? "
            "ORDER BY ts DESC, id DESC LIMIT 1",
            (str(tf_id), query_time),
        ).fetchone()
        hi = conn.execute(
            f"SELECT {id_tag}, ts FROM {stream} WHERE {tf_tag} = ? AND ts >= ? "
            "ORDER BY ts ASC, id ASC LIMIT 1",
            (str(tf_id), query_time),
        ).fetchone()
        samples: list[tuple[float, np.ndarray]] = []
        seen_ids: set[str] = set()
        for node_id, ts in (row for row in (lo, hi) if row is not None):
            if node_id in seen_ids:
                continue
            seen_ids.add(node_id)
            delta = self._node_delta(tf_id, node_id)
            if delta is not None:
                samples.append((ts, delta))
        if not samples:
            return None
        if len(samples) == 1:
            return samples[0][1]
        (ts_lo, mat_lo), (ts_hi, mat_hi) = sorted(samples, key=lambda item: item[0])
        weight = 0.0 if ts_hi == ts_lo else (query_time - ts_lo) / (ts_hi - ts_lo)
        return blend_se3(mat_lo, mat_hi, weight)

    def _edge_corrections(
        self, graph: dict[str, Any], edges: list[str], query_time: float
    ) -> dict[str, np.ndarray]:
        """For each edge whose tf_id matches the deformation stream, its blended SE(3)
        correction (applied on the parent side of the edge). Empty when nothing matches."""
        tf_ids = self._load_deformation_ids()
        if not tf_ids:
            return {}
        corrections: dict[str, np.ndarray] = {}
        for frame in edges:
            tf_id = tf_id_for(graph[frame]["parent"], frame)
            if tf_id not in tf_ids:
                continue
            delta = self._edge_delta(tf_id, query_time)
            if delta is not None:
                corrections[frame] = delta
        return corrections

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        """Transform that maps a point in ``source_frame`` into ``target_frame``,
        or ``None`` if no chain connects them at the requested time."""
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
        # Per-edge loop-closure corrections (empty unless a deformation stream matches).
        corrections = self._edge_corrections(graph, edges, query_time)

        buffer = MultiTBuffer(buffer_size=math.inf)
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
            transform = _restamp(transform, query_time)
            if frame in corrections:
                transform = apply_delta(corrections[frame], transform, query_time)
            buffer.receive_transform(transform)
        for frame in dynamic:
            lo = rows.get((frame, "lo"))
            hi = rows.get((frame, "hi"))
            chosen = lo if lo is not None else hi
            if chosen is None:
                return None
            if frame in corrections:
                # Resolve the raw edge at query_time, then deform it. The correction
                # already carries the time blend, so a single corrected sample suffices.
                raw = self._interpolate_dynamic(
                    frame, graph[frame]["parent"], lo, hi, query_time, time_tolerance
                )
                if raw is None:
                    return None
                buffer.receive_transform(apply_delta(corrections[frame], raw, query_time))
                continue
            buffer.receive_transform(self._decode_blob(chosen, frame))
            other = hi if hi is not None else lo
            if other is not None and other is not chosen:
                buffer.receive_transform(self._decode_blob(other, frame))
        return buffer.lookup(target_frame, source_frame, time_point, time_tolerance)

    def _interpolate_dynamic(
        self,
        frame: str,
        parent: str,
        lo: bytes | None,
        hi: bytes | None,
        query_time: float,
        time_tolerance: float | None,
    ) -> Transform | None:
        """The raw ``parent <- frame`` transform at ``query_time``, interpolated from
        the bracketing rows (its own small buffer so the chain buffer only ever sees
        the corrected result)."""
        edge_buffer = MultiTBuffer(buffer_size=math.inf)
        chosen = lo if lo is not None else hi
        if chosen is None:
            return None
        edge_buffer.receive_transform(self._decode_blob(chosen, frame))
        other = hi if hi is not None else lo
        if other is not None and other is not chosen:
            edge_buffer.receive_transform(self._decode_blob(other, frame))
        return edge_buffer.lookup(parent, frame, query_time, time_tolerance)


def _restamp(transform: Transform, ts: float) -> Transform:
    return Transform(
        translation=transform.translation,
        rotation=transform.rotation,
        frame_id=transform.frame_id,
        child_frame_id=transform.child_frame_id,
        ts=ts,
    )


def _connect(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.execute("PRAGMA busy_timeout=5000")
    return conn


def _ensure_child_index(conn: sqlite3.Connection) -> None:
    """Index the child_frame json tag on the tf rows so per-frame time queries
    seek. The live recorder gets this for free (the store auto-indexes tag keys on
    tagged appends); this is for migrated recordings and the read side. Requires
    the tf table to exist."""
    # Composite (child_frame, ts) so a per-frame "latest at/before T" is a direct
    # index range seek, not a scan+sort. Index names share SQLite's global namespace
    # with tables, so the name is double-underscore-namespaced to keep it clear of any
    # real stream/table name (no stream would contain "__dbtf_").
    index_name = "tf__dbtf_child_ts_idx"
    conn.execute(
        f'CREATE INDEX IF NOT EXISTS "{index_name}" '
        "ON \"tf\"(json_extract(tags, '$.child_frame'), ts)"
    )
    conn.commit()
