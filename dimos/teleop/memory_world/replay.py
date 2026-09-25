# Copyright 2025-2026 Dimensional Inc.
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

"""Voxel replay: reading the keyframe-and-diff streams an older recording carries.

``voxel_keyframe`` holds the whole voxel set every few seconds and ``voxel_diff`` holds
what each lidar scan added or removed, so any moment is the nearest earlier keyframe plus
the diffs up to it -- and a viewer already showing one moment reaches its neighbour by
applying, or un-applying, a few of them. :class:`VoxelReplay` serves that to a browser as
segments and its final keyframe doubles as the static map.

**NOTHING WRITES THESE STREAMS ANY MORE.** They were ray-traced from the raw scans on
first use, which cost half an hour on a large recording and was deleted on 2026-09-22:
a recording whose mapper published ``global_map`` as it grew already holds the map at
every moment of it, and the timeline is read off those messages instead (see
``world_cache._map_timeline``). This file is kept for the recordings made before that,
which carry the streams already; a recording without them has no voxel timeline and is
scrubbed through its map stream or not at all.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Iterable
from dataclasses import dataclass
import gzip
import json
import struct
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DIFF_STREAM = "voxel_diff"
KEYFRAME_STREAM = "voxel_keyframe"
TAG_ADDED = 1
TAG_REMOVED = 2
WIRE_CACHE_BYTES = 256_000_000  # gzipped segments kept for the next viewer


def accumulate_scans(
    stream: Any, to_world: Callable[[Any], Any], voxel_size: float, n_scans: int
) -> np.ndarray | None:
    """*n_scans* scans spread over *stream* voxelised into one (N, 3) cloud, no clearing.

    ``to_world`` maps a lidar observation to one in the world frame (None to
    skip it). ``n_scans <= 0`` uses every scan. This is the plain accumulation used when
    a recording carries neither a global map nor ray-traced replay streams.
    """
    from dimos.mapping.voxels.module import VoxelMapTransformer
    from dimos.memory.transform import FnTransformer, throttle

    try:
        first, last = stream.first(), stream.last()
    except LookupError:  # declared but empty: no scans is "no cloud", not a stream error
        return None
    span = max(float(last.ts) - float(first.ts), 1e-3)

    def placed(obs: Any) -> Any:
        cloud = to_world(obs)
        if cloud is None:
            return None
        return cloud if cloud is obs else obs.derive(data=cloud)

    pipeline = stream if n_scans <= 0 else stream.transform(throttle(span / n_scans))
    try:
        result = (
            pipeline.transform(FnTransformer(placed))
            .transform(VoxelMapTransformer(emit_every=0, voxel_size=voxel_size))
            .last()
        )
    except LookupError:
        # tf placed no scan at all, which is the commonest reason a memory world comes up
        # empty. The `-> ndarray | None` above promised this, and last() raising instead
        # sent the operator a traceback from the stream layer rather than the real answer.
        return None
    if result is None or result.data is None:
        return None
    xyz, _ = result.data.as_numpy()
    return None if xyz is None else np.asarray(xyz, dtype=np.float32)


def frame_positions(
    stamps: Iterable[float], pose_at: Callable[[float], np.ndarray | None]
) -> list[list[float]]:
    """A frame's position at each stamp, from *pose_at* (world_T_frame or None).

    A gap repeats the previous position, so a viewer following the robot never
    jumps. A gap BEFORE the first known position is back-filled with that first
    position rather than the origin: the same array is the path a route is planned
    over, and a run of origins there is a straight line through unmapped space that
    is indistinguishable from somewhere the robot actually went.
    """
    return _held_through_gaps(
        None if (matrix := pose_at(float(ts))) is None else [float(v) for v in matrix[:3, 3]]
        for ts in stamps
    )


def stamped_positions(observations: Iterable[Any]) -> list[list[float]]:
    """The pose stamped on each observation, gaps repeating the nearest known one."""
    return _held_through_gaps(
        None if (pose := getattr(obs, "pose_tuple", None)) is None else [float(v) for v in pose[:3]]
        for obs in observations
    )


def _held_through_gaps(known: Iterable[list[float] | None]) -> list[list[float]]:
    """Each position, with a gap holding the last known one and a leading gap the first.

    Nothing is invented: an unknown position is reported as the nearest known one, so
    a consumer that reads this as a path never sees a place the robot was not.
    """
    positions = list(known)
    first = next((p for p in positions if p is not None), None)
    if first is None:
        # Nothing is known, so there is nothing to hold. Returning origins here would
        # report the robot at the world origin for a whole recording, which is exactly
        # the invented place this function exists to avoid -- and /navigate would plan a
        # route from it. Both callers already treat an empty path as "not known yet".
        return []
    held: list[list[float]] = []
    last = first
    for position in positions:
        if position is not None:
            last = position
        held.append([round(v, 3) for v in last])
    return held


# ---- reading ---------------------------------------------------------------


@dataclass
class ReplayIndex:
    """Everything a viewer needs to seek: scan stamps and where the keyframes sit."""

    voxel_size: float
    scan_ts: np.ndarray  # (S,) float64, one per diff message, in stream order
    keyframe_scan: np.ndarray  # (K,) int, scan index each keyframe was taken after
    keyframe_ts: np.ndarray  # (K,) float64
    origin: tuple[int, int, int]  # voxel index the int16 wire coordinates are relative to

    # scan_at and segment_of are the reference for the viewer's scanAt/segmentOf
    # (replay.js); the server itself seeks by scan index. Both assume ascending
    # stamps, which _load_index checks.
    def scan_at(self, ts: float) -> int:
        """Index of the last scan at or before *ts* (0 before the first)."""
        return max(0, int(np.searchsorted(self.scan_ts, ts, side="right")) - 1)

    def segment_of(self, scan: int) -> int:
        """Index of the keyframe a scan is replayed from."""
        return max(0, int(np.searchsorted(self.keyframe_scan, scan, side="right")) - 1)

    def segment_scans(self, segment: int) -> tuple[int, int]:
        """Half-open scan range [start, end) covered by a keyframe."""
        start = int(self.keyframe_scan[segment])
        end = (
            int(self.keyframe_scan[segment + 1])
            if segment + 1 < len(self.keyframe_scan)
            else len(self.scan_ts)
        )
        return start, end

    def to_json(self) -> dict[str, Any]:
        return {
            "voxel_size": self.voxel_size,
            "origin": list(self.origin),
            "scans": [round(float(t), 4) for t in self.scan_ts],
            "keyframes": [
                {"scan": int(s), "ts": round(float(t), 4)}
                for s, t in zip(self.keyframe_scan, self.keyframe_ts, strict=True)
            ],
        }


def _final(keyframes: Any) -> Any:
    """The keyframe with the highest scan index (two scans can share a stamp, so
    not ``last()``); tags only, the clouds stay on disk until ``.data``."""
    return max(keyframes, key=lambda obs: int((obs.tags or {}).get("scan_index", -1)))


class VoxelReplay:
    """Serves a recording's replay streams as compact segments for a viewer.

    A segment is one keyframe plus the diffs of every scan up to the next
    keyframe, resolved for the viewer: a table of every voxel position the
    segment can show (the keyframe's, then any the diffs add) as int16 grid
    indices relative to a shared origin, and each diff entry as the slot in
    that table plus an op. Seeking on the viewer is then byte flips in a
    visibility array, no hashing of voxel keys at all.
    """

    def __init__(
        self,
        store: Any,
        *,
        diff_stream_name: str = DIFF_STREAM,
        keyframe_stream_name: str = KEYFRAME_STREAM,
        z_min: float = -np.inf,
        z_max: float = np.inf,
    ) -> None:
        self.store = store
        self.diffs = store.streams[diff_stream_name]
        self.keyframes = store.streams[keyframe_stream_name]
        self.z_min = z_min
        self.z_max = z_max
        self.index = self._load_index()
        # Gzipped wire form of the segments served so far, most recent last. A long
        # recording has gigabytes of segments and the viewer preloads them all.
        self._wire: OrderedDict[int, bytes] = OrderedDict()
        self._wire_bytes = 0

    @staticmethod
    def available(store: Any) -> bool:
        """True when this recording carries a finished, seekable pair of replay streams.

        The tag checks this used to make -- voxel size, ray range, lidar stream, keyframe
        spacing, format version -- all existed to decide whether to REBUILD, and nothing
        rebuilds any more. What is on disk is either readable or the recording has no
        voxel timeline; the index takes its voxel size from the streams' own tags, so
        serving them is self-consistent whatever built them.
        """
        names = store.list_streams()
        if DIFF_STREAM not in names or KEYFRAME_STREAM not in names:
            return False
        keyframes = store.streams[KEYFRAME_STREAM]
        if keyframes.count() == 0:  # a build that died before its first keyframe
            return False
        # ...and one that has no DIFFS is not a replay either, whatever its keyframes say.
        # The streams were accepted, `/replay/index` served `scans: []`, and the viewer sat
        # on "replay index still building (0 scans)" for ever. `next(iter(...))` rather
        # than `count()`: on an mcap the two disagree.
        if next(iter(store.streams[DIFF_STREAM]), None) is None:
            return False
        # An interrupted build has no keyframe tagged `last`, and half a map is not the
        # map: roscon's old companion db looked complete at 9,196 diffs and 184 keyframes and
        # is an abandoned run from that afternoon. A ROW COUNT DOES NOT SAY A BUILD
        # FINISHED; THIS TAG DOES.
        if not bool(_final(keyframes).tags.get("last")):
            logger.info("the replay streams in this recording are half-written; ignoring them")
            return False
        return True

    def _load_index(self) -> ReplayIndex:
        first = self.keyframes.first()
        tags = dict(first.tags or {})
        voxel_size = float(tags["voxel_size"])
        scan_ts = np.array([float(obs.ts) for obs in self.diffs], dtype=np.float64)
        if len(scan_ts) > 1 and np.any(np.diff(scan_ts) < 0):
            logger.warning("replay scan stamps are not ascending; seeks by time are approximate")
        keyframe_scan: list[int] = []
        keyframe_ts: list[float] = []
        for obs in self.keyframes:  # tags only: the clouds stay on disk
            keyframe_scan.append(int(obs.tags["scan_index"]))
            keyframe_ts.append(float(obs.ts))
        last = _final(self.keyframes).tags
        low, high = (
            np.array(last["low"], dtype=np.float64),
            np.array(last["high"], dtype=np.float64),
        )
        centre = np.floor((low + high) / 2 / voxel_size).astype(np.int64)
        return ReplayIndex(
            voxel_size=voxel_size,
            scan_ts=scan_ts,
            keyframe_scan=np.array(keyframe_scan, dtype=np.int64),
            keyframe_ts=np.array(keyframe_ts, dtype=np.float64),
            origin=(int(centre[0]), int(centre[1]), int(centre[2])),
        )

    def final_keyframe(self) -> Any:
        """The keyframe of the last scan: the finished map."""
        return _final(self.keyframes)

    def _grid_indices(self, cloud: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
        """(N, 3) int64 grid indices of a cloud's voxels within the z slab, and the kept mask."""
        points = cloud.points_f32()
        keep = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        indices = np.floor(points[keep] / self.index.voxel_size).astype(np.int64)
        indices -= np.asarray(self.index.origin, dtype=np.int64)
        if len(indices) and np.abs(indices).max() > np.iinfo(np.int16).max:
            raise ValueError("voxel further than int16 range from the replay origin")
        return indices, keep

    @staticmethod
    def _keys(indices: np.ndarray) -> np.ndarray:
        """One int64 per voxel; only used to match diff entries to table slots."""
        shifted = indices + np.int64(1 << 15)
        return (shifted[:, 0] << 32) | (shifted[:, 1] << 16) | shifted[:, 2]

    def segment(self, number: int) -> tuple[dict[str, Any], bytes]:
        """Header and payload of one segment.

        Payload: ``int16 xyz`` for every slot of the position table (keyframe
        voxels first, then voxels the diffs introduce, in order of appearance),
        then ``uint32`` slot per diff entry of every scan in order, then one
        ``uint8`` op per entry.
        """
        start, end = self.index.segment_scans(number)
        keyframe = self.keyframes.tags(scan_index=start).first()  # by index: stamps can repeat
        table, _ = self._grid_indices(keyframe.data)
        table_keys = self._keys(table)
        order = np.argsort(table_keys)
        table_keys = table_keys[order]
        table = table[order]
        keyframe_n = len(table)

        scans: list[dict[str, Any]] = []
        entry_keys: list[np.ndarray] = []
        entry_ops: list[np.ndarray] = []
        for scan_index, obs in enumerate(self._diffs_between(start, end), start=start):
            indices, keep = self._grid_indices(obs.data)
            tags = obs.data.tags_u8()
            ops = np.zeros(0, np.uint8) if tags is None else np.asarray(tags, np.uint8)[keep]
            scans.append({"index": scan_index, "ts": round(float(obs.ts), 4), "n": len(indices)})
            entry_keys.append(self._keys(indices))
            entry_ops.append(ops)
        all_keys = np.concatenate(entry_keys) if entry_keys else np.zeros(0, np.int64)
        all_ops = np.concatenate(entry_ops) if entry_ops else np.zeros(0, np.uint8)

        # voxels the diffs mention that the keyframe lacks get the slots after it
        position = np.searchsorted(table_keys, all_keys)
        in_table = position < keyframe_n
        in_table[in_table] = table_keys[position[in_table]] == all_keys[in_table]
        extra_keys, first_seen = np.unique(all_keys[~in_table], return_index=True)
        extra_keys = extra_keys[np.argsort(first_seen)]  # slots in order of appearance
        slots = np.empty(len(all_keys), dtype=np.uint32)
        slots[in_table] = position[in_table]
        if len(extra_keys):
            extra_sorted = np.argsort(extra_keys)
            found = np.searchsorted(extra_keys[extra_sorted], all_keys[~in_table])
            slots[~in_table] = keyframe_n + extra_sorted[found]
            shifted = np.stack(
                [
                    (extra_keys >> 32) & 0xFFFF,
                    (extra_keys >> 16) & 0xFFFF,
                    extra_keys & 0xFFFF,
                ],
                axis=1,
            ) - np.int64(1 << 15)
            table = np.concatenate([table, shifted])

        header = {
            "segment": number,
            "keyframe": {
                "scan": start,
                "ts": round(float(self.index.keyframe_ts[number]), 4),
                "n": keyframe_n,
            },
            "slots": len(table),
            "slots_offset": len(table) * 6 + (-(len(table) * 6) % 4),  # uint32-aligned
            "scans": scans,
        }
        payload = b"".join(
            [
                np.ascontiguousarray(table.astype("<i2")).tobytes(),
                b"\0" * (-(len(table) * 6) % 4),
                slots.astype("<u4").tobytes(),
                all_ops.tobytes(),
            ]
        )
        return header, payload

    def encoded_segment(self, number: int) -> bytes:
        """The gzipped wire form of a segment, built once and kept within WIRE_CACHE_BYTES."""
        cached = self._wire.get(number)
        if cached is None:
            cached = gzip.compress(
                self.encode_segment(*self.segment(number)), compresslevel=6, mtime=0
            )  # mtime=0: the same bytes every time, so the viewer's cache validates
            self._wire[number] = cached
            self._wire_bytes += len(cached)
            while self._wire_bytes > WIRE_CACHE_BYTES and len(self._wire) > 1:
                self._wire_bytes -= len(self._wire.popitem(last=False)[1])
        else:
            self._wire.move_to_end(number)
        return cached

    def _diffs_between(self, start: int, end: int) -> Iterable[Any]:
        """Diff messages for scans [start, end), by their stamps."""
        window = self.index.scan_ts[start:end]  # min/max: stamps need not rise with the index
        if not len(window):
            # An empty window reached `.min()` and came out as numpy's "zero-size array to
            # reduction operation minimum", which says nothing about scans or segments. The
            # diagnostic four lines below is the one worth showing, so say the same thing.
            raise RuntimeError(f"no scans in the range {start}..{end}")
        t0, t1 = float(window.min()), float(window.max())
        # By scan index: two scans can share a stamp, and the range then over-fetches.
        by_index = {
            int(obs.tags["scan_index"]): obs for obs in self.diffs.time_range(t0 - 1e-4, t1 + 1e-4)
        }
        found = [by_index[i] for i in range(start, end) if i in by_index]
        if len(found) != end - start:
            raise RuntimeError(
                f"expected {end - start} diffs for scans {start}..{end}, got {len(found)}"
            )
        return found

    @staticmethod
    def encode_segment(header: dict[str, Any], payload: bytes) -> bytes:
        """``[u32 header length][header JSON, padded to a multiple of 4][payload]``.

        The padding keeps the payload 4-byte aligned from the start of the buffer, and
        `build_segment` pads again between the int16 table and the uint32 slots
        (`slots_offset` rounds the table's `n * 6` bytes up to a multiple of 4). Both
        are load-bearing: `replay.js` takes ZERO-COPY views over this buffer --
        `new Int16Array(buffer, base, ...)` and `new Uint32Array(buffer, slotsOffset,
        ...)` -- and a typed-array view whose offset is not a multiple of its element
        size throws a RangeError outright. It does not copy on a miss; there is no miss
        to handle. Dropping either pad breaks roughly half of all segments.
        """
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        header_bytes += b" " * (-(4 + len(header_bytes)) % 4)
        return struct.pack("<I", len(header_bytes)) + header_bytes + payload
