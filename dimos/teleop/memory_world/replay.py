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

"""Voxel replay: keyframes plus add/remove diffs, stored in the recording.

Scrubbing a timeline needs the voxel map *as it was* at any moment. The ray
tracing mapper publishes its global map as the recording replays, and every
snapshot is folded into two streams of the recording:

``voxel_keyframe``
    The whole voxel set, one PointCloud2 every ``keyframe_interval_s``.
``voxel_diff``
    One PointCloud2 per snapshot holding only the voxels it added (tag
    ``TAG_ADDED``) or removed (tag ``TAG_REMOVED``) since the previous one.

Any moment is then the nearest earlier keyframe plus the diffs up to it, and a
viewer that already shows some moment reaches a neighbouring one by applying
(or un-applying, since a diff is its own inverse with the tags swapped) a few
diffs. A timeline that covers the whole recording is kept across restarts.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
import gzip
import json
import struct
import time
from typing import Any

import numpy as np

from dimos.mapping.voxels.keys import FIELD_BITS, FIELD_MASK, KEY_OFFSET
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DIFF_STREAM = "voxel_diff"
KEYFRAME_STREAM = "voxel_keyframe"
TAG_ADDED = 1
TAG_REMOVED = 2
FORMAT_VERSION = 3


def _pack(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Sorted unique packed keys of the voxels the points fall in."""
    vox = np.floor(points / np.float32(voxel_size)).astype(np.int64)
    if np.abs(vox).max(initial=0) >= KEY_OFFSET:
        raise ValueError(f"point outside +-{KEY_OFFSET * voxel_size:.0f} m packed range")
    vox += KEY_OFFSET
    return np.unique((vox[:, 0] << (2 * FIELD_BITS)) | (vox[:, 1] << FIELD_BITS) | vox[:, 2])


def _centres(keys: np.ndarray, voxel_size: float) -> np.ndarray:
    """(N, 3) float32 voxel centres of packed keys."""
    vox = np.stack(
        [
            (keys >> (2 * FIELD_BITS)) & FIELD_MASK,
            (keys >> FIELD_BITS) & FIELD_MASK,
            keys & FIELD_MASK,
        ],
        axis=1,
    ).astype(np.float32)
    vox -= np.float32(KEY_OFFSET)
    return (vox + np.float32(0.5)) * np.float32(voxel_size)


class ReplayRecorder:
    """Folds successive map snapshots into fresh keyframe and diff streams."""

    def __init__(
        self,
        store: Any,
        *,
        voxel_size: float,
        keyframe_interval_s: float,
        diff_stream_name: str = DIFF_STREAM,
        keyframe_stream_name: str = KEYFRAME_STREAM,
    ) -> None:
        for name in (diff_stream_name, keyframe_stream_name):
            if name in store.list_streams():
                store.delete_stream(name)
        self.diffs = store.stream(diff_stream_name, PointCloud2)
        self.keyframes = store.stream(keyframe_stream_name, PointCloud2)
        self.voxel_size = voxel_size
        self.keyframe_interval_s = keyframe_interval_s
        self.tags: dict[str, Any] = {
            "voxel_size": float(voxel_size),
            "keyframe_interval_s": float(keyframe_interval_s),
            "format": FORMAT_VERSION,
            "built_at": round(time.time(), 3),
        }
        self.scans = 0
        self._keys = np.empty(0, dtype=np.int64)
        self._last_keyframe_ts: float | None = None

    def add_snapshot(self, points: np.ndarray, ts: float) -> bool:
        """Record the map as of *ts*; returns whether a keyframe was written."""
        keys = _pack(np.asarray(points, dtype=np.float32)[:, :3], self.voxel_size)
        added = np.setdiff1d(keys, self._keys, assume_unique=True)
        removed = np.setdiff1d(self._keys, keys, assume_unique=True)
        self._keys = keys
        if self.scans == 0:
            centre = np.floor(_centres(keys, self.voxel_size).mean(axis=0) / self.voxel_size)
            if len(keys) == 0:
                centre = np.zeros(3)
            self.tags["origin"] = [int(v) for v in centre]
        take_keyframe = (
            self._last_keyframe_ts is None
            or ts - self._last_keyframe_ts >= self.keyframe_interval_s
        )
        centres = np.concatenate(
            [_centres(added, self.voxel_size), _centres(removed, self.voxel_size)]
        )
        tags = np.concatenate(
            [np.full(len(added), TAG_ADDED, np.uint8), np.full(len(removed), TAG_REMOVED, np.uint8)]
        )
        self.diffs.append(
            PointCloud2.from_numpy(centres, frame_id="world", timestamp=ts, tags=tags),
            ts=ts,
            tags={**self.tags, "scan_index": self.scans},
        )
        if take_keyframe:
            self._last_keyframe_ts = ts
            self.keyframes.append(
                PointCloud2.from_numpy(
                    _centres(keys, self.voxel_size), frame_id="world", timestamp=ts
                ),
                ts=ts,
                tags={**self.tags, "scan_index": self.scans},
            )
        self.scans += 1
        return take_keyframe


# ---- reading ---------------------------------------------------------------


@dataclass
class ReplayIndex:
    """Everything a viewer needs to seek: scan stamps and where the keyframes sit."""

    voxel_size: float
    scan_ts: np.ndarray  # (S,) float64, one per diff message, ascending
    keyframe_scan: np.ndarray  # (K,) int, scan index each keyframe was taken after
    keyframe_ts: np.ndarray  # (K,) float64
    origin: tuple[int, int, int]  # voxel index the int16 wire coordinates are relative to
    stream_tags: dict[str, Any] = field(default_factory=dict)

    def extend(self, ts: float, keyframe: bool) -> None:
        """Append one more scan, optionally a keyframe taken after it."""
        self.scan_ts = np.append(self.scan_ts, float(ts))
        if keyframe:
            self.keyframe_scan = np.append(self.keyframe_scan, len(self.scan_ts) - 1)
            self.keyframe_ts = np.append(self.keyframe_ts, float(ts))

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
        self._segments: dict[int, tuple[dict[str, Any], bytes]] = {}
        self._encoded: dict[int, tuple[bytes, bytes]] = {}

    @staticmethod
    def matches(store: Any, *, voxel_size: float, keyframe_interval_s: float) -> bool:
        """True when both streams exist and were recorded with these settings."""
        names = store.list_streams()
        if DIFF_STREAM not in names or KEYFRAME_STREAM not in names:
            return False
        tags = store.streams[KEYFRAME_STREAM].first().tags or {}
        wanted = {
            "voxel_size": float(voxel_size),
            "keyframe_interval_s": float(keyframe_interval_s),
            "format": FORMAT_VERSION,
        }
        return all(tags.get(key) == value for key, value in wanted.items())

    def covers(self, ts: float) -> bool:
        """True when the recorded timeline reaches *ts*."""
        return len(self.index.scan_ts) > 0 and float(self.index.scan_ts[-1]) >= ts

    def extend(self, ts: float, keyframe: bool) -> None:
        """Take in one more recorded snapshot; the open segment is rebuilt on demand."""
        self.index.extend(ts, keyframe)
        last = len(self.index.keyframe_scan) - 1
        self._segments.pop(last, None)
        self._encoded.pop(last, None)
        if keyframe:
            self._segments.pop(last - 1, None)
            self._encoded.pop(last - 1, None)

    def _load_index(self) -> ReplayIndex:
        tags = dict(self.keyframes.first().tags or {})
        scan_ts = np.array([float(obs.ts) for obs in self.diffs], dtype=np.float64)
        keyframe_scan = [int(obs.tags["scan_index"]) for obs in self.keyframes]
        keyframe_ts = [float(obs.ts) for obs in self.keyframes]
        origin = [int(v) for v in tags["origin"]]
        return ReplayIndex(
            voxel_size=float(tags["voxel_size"]),
            scan_ts=scan_ts,
            keyframe_scan=np.array(keyframe_scan, dtype=np.int64),
            keyframe_ts=np.array(keyframe_ts, dtype=np.float64),
            origin=(origin[0], origin[1], origin[2]),
            stream_tags=tags,
        )

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
        """Header and payload of one segment; built once, then cached.

        Payload: ``int16 xyz`` for every slot of the position table (keyframe
        voxels first, then voxels the diffs introduce, in order of appearance),
        then ``uint32`` slot per diff entry of every scan in order, then one
        ``uint8`` op per entry.
        """
        cached = self._segments.get(number)
        if cached is not None:
            return cached
        start, end = self.index.segment_scans(number)
        keyframe = self.keyframes.at(float(self.index.keyframe_ts[number]), tolerance=1e-3).first()
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
        self._segments[number] = (header, payload)
        return header, payload

    def encoded_segment(self, number: int) -> tuple[bytes, bytes]:
        """The wire form of a segment, raw and gzipped, each built once."""
        cached = self._encoded.get(number)
        if cached is None:
            raw = self.encode_segment(*self.segment(number))
            cached = (raw, gzip.compress(raw, compresslevel=6))
            self._encoded[number] = cached
        return cached

    def _diffs_between(self, start: int, end: int) -> Iterable[Any]:
        """Diff messages for scans [start, end), by their stamps."""
        t0 = float(self.index.scan_ts[start])
        t1 = float(self.index.scan_ts[end - 1]) if end - 1 < len(self.index.scan_ts) else t0
        found = list(self.diffs.time_range(t0 - 1e-4, t1 + 1e-4))
        if len(found) != end - start:
            raise RuntimeError(
                f"expected {end - start} diffs for scans {start}..{end}, got {len(found)}"
            )
        return found

    @staticmethod
    def encode_segment(header: dict[str, Any], payload: bytes) -> bytes:
        """``[u32 header length][header JSON, padded to a multiple of 4][payload]``.

        The padding keeps the int16 table and the uint32 slots aligned for
        typed-array views on the viewer without copying (the table's byte
        length is a multiple of 6, so the slots follow it 2-byte aligned; the
        viewer copies them when 4-byte alignment is not met).
        """
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        header_bytes += b" " * (-(4 + len(header_bytes)) % 4)
        return struct.pack("<I", len(header_bytes)) + header_bytes + payload


# ---- command line ------------------------------------------------------------
