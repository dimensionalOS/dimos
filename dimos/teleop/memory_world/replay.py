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

"""Voxel replay: keyframes plus per-scan add/remove diffs, stored in the recording.

Scrubbing a timeline needs the voxel map *as it was* at any moment. Rebuilding
it from raw scans is far too slow for that, so two streams are written into the
recording once:

``voxel_keyframe``
    The whole voxel set, one PointCloud2 every ``keyframe_interval_s``.
``voxel_diff``
    One PointCloud2 per lidar scan holding only the voxels that scan added
    (tag ``TAG_ADDED``) or removed (tag ``TAG_REMOVED``).

Any moment is then the nearest earlier keyframe plus the diffs up to it, and a
viewer that already shows some moment reaches a neighbouring one by applying
(or un-applying, since a diff is its own inverse with the tags swapped) a few
diffs. Voxels come from the same :class:`PackedVoxels` grid the static map is
built with, so the replayed map converges on the map the viewer shows anyway.

Run ``python -m dimos.teleop.memory_world.replay <recording.db>`` to build the
streams ahead of time; the module builds them on first use otherwise.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterable
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
FORMAT_VERSION = 1


class ReplayGrid:
    """A column-carving voxel set whose removals have hysteresis.

    Same packed sorted-key layout as :class:`PackedVoxels`, but a voxel in a
    column the current scan touches is only dropped after ``remove_after``
    consecutive scans that touched its column without hitting it. With
    ``remove_after=1`` this is plain column carving, which on a sweeping lidar
    flickers every sparsely-sampled wall in and out at ~13k edits per scan;
    a few misses of grace keeps the diffs small while people and doors still
    disappear within a fraction of a second.
    """

    def __init__(self, voxel_size: float, remove_after: int = 1) -> None:
        self.voxel_size = voxel_size
        self.remove_after = max(1, int(remove_after))
        self.keys = np.empty(0, dtype=np.int64)
        self.health = np.empty(0, dtype=np.int16)

    def add_scan(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Merge one world-frame scan; returns the (added, removed) sorted keys."""
        empty = np.empty(0, dtype=np.int64)
        if not len(points):
            return empty, empty
        vox = np.floor(points / np.float32(self.voxel_size)).astype(np.int64)
        if np.abs(vox).max(initial=0) >= KEY_OFFSET:
            raise ValueError(f"point outside +-{KEY_OFFSET * self.voxel_size:.0f} m packed range")
        vox += KEY_OFFSET
        new = np.unique((vox[:, 0] << (2 * FIELD_BITS)) | (vox[:, 1] << FIELD_BITS) | vox[:, 2])

        keys, health = self.keys, self.health
        # every existing voxel in a column the scan touched is a hit or a miss
        columns = np.unique(new >> FIELD_BITS)
        starts = np.searchsorted(keys, columns << FIELD_BITS, side="left")
        ends = np.searchsorted(keys, (columns + 1) << FIELD_BITS, side="left")
        delta = np.zeros(len(keys) + 1, dtype=np.int32)
        np.add.at(delta, starts, 1)
        np.add.at(delta, ends, -1)
        in_column = np.cumsum(delta[:-1]) > 0

        position = np.searchsorted(keys, new)
        exists = position < len(keys)
        exists[exists] = keys[position[exists]] == new[exists]
        hit = np.zeros(len(keys), dtype=bool)
        hit[position[exists]] = True
        health = health.copy()
        health[hit] = self.remove_after
        missed = in_column & ~hit
        health[missed] -= 1
        gone = missed & (health <= 0)

        inserted = new[~exists]
        kept_keys, kept_health = keys[~gone], health[~gone]
        where = np.searchsorted(kept_keys, inserted)
        self.keys = np.insert(kept_keys, where, inserted)
        self.health = np.insert(kept_health, where, np.int16(self.remove_after))
        return inserted, keys[gone]

    def centres(self, keys: np.ndarray | None = None) -> np.ndarray:
        """Voxel centres, (N, 3) float32, of *keys* (default: the whole set)."""
        k = self.keys if keys is None else keys
        vox = np.empty((len(k), 3), dtype=np.float32)
        vox[:, 0] = (k >> (2 * FIELD_BITS)) - KEY_OFFSET
        vox[:, 1] = ((k >> FIELD_BITS) & FIELD_MASK) - KEY_OFFSET
        vox[:, 2] = (k & FIELD_MASK) - KEY_OFFSET
        return (vox + np.float32(0.5)) * np.float32(self.voxel_size)


@dataclass
class ReplayStats:
    scans: int = 0
    keyframes: int = 0
    added: int = 0
    removed: int = 0
    final_voxels: int = 0
    seconds: float = 0.0


def build_replay_streams(
    store: Any,
    *,
    lidar_stream_name: str,
    to_world: Callable[[Any], PointCloud2 | None],
    voxel_size: float,
    keyframe_interval_s: float = 5.0,
    remove_after: int = 1,
    diff_stream_name: str = DIFF_STREAM,
    keyframe_stream_name: str = KEYFRAME_STREAM,
    dry_run: bool = False,
) -> ReplayStats:
    """Write the keyframe and diff streams for every scan of the lidar stream.

    ``to_world`` turns a lidar observation into a world-frame cloud (or None to
    skip it). Existing streams of the same names are replaced. ``dry_run``
    only gathers the statistics.
    """
    started = time.monotonic()
    stream_tags = {
        "voxel_size": float(voxel_size),
        "lidar_stream": lidar_stream_name,
        "keyframe_interval_s": float(keyframe_interval_s),
        "remove_after": int(remove_after),
        "format": FORMAT_VERSION,
    }
    diffs = keyframes = None
    if not dry_run:
        for name in (diff_stream_name, keyframe_stream_name):
            if name in store.list_streams():
                store.delete_stream(name)
        diffs = store.stream(diff_stream_name, PointCloud2)
        keyframes = store.stream(keyframe_stream_name, PointCloud2)

    grid = ReplayGrid(voxel_size, remove_after)
    stats = ReplayStats()
    last_keyframe_ts: float | None = None
    for obs in store.streams[lidar_stream_name]:
        cloud = to_world(obs)
        points = cloud.points_f32() if cloud is not None else np.zeros((0, 3), np.float32)
        added, removed = grid.add_scan(points)
        stats.added += len(added)
        stats.removed += len(removed)
        ts = float(obs.ts)
        take_keyframe = last_keyframe_ts is None or ts - last_keyframe_ts >= keyframe_interval_s
        if take_keyframe:
            last_keyframe_ts = ts
            stats.keyframes += 1
        if diffs is not None and keyframes is not None:
            centres = np.concatenate([grid.centres(added), grid.centres(removed)])
            tags = np.concatenate(
                [
                    np.full(len(added), TAG_ADDED, np.uint8),
                    np.full(len(removed), TAG_REMOVED, np.uint8),
                ]
            )
            diffs.append(
                PointCloud2.from_numpy(centres, frame_id="world", timestamp=ts, tags=tags),
                ts=ts,
                tags={**stream_tags, "scan_index": stats.scans},
            )
            if take_keyframe:
                keyframes.append(
                    PointCloud2.from_numpy(grid.centres(), frame_id="world", timestamp=ts),
                    ts=ts,
                    tags={**stream_tags, "scan_index": stats.scans},
                )
        stats.scans += 1
        if stats.scans % 200 == 0:
            logger.info(
                "replay build: %d scans, %d voxels, +%d/-%d so far",
                stats.scans,
                len(grid.keys),
                stats.added,
                stats.removed,
            )
    stats.final_voxels = len(grid.keys)
    stats.seconds = time.monotonic() - started
    return stats


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
    def available(store: Any, *, voxel_size: float, lidar_stream_name: str) -> bool:
        """True when both streams exist and were built for this grid and lidar."""
        names = store.list_streams()
        if DIFF_STREAM not in names or KEYFRAME_STREAM not in names:
            return False
        first = store.streams[KEYFRAME_STREAM].first()
        tags = first.tags or {}
        return (
            tags.get("format") == FORMAT_VERSION
            and abs(float(tags.get("voxel_size", 0.0)) - voxel_size) < 1e-9
            and tags.get("lidar_stream") == lidar_stream_name
        )

    def _load_index(self) -> ReplayIndex:
        first = self.keyframes.first()
        tags = dict(first.tags or {})
        voxel_size = float(tags["voxel_size"])
        scan_ts = np.array([float(obs.ts) for obs in self.diffs], dtype=np.float64)
        keyframe_scan: list[int] = []
        keyframe_ts: list[float] = []
        low = np.full(3, np.inf)
        high = np.full(3, -np.inf)
        for obs in self.keyframes:
            keyframe_scan.append(int(obs.tags["scan_index"]))
            keyframe_ts.append(float(obs.ts))
            points = obs.data.points_f32()
            if len(points):
                low = np.minimum(low, points.min(axis=0))
                high = np.maximum(high, points.max(axis=0))
        if not np.all(np.isfinite(low)):
            low = high = np.zeros(3)
        centre = np.floor((low + high) / 2 / voxel_size).astype(np.int64)
        return ReplayIndex(
            voxel_size=voxel_size,
            scan_ts=scan_ts,
            keyframe_scan=np.array(keyframe_scan, dtype=np.int64),
            keyframe_ts=np.array(keyframe_ts, dtype=np.float64),
            origin=(int(centre[0]), int(centre[1]), int(centre[2])),
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


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Build the voxel replay streams of a recording")
    parser.add_argument("store_path")
    parser.add_argument("--lidar-stream", default="lidar")
    parser.add_argument("--tf-stream", default="tf")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--keyframe-interval", type=float, default=5.0)
    parser.add_argument("--tf-tolerance", type=float, default=0.1)
    parser.add_argument("--remove-after", type=int, default=1, help="misses before a voxel goes")
    parser.add_argument("--dry-run", action="store_true", help="only report the statistics")
    args = parser.parse_args(argv)

    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.teleop.memory_world.recording import open_recording
    from dimos.teleop.memory_world.tf_tree import TfTree

    store = open_recording(args.store_path)
    tree = TfTree.from_stream(store.streams[args.tf_stream])

    def to_world(obs: Any) -> PointCloud2 | None:
        frame = str(getattr(obs.data, "frame_id", "") or "").lstrip("/")
        matrix = tree.lookup(args.world_frame, frame, float(obs.ts), args.tf_tolerance)
        if matrix is None:
            return None
        transformed: PointCloud2 = obs.data.transform(Transform.from_matrix(matrix))
        return transformed

    stats = build_replay_streams(
        store,
        lidar_stream_name=args.lidar_stream,
        to_world=to_world,
        voxel_size=args.voxel_size,
        keyframe_interval_s=args.keyframe_interval,
        remove_after=args.remove_after,
        dry_run=args.dry_run,
    )
    print(
        f"{stats.scans} scans, {stats.keyframes} keyframes, +{stats.added} / -{stats.removed} voxel "
        f"edits, {stats.final_voxels} voxels at the end, {stats.seconds:.1f} s"
    )


if __name__ == "__main__":
    main()
