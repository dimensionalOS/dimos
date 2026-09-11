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
    The whole voxel set, one PointCloud2 every ``keyframe_interval_s`` and one
    at the last scan, so the final keyframe is the finished map.
``voxel_diff``
    One PointCloud2 per lidar scan holding only the voxels that scan added
    (tag ``TAG_ADDED``) or removed (tag ``TAG_REMOVED``).

Any moment is then the nearest earlier keyframe plus the diffs up to it, and a
viewer that already shows some moment reaches a neighbouring one by applying
(or un-applying, since a diff is its own inverse with the tags swapped) a few
diffs. The map is the ray-traced one (:class:`RayTracedGrid`, over dimos's
``VoxelRayMapper``): every scan casts rays from the sensor, so a voxel a later
scan sees through is cleared, and people and doors that moved disappear from
the map instead of leaving a shell. The final keyframe doubles as the static
map the viewer shows outside the timeline.

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

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.mapping.voxels.keys import FIELD_BITS, FIELD_MASK, KEY_OFFSET
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DIFF_STREAM = "voxel_diff"
KEYFRAME_STREAM = "voxel_keyframe"
TAG_ADDED = 1
TAG_REMOVED = 2
FORMAT_VERSION = 1
# Streams built another way (column carving, before ray tracing) are rebuilt.
BUILDER = "raytrace:healthy"


@dataclass(frozen=True)
class SensorScan:
    """One lidar scan in its sensor frame, and where that sensor was."""

    points: np.ndarray  # (N, 3) float32, sensor frame
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]  # (x, y, z, w)


def sensor_scan(points: np.ndarray, world_from_sensor: np.ndarray, in_world: bool) -> SensorScan:
    """A scan as :class:`SensorScan`, given the sensor's pose as a 4x4 matrix.

    A scan stored already in the world frame (``in_world``) is moved back into
    the sensor frame first: the rays have to start at the sensor.
    """
    from dimos.teleop.memory_world.tf_tree import quaternion_from_matrix

    points = np.ascontiguousarray(points, dtype=np.float32)
    if in_world and len(points):
        sensor_from_world = np.linalg.inv(world_from_sensor)
        points = np.ascontiguousarray(
            (points @ sensor_from_world[:3, :3].T + sensor_from_world[:3, 3]).astype(np.float32)
        )
    return SensorScan(
        points=points,
        position=tuple(float(v) for v in world_from_sensor[:3, 3]),  # type: ignore[arg-type]
        orientation=quaternion_from_matrix(world_from_sensor[:3, :3]),
    )


def pack_keys(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Sorted, unique packed voxel keys of world points (the PackedVoxels layout)."""
    if not len(points):
        return np.empty(0, dtype=np.int64)
    vox = np.floor(points / np.float32(voxel_size)).astype(np.int64)
    if np.abs(vox).max(initial=0) >= KEY_OFFSET:
        raise ValueError(f"point outside +-{KEY_OFFSET * voxel_size:.0f} m packed range")
    vox += KEY_OFFSET
    return np.unique((vox[:, 0] << (2 * FIELD_BITS)) | (vox[:, 1] << FIELD_BITS) | vox[:, 2])


def unpack_centres(keys: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel centres, (N, 3) float32, of packed keys."""
    vox = np.empty((len(keys), 3), dtype=np.float32)
    vox[:, 0] = (keys >> (2 * FIELD_BITS)) - KEY_OFFSET
    vox[:, 1] = ((keys >> FIELD_BITS) & FIELD_MASK) - KEY_OFFSET
    vox[:, 2] = (keys & FIELD_MASK) - KEY_OFFSET
    return (vox + np.float32(0.5)) * np.float32(voxel_size)


class RayTracedGrid:
    """The voxel set of a ray-traced map as packed keys, with a diff per scan.

    Each scan goes into a :class:`VoxelRayMapper`, which casts a ray from the
    sensor to every return and lowers the health of the voxels it passes
    through, so a wall that was really a person walking past is cleared by the
    scans that later see the space empty. A scan can only change voxels within
    ``max_range`` of the sensor, so the diff is taken by comparing the mapper's
    voxels inside that cylinder with the ones held for it before the scan.

    Every healthy voxel is kept, as in the ray-tracing module's own global
    map. The mapper's ``support_min`` gate (a voxel must have that many
    healthy neighbours to be shown) is meant for its live local map; on a
    sparse lidar it hides two thirds of a building's walls, so it is off here.
    """

    def __init__(
        self, voxel_size: float, max_range: float, support_min: int = 0, **mapper_kwargs: Any
    ) -> None:
        self.voxel_size = voxel_size
        self.max_range = max_range
        self.mapper = VoxelRayMapper(
            voxel_size=voxel_size, max_range=max_range, support_min=support_min, **mapper_kwargs
        )
        self.keys = np.empty(0, dtype=np.int64)
        # Centres of `keys`, kept in step, so the cylinder test is a lookup.
        self._centres = np.empty((0, 3), dtype=np.float32)

    def _near(self, position: tuple[float, float, float]) -> np.ndarray:
        """Mask of the held keys inside the cylinder a scan from *position* can touch."""
        dx = self._centres[:, 0] - position[0]
        dy = self._centres[:, 1] - position[1]
        near_xy = dx * dx + dy * dy <= self.max_range**2
        near_z = np.abs(self._centres[:, 2] - position[2]) <= self.max_range
        return np.asarray(near_xy & near_z, dtype=bool)

    def add_scan(self, scan: SensorScan) -> tuple[np.ndarray, np.ndarray]:
        """Fold one scan into the map; returns the (added, removed) sorted keys."""
        self.mapper.add_frame(scan.points, scan.position, scan.orientation)
        x, y, z = scan.position
        now = pack_keys(
            self.mapper.local_map(
                scan.position, self.max_range, z - self.max_range, z + self.max_range
            ),
            self.voxel_size,
        )
        near = self._near(scan.position)
        before = self.keys[near]
        added = np.setdiff1d(now, before, assume_unique=True)
        removed = np.setdiff1d(before, now, assume_unique=True)
        if len(removed):
            # `keys` is sorted and `removed` is a subset of it: a merge, not a scan.
            keep = np.ones(len(self.keys), dtype=bool)
            keep[np.searchsorted(self.keys, removed)] = False
            self.keys = self.keys[keep]
            self._centres = self._centres[keep]
        if len(added):
            at = np.searchsorted(self.keys, added)
            self.keys = np.insert(self.keys, at, added)
            self._centres = np.insert(
                self._centres, at, unpack_centres(added, self.voxel_size), axis=0
            )
        return added, removed

    def centres(self, keys: np.ndarray | None = None) -> np.ndarray:
        """Voxel centres, (N, 3) float32, of *keys* (default: the whole set)."""
        return self._centres if keys is None else unpack_centres(keys, self.voxel_size)


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
    to_scan: Callable[[Any], SensorScan | None],
    voxel_size: float,
    max_range: float = 20.0,
    keyframe_interval_s: float = 5.0,
    diff_stream_name: str = DIFF_STREAM,
    keyframe_stream_name: str = KEYFRAME_STREAM,
    dry_run: bool = False,
) -> ReplayStats:
    """Write the keyframe and diff streams for every scan of the lidar stream.

    ``to_scan`` turns a lidar observation into a :class:`SensorScan` (or None
    to skip it). Existing streams of the same names are replaced. ``dry_run``
    only gathers the statistics.
    """
    started = time.monotonic()
    stream_tags = {
        "voxel_size": float(voxel_size),
        "max_range": float(max_range),
        "lidar_stream": lidar_stream_name,
        "keyframe_interval_s": float(keyframe_interval_s),
        "builder": BUILDER,
        "format": FORMAT_VERSION,
    }
    diffs = keyframes = None
    if not dry_run:
        for name in (diff_stream_name, keyframe_stream_name):
            if name in store.list_streams():
                store.delete_stream(name)
        diffs = store.stream(diff_stream_name, PointCloud2)
        keyframes = store.stream(keyframe_stream_name, PointCloud2)

    grid = RayTracedGrid(voxel_size, max_range)
    stats = ReplayStats()
    last_keyframe_ts: float | None = None
    total = store.streams[lidar_stream_name].count()
    for obs in store.streams[lidar_stream_name]:
        scan = to_scan(obs)
        if scan is not None:
            added, removed = grid.add_scan(scan)
        else:
            added = removed = np.empty(0, dtype=np.int64)
        stats.added += len(added)
        stats.removed += len(removed)
        ts = float(obs.ts)
        # The last scan is always a keyframe: that is the finished map.
        take_keyframe = (
            last_keyframe_ts is None
            or ts - last_keyframe_ts >= keyframe_interval_s
            or stats.scans == total - 1
        )
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


def accumulate_scans(
    stream: Any, to_world: Callable[[Any], Any], voxel_size: float, n_scans: int
) -> np.ndarray | None:
    """*n_scans* scans spread over *stream* voxelised into one (N, 3) cloud, no clearing.

    ``to_world`` maps a lidar observation to one in the world frame (None to
    skip it). ``n_scans <= 0`` uses every scan. This is the plain accumulation
    used when no ray-traced replay is built.
    """
    from dimos.mapping.voxels.module import VoxelMapTransformer
    from dimos.memory.transform import FnTransformer, throttle

    first, last = stream.first(), stream.last()
    span = max(float(last.ts) - float(first.ts), 1e-3)

    def placed(obs: Any) -> Any:
        cloud = to_world(obs)
        if cloud is None:
            return None
        return cloud if cloud is obs else obs.derive(data=cloud)

    pipeline = stream if n_scans <= 0 else stream.transform(throttle(span / n_scans))
    result = (
        pipeline.transform(FnTransformer(placed))
        .transform(VoxelMapTransformer(emit_every=0, voxel_size=voxel_size))
        .last()
    )
    if result is None or result.data is None:
        return None
    xyz, _ = result.data.as_numpy()
    return None if xyz is None else np.asarray(xyz, dtype=np.float32)


def frame_positions(
    stamps: Iterable[float], pose_at: Callable[[float], np.ndarray | None]
) -> list[list[float]]:
    """A frame's position at each stamp, from *pose_at* (world_T_frame or None).

    Gaps repeat the previous position, so a viewer following the robot along
    the timeline never jumps to the origin.
    """
    positions: list[list[float]] = []
    last = [0.0, 0.0, 0.0]
    for ts in stamps:
        matrix = pose_at(float(ts))
        if matrix is not None:
            last = [round(float(v), 3) for v in matrix[:3, 3]]
        positions.append(last)
    return positions


def stamped_positions(observations: Iterable[Any]) -> list[list[float]]:
    """The pose stamped on each observation, gaps repeating the previous one."""
    positions: list[list[float]] = []
    last = [0.0, 0.0, 0.0]
    for obs in observations:
        pose = getattr(obs, "pose_tuple", None)
        if pose is not None:
            last = [round(float(v), 3) for v in pose[:3]]
        positions.append(last)
    return positions


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
            and tags.get("builder") == BUILDER
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
    parser.add_argument("--max-range", type=float, default=20.0, help="ray length limit, metres")
    parser.add_argument("--dry-run", action="store_true", help="only report the statistics")
    args = parser.parse_args(argv)

    from dimos.teleop.memory_world.recording import open_recording
    from dimos.teleop.memory_world.tf_tree import TfTree

    store = open_recording(args.store_path)
    tree = TfTree.from_stream(store.streams[args.tf_stream])

    def to_scan(obs: Any) -> SensorScan | None:
        # Scans must be in their sensor frame here; a world-aligned stream
        # needs the module, which knows a frame to cast the rays from.
        frame = str(getattr(obs.data, "frame_id", "") or "").lstrip("/")
        matrix = tree.lookup(args.world_frame, frame, float(obs.ts), args.tf_tolerance)
        if matrix is None:
            return None
        return sensor_scan(obs.data.points_f32(), matrix, in_world=False)

    stats = build_replay_streams(
        store,
        lidar_stream_name=args.lidar_stream,
        to_scan=to_scan,
        voxel_size=args.voxel_size,
        max_range=args.max_range,
        keyframe_interval_s=args.keyframe_interval,
        dry_run=args.dry_run,
    )
    print(
        f"{stats.scans} scans, {stats.keyframes} keyframes, +{stats.added} / -{stats.removed} voxel "
        f"edits, {stats.final_voxels} voxels at the end, {stats.seconds:.1f} s"
    )


if __name__ == "__main__":
    main()
