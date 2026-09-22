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

"""The mapper's map over a recording, stored as keyframes plus add/remove diffs.

The ray tracing mapper publishes its global map as the recording replays, and
every snapshot is folded into two streams of the recording:

``voxel_keyframe``
    The whole voxel set, one PointCloud2 every ``keyframe_interval_s``.
``voxel_diff``
    One PointCloud2 per snapshot holding only the voxels it added (tag
    ``TAG_ADDED``) or removed (tag ``TAG_REMOVED``) since the previous one.

The map the recording ends on is the last keyframe plus the diffs after it. A
timeline that covers the whole recording is kept across restarts, so a later
run loads that map at once instead of mapping again.
"""

from __future__ import annotations

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


def timeline_matches(store: Any, *, voxel_size: float, keyframe_interval_s: float) -> bool:
    """True when both streams exist and were recorded with these settings."""
    names = store.list_streams()
    if DIFF_STREAM not in names or KEYFRAME_STREAM not in names:
        return False
    if store.streams[KEYFRAME_STREAM].count() == 0:
        return False
    tags = store.streams[KEYFRAME_STREAM].first().tags or {}
    wanted = {
        "voxel_size": float(voxel_size),
        "keyframe_interval_s": float(keyframe_interval_s),
        "format": FORMAT_VERSION,
    }
    return all(tags.get(key) == value for key, value in wanted.items())


def timeline_end(store: Any) -> float:
    """Stamp of the last recorded snapshot, or -inf without one."""
    diffs = store.streams[DIFF_STREAM]
    return float(diffs.last().ts) if diffs.count() else -np.inf


def final_map(store: Any, voxel_size: float) -> tuple[np.ndarray, float]:
    """The voxel centres the recorded timeline ends on, and the stamp of its last snapshot."""
    keyframe = store.streams[KEYFRAME_STREAM].last()
    keys = _pack(keyframe.data.points_f32(), voxel_size)
    last_ts = float(keyframe.ts)
    for diff in store.streams[DIFF_STREAM].after(last_ts + 1e-6):
        points = diff.data.points_f32()
        tags = diff.data.tags_u8()
        removed = _pack(points[tags == TAG_REMOVED], voxel_size)
        added = _pack(points[tags == TAG_ADDED], voxel_size)
        keys = np.union1d(np.setdiff1d(keys, removed, assume_unique=True), added)
        last_ts = float(diff.ts)
    return _centres(keys, voxel_size), last_ts
