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

"""The live clouds relocalization matches: a window of lidar scans, or the ray tracer's live voxels."""

from __future__ import annotations

from collections import deque
from functools import reduce
import operator
import threading

import numpy as np
from reactivex import Observable, operators as ops

from dimos.mapping.relocalization.lidar.relocalize import RelocalizeConfig
from dimos.msgs.sensor_msgs.PointCloud2 import FLAG_NAMES, PointCloud2


def window(
    scans: Observable[PointCloud2], cfg: RelocalizeConfig, interval: float
) -> Observable[PointCloud2]:
    """The last ``max_frames`` scans merged, at most one merge every ``interval`` s."""
    held: deque[PointCloud2] = deque(maxlen=cfg.max_frames)
    return scans.pipe(
        ops.do_action(held.append),
        ops.throttle_first(interval),
        ops.filter(lambda _: len(held) >= cfg.min_frames),
        ops.map(lambda _: reduce(operator.add, held)),
    )


_PREMAP_BIT = 1 << FLAG_NAMES.index("premap")

# A voxel key's three axes, packed into one int64 at this many bits each.
_KEY_BITS = 21
_KEY_OFFSET = 1 << (_KEY_BITS - 1)


def _pack(points: np.ndarray, voxel: float) -> np.ndarray:
    keys = np.floor(points / voxel).astype(np.int64) + _KEY_OFFSET
    packed: np.ndarray = (keys[:, 0] << (2 * _KEY_BITS)) | (keys[:, 1] << _KEY_BITS) | keys[:, 2]
    return packed


class LiveMap:
    """The live (non-premap) part of the ray tracer's local maps, kept one point per voxel."""

    def __init__(self, voxel: float) -> None:
        self.voxel = voxel
        self._keys = np.empty(0, dtype=np.int64)
        self._points = np.empty((0, 3), dtype=np.float32)
        self._lock = threading.Lock()

    def __len__(self) -> int:
        return len(self._keys)

    def fold(self, local_map: PointCloud2) -> None:
        flags = local_map.flags_u32()
        if flags is None:
            raise ValueError(
                "local_map has no flags, so premap voxels cannot be told apart: run the ray tracer with a premap"
            )
        points = local_map.points_f32().reshape(-1, 3)[(flags & _PREMAP_BIT) == 0]
        keys = _pack(points, self.voxel)
        with self._lock:
            all_keys = np.concatenate([keys, self._keys])
            all_points = np.concatenate([points, self._points])
            # First occurrence wins, so a voxel's newest point is kept.
            self._keys, first = np.unique(all_keys, return_index=True)
            self._points = all_points[first]

    def cloud(
        self, frame_id: str, center: tuple[float, float] | None, radius: float
    ) -> PointCloud2:
        """The kept points, only those within ``radius`` (xy) of ``center`` when both are given."""
        with self._lock:
            points = self._points
        if center is not None and radius > 0:
            dx = points[:, 0] - center[0]
            dy = points[:, 1] - center[1]
            points = points[dx * dx + dy * dy <= radius * radius]
        return PointCloud2.from_numpy(points, frame_id=frame_id)
