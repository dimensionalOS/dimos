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

from __future__ import annotations

from collections import deque
from functools import reduce
import operator
import threading
import time
from typing import Any

import numpy as np
import reactivex as rx
from reactivex import Observable, operators as ops

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.mapping.relocalization.lidar.relocalize import (
    MID360,
    LidarRelocalizer,
    RelocalizeConfig,
)
from dimos.mapping.relocalization.module import Config, PremapModule
from dimos.msgs.sensor_msgs.PointCloud2 import FLAG_NAMES, PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

_SPARSE_CLOUD_GRACE_S = 10.0
_SKIP_LOG_INTERVAL_S = 5.0


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


class LidarConfig(Config):
    reloc_interval: float = 2.0
    # Skip clouds sparser than this (a mid360 sweep is ~3k points)
    min_local_points: int = 2_000
    # Rig preset to align with, see relocalize.PRESETS
    relocalize: RelocalizeConfig = MID360
    # After the first fix, refine it by ICP alone instead of searching the whole premap again
    refine_fix: bool = False


class CloudRelocalization(PremapModule):
    """Coarse FPFH+RANSAC then ICP of a windowed live cloud against a pointcloud premap.

    Subclasses supply :meth:`clouds`.
    """

    config: LidarConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._relocalizer: LidarRelocalizer | None = None
        self._first_sparse_cloud: float | None = None
        self._last_skip_log: float | None = None

    def clouds(self) -> Observable[PointCloud2]:
        """The windowed live cloud to match, at most one every ``reloc_interval``."""
        raise NotImplementedError

    @rpc
    def start(self) -> None:
        super().start()
        if self.premap is None:
            return
        # Premap preprocessing is the dominant cost, paid once here.
        self._relocalizer = LidarRelocalizer(self.premap.pointcloud, self.config.relocalize)
        self.register_disposable(
            backpressure(
                self.clouds().pipe(
                    ops.filter(lambda _: self.keep_relocalizing()),
                    ops.do_action(self._maybe_log_skip),
                    ops.filter(self._has_enough_points),
                )
            ).subscribe(self._relocalize)
        )

    def _maybe_log_skip(self, msg: PointCloud2) -> None:
        if self._has_enough_points(msg):
            self._first_sparse_cloud = None
            self._last_skip_log = None
            return
        now = time.monotonic()
        if self._first_sparse_cloud is None:
            self._first_sparse_cloud = now
        if self._last_skip_log is None or now - self._last_skip_log >= _SKIP_LOG_INTERVAL_S:
            log = (
                logger.info
                if now - self._first_sparse_cloud < _SPARSE_CLOUD_GRACE_S
                else logger.warning
            )
            log(
                "relocalize skipped: not enough local points",
                n_pts=len(msg),
                min_local_points=self.config.min_local_points,
            )
            self._last_skip_log = now

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= self.config.min_local_points

    def _relocalize(self, msg: PointCloud2) -> None:
        assert self._relocalizer is not None
        t0 = time.monotonic()
        try:
            initial_fix = self.fix if self.config.refine_fix else None
            tf = self._relocalizer.attempt(
                msg.pointcloud, self.config.world_frame, self.map_frame, initial_fix
            ).fix
        except Exception:
            logger.exception("relocalize() failed")
            return
        dt = time.monotonic() - t0
        if tf is None:
            logger.info(
                f"relocalize lidar: refused after {dt:.1f}s n_pts={len(msg)} "
                f"(below fitness_threshold={self.config.relocalize.fitness_threshold})"
            )
            return
        logger.info(f"relocalize lidar: time_cost={dt:.1f}s n_pts={len(msg)}")
        self.submit(tf, "lidar")


class LidarWindowRelocalization(CloudRelocalization):
    """Matches the last `max_frames` scans merged."""

    lidar: In[PointCloud2]

    def clouds(self) -> Observable[PointCloud2]:
        return window(
            self.lidar.observable(),
            self.config.relocalize,
            self.config.reloc_interval,
        )


class LocalMapConfig(LidarConfig):
    # Smaller carved local maps are too small a patch to match
    min_local_points: int = 20_000


class LocalMapRelocalization(CloudRelocalization):
    """Matches the ray-tracing mapper's local map, a window in space, already carved."""

    config: LocalMapConfig

    local_map: In[PointCloud2]

    def clouds(self) -> Observable[PointCloud2]:
        return self.local_map.observable().pipe(  # type: ignore[no-untyped-call]
            ops.throttle_first(self.config.reloc_interval)
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


class ContinuousConfig(LidarConfig):
    relocalize_once: bool = False
    refine_fix: bool = True
    # Fold the ray tracer's local_map into the kept live map at most this often (s)
    accumulate_interval: float = 1.0
    # The kept live map's voxel edge (m)
    live_voxel_m: float = 0.1
    # Match only kept live points within this xy distance (m) of the robot's latest local map; 0 matches them all
    match_radius_m: float = 30.0


class ContinuousRelocalization(CloudRelocalization):
    """Keeps refining ``world -> map`` against its own map of the ray tracer's live voxels.

    It keeps the live (non-premap) voxels of the ray tracer's local maps, so the premap never
    confirms itself. The first fix searches the whole premap and places it in the ray tracer;
    every later one refines the current fix by ICP and only moves ``world -> map``.
    world_frame must be the frame the ray tracer maps in, and its local_map must carry the premap bit.
    """

    config: ContinuousConfig

    local_map: In[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.live = LiveMap(self.config.live_voxel_m)
        self._center: tuple[float, float] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self.premap is None:
            return
        self.register_disposable(
            self.local_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.throttle_first(self.config.accumulate_interval))
            .subscribe(self._on_local_map)
        )

    def _on_local_map(self, cloud: PointCloud2) -> None:
        self.live.fold(cloud)
        points = cloud.points_f32().reshape(-1, 3)
        if len(points):
            # The local map is a window around the robot, so its xy middle is close enough.
            low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
            self._center = (float(low[0] + high[0]) / 2, float(low[1] + high[1]) / 2)

    def clouds(self) -> Observable[PointCloud2]:
        return rx.interval(self.config.reloc_interval).pipe(
            ops.map(
                lambda _: self.live.cloud(
                    self.config.world_frame, self._center, self.config.match_radius_m
                )
            )
        )
