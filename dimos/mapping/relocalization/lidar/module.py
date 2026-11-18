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

from dimos_lcm.std_msgs import Bool
import numpy as np
import reactivex as rx
from reactivex import Observable, operators as ops

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.lidar.relocalize import (
    MID360,
    LidarRelocalizer,
    RelocalizeConfig,
)
from dimos.mapping.relocalization.module import Config, PremapModule
from dimos.msgs.geometry_msgs.Transform import Transform
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

    def near(self) -> Transform | None:
        """The fix to refine from instead of searching the whole premap, or None to search."""
        return None

    def _relocalize(self, msg: PointCloud2) -> None:
        assert self._relocalizer is not None
        t0 = time.monotonic()
        try:
            tf = self._relocalizer.attempt(
                msg.pointcloud, self.config.world_frame, self.map_frame, self.near()
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


class DiffConfig(LidarConfig):
    relocalize_once: bool = False


class DiffRelocalization(CloudRelocalization):
    """Matches its own copy of the live map, kept from the ray tracer's diffs without the premap's voxels.

    The first fix searches the whole premap. Every later one refines the current
    fix by ICP, so the premap stays where the first fix put it in the ray tracer
    and only ``world -> map`` moves.
    """

    config: DiffConfig

    map_update: In[PointCloud2]
    # The ray tracer's keyframes: its whole map, which the diffs carry on from.
    global_map: In[PointCloud2]
    cloud_keyframe_request: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Live voxel centers (no premap-only voxel), and the stamp of the keyframe they carry on from.
        self._live: set[tuple[float, float, float]] = set()
        self._keyframe_ts: float | None = None
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        if self.premap is None:
            return
        self.register_disposable(self.global_map.observable().subscribe(self.on_keyframe))  # type: ignore[no-untyped-call]
        self.register_disposable(self.map_update.observable().subscribe(self.on_diff))  # type: ignore[no-untyped-call]
        # The diffs since the ray tracer started may be gone, so start from its whole map.
        self.cloud_keyframe_request.publish(Bool(data=True))

    def on_keyframe(self, cloud: PointCloud2) -> None:
        live = {tuple(p) for p in cloud.points_f32()[~_premap(cloud)]}
        with self._lock:
            self._live = live
            self._keyframe_ts = cloud.ts

    def on_diff(self, cloud: PointCloud2) -> None:
        if self._keyframe_ts is not None and cloud.ts is not None and cloud.ts < self._keyframe_ts:
            return  # the keyframe already holds it
        change = cloud.change_u8()
        if change is None:
            logger.warning("map_update without a change field, dropped")
            return
        points, premap = cloud.points_f32(), _premap(cloud)
        added = {tuple(p) for p in points[(change != 0) & ~premap]}
        # Removed voxels, and ones only the premap now holds.
        gone = {tuple(p) for p in points[(change == 0) | premap]}
        with self._lock:
            self._live -= gone
            self._live |= added

    def live_cloud(self) -> PointCloud2:
        with self._lock:
            points = np.array(list(self._live), dtype=np.float32).reshape(-1, 3)
        return PointCloud2.from_numpy(points, frame_id=self.config.world_frame)

    def clouds(self) -> Observable[PointCloud2]:
        return rx.interval(self.config.reloc_interval).pipe(ops.map(lambda _: self.live_cloud()))

    def near(self) -> Transform | None:
        return self.fix


def _premap(cloud: PointCloud2) -> np.ndarray:
    """Per point, whether only the premap put that voxel there (all False without a flags field)."""
    flags = cloud.flags_u32()
    if flags is None:
        return np.zeros(len(cloud), dtype=bool)
    premap: np.ndarray = (flags & (1 << FLAG_NAMES.index("premap"))) != 0
    return premap
