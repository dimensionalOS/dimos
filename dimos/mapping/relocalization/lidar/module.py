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
from dimos.mapping.relocalization.module import Config, RelocalizationModule
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


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


class CloudRelocalization(RelocalizationModule):
    """Coarse FPFH+RANSAC then ICP of a windowed live cloud against a pointcloud premap.

    Subclasses supply :meth:`clouds`.
    """

    config: LidarConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._relocalizer: LidarRelocalizer | None = None
        self._last_skip_log = 0.0

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
            return
        now = time.monotonic()
        if now - self._last_skip_log > 5.0:
            logger.warning(
                f"relocalize skipped: n_pts={len(msg)} "
                f"< min_local_points={self.config.min_local_points}"
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
                msg.pointcloud, self.config.world_frame, self.config.map_frame, self.near()
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
        live = {tuple(p) for p in _flagged(cloud, None)}
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
        added = {tuple(p) for p in _flagged(cloud, change != 0)}
        removed = {tuple(p) for p in cloud.points_f32()[change == 0]}
        with self._lock:
            self._live |= added
            self._live -= removed

    def live_cloud(self) -> PointCloud2:
        with self._lock:
            points = np.array(list(self._live), dtype=np.float32).reshape(-1, 3)
        return PointCloud2.from_numpy(points, frame_id=self.config.world_frame)

    def clouds(self) -> Observable[PointCloud2]:
        return rx.interval(self.config.reloc_interval).pipe(ops.map(lambda _: self.live_cloud()))

    def near(self) -> Transform | None:
        return self.fix


def _flagged(cloud: PointCloud2, keep: np.ndarray | None) -> np.ndarray:
    """The points of ``cloud`` that are live (premap flag 0), among ``keep`` if given."""
    points = cloud.points_f32()
    premap = cloud.premap_u8()
    mask = np.ones(len(points), dtype=bool) if keep is None else keep
    if premap is not None:
        mask = mask & (premap == 0)
    live: np.ndarray = points[mask]
    return live
