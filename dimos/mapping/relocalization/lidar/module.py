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
import time
from typing import Any

from reactivex import Observable, operators as ops

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.mapping.relocalization.lidar.relocalize import (
    MID360,
    LidarRelocalizer,
    RelocalizeConfig,
)
from dimos.mapping.relocalization.module import Config, RelocalizationModule
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


def window(
    scans: Observable[PointCloud2], cfg: RelocalizeConfig, interval: float
) -> Observable[PointCloud2]:
    """The last ``max_frames`` scans merged, at most one merge every ``interval`` s.

    Every scan enters the window and only the throttle decides when one is
    matched, so the scans that arrive during a failed attempt are the next
    attempt's evidence - which is how ``tune.py`` scores a retry too.
    """
    held: deque[PointCloud2] = deque(maxlen=cfg.max_frames)
    return scans.pipe(
        ops.do_action(held.append),
        ops.throttle_first(interval),
        ops.filter(lambda _: len(held) >= cfg.min_frames),
        ops.map(lambda _: reduce(operator.add, held)),
    )


class LidarConfig(Config):
    reloc_interval: float = 2.0
    # Sanity check, skip a cloud too sparse to be worth a match,
    # A mid360 sweep is only ~3k points and two of them voxel down to ~3.5k
    # which is already enough to relocalize
    min_local_points: int = 2_000
    # Which rig's measured scales to align with. mid360 because that is
    # what has been measured; a different sensor wants its own preset
    # (see relocalize.PRESETS and tune.md), not these numbers.
    relocalize: RelocalizeConfig = MID360


class CloudRelocalization(RelocalizationModule):
    """Coarse FPFH+RANSAC then ICP of a windowed live cloud against a pointcloud premap.

    A subclass names where that window comes from, and nothing else.
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
        # Downsampling the premap and computing its normals and FPFH is the
        # pipeline's dominant cost, so it is a startup cost, not a per-fix one.
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

    def _relocalize(self, msg: PointCloud2) -> None:
        assert self._relocalizer is not None
        t0 = time.monotonic()
        try:
            tf = self._relocalizer.relocalize(msg.pointcloud, self.config.world_frame)
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
    """Matches the last `max_frames` scans, the way tune.py's probes are built."""

    lidar: In[PointCloud2]

    def clouds(self) -> Observable[PointCloud2]:
        return window(
            self.lidar.observable(),
            self.config.relocalize,
            self.config.reloc_interval,
        )


class LocalMapRelocalization(CloudRelocalization):
    """Matches the ray-tracing mapper's local map, a window in space, already carved."""

    local_map: In[PointCloud2]

    def clouds(self) -> Observable[PointCloud2]:
        return self.local_map.observable().pipe(  # type: ignore[no-untyped-call]
            ops.throttle_first(self.config.reloc_interval)
        )
