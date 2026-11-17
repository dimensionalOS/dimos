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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
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


class CloudRelocalization(PremapModule):
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
                msg.pointcloud, self.config.live_frame, self.map_frame, self.near()
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


# A voxel key's three axes, packed into one int64 at this many bits each.
_KEY_BITS = 21
_KEY_OFFSET = 1 << (_KEY_BITS - 1)


def _pack(points: np.ndarray, voxel: float) -> np.ndarray:
    keys = np.floor(points / voxel).astype(np.int64) + _KEY_OFFSET
    packed: np.ndarray = (keys[:, 0] << (2 * _KEY_BITS)) | (keys[:, 1] << _KEY_BITS) | keys[:, 2]
    return packed


def _in_cylinder(points: np.ndarray, bounds: PoseStamped) -> np.ndarray:
    """Which points the ray tracer's bounds cylinder holds (orientation x, y, z = radius, z_min, z_max)."""
    radius, z_min, z_max = bounds.orientation.x, bounds.orientation.y, bounds.orientation.z
    dx = points[:, 0] - bounds.position.x
    dy = points[:, 1] - bounds.position.y
    inside: np.ndarray = (
        (dx * dx + dy * dy <= radius * radius) & (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    )
    return inside


class LiveMap:
    """The live part of the ray tracer's local maps, kept one point per voxel.

    Each local map replaces what its bounds cylinder held, so what live rays carved goes too.
    """

    def __init__(self, voxel: float) -> None:
        self.voxel = voxel
        self._keys = np.empty(0, dtype=np.int64)
        self._points = np.empty((0, 3), dtype=np.float32)
        self._lock = threading.Lock()

    def __len__(self) -> int:
        return len(self._keys)

    def fold(self, local_map: PointCloud2, bounds: PoseStamped | None) -> None:
        points = local_map.points_f32().reshape(-1, 3)
        flags = local_map.flags_u32()
        if flags is not None:
            points = points[(flags & 1) == 0]
        keys = _pack(points, self.voxel)
        with self._lock:
            kept = np.ones(len(self._keys), dtype=bool)
            if bounds is not None:
                kept = ~_in_cylinder(self._points, bounds)
            all_keys = np.concatenate([keys, self._keys[kept]])
            all_points = np.concatenate([points, self._points[kept]])
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
    # Fold the ray tracer's local_map into the kept live map at most this often (s)
    accumulate_interval: float = 1.0
    # The kept live map's voxel edge (m)
    live_voxel_m: float = 0.1
    # Match only kept live points within this xy distance (m) of the robot's latest local map; 0 matches them all
    match_radius_m: float = 30.0
    # Resend loaded_map this often (s) after the first fix; the ray tracer keeps the first one per frame
    republish_loaded_map: float = 30.0


class ContinuousRelocalization(CloudRelocalization):
    """Keeps live_frame -> map current against a ray tracer that keeps the premap apart (``separate_premaps``).

    It keeps its own map of the live (non-premap) voxels of the ray tracer's local maps, so the
    premap never confirms itself. The first fix searches the whole premap; every later one refines
    the current fix by ICP. The ray tracer moves the premap by each fix it publishes.
    live_frame must be the frame the ray tracer maps in (its world_frame).
    """

    config: ContinuousConfig

    local_map: In[PointCloud2]
    region_bounds: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.live = LiveMap(self.config.live_voxel_m)
        # region_bounds by stamp, which local_map pairs on
        self._bounds: deque[PoseStamped] = deque(maxlen=32)
        self._center: tuple[float, float] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self.premap is None:
            return
        self.register_disposable(self.region_bounds.observable().subscribe(self._bounds.append))  # type: ignore[no-untyped-call]
        self.register_disposable(
            self.local_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.throttle_first(self.config.accumulate_interval))
            .subscribe(self.on_local_map)
        )

    def on_local_map(self, cloud: PointCloud2) -> None:
        bounds = next((b for b in reversed(self._bounds) if b.ts == cloud.ts), None)
        self.live.fold(cloud, bounds)
        if bounds is not None:
            self._center = (bounds.position.x, bounds.position.y)

    def clouds(self) -> Observable[PointCloud2]:
        return rx.interval(self.config.reloc_interval).pipe(
            ops.map(
                lambda _: self.live.cloud(
                    self.config.live_frame, self._center, self.config.match_radius_m
                )
            )
        )

    def near(self) -> Transform | None:
        return self.fix

    def loaded_map_fixes(self) -> Observable[Transform]:
        # Every fix moves the premap; the ray tracer needs it only once
        return self.fixes.pipe(ops.take(1))
