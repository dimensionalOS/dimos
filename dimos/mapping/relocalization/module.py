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

import time
from typing import Any

import numpy as np
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.relocalize import relocalize as _relocalize
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

PUBLISH_INTERVAL = 2.0  # for loaded_map + TF
RELOC_INTERVAL = 2.0
MIN_LOCAL_POINTS = 50_000
MAP_SUFFIX = ".pc2.lcm"


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    publish_loaded_map: bool = False
    fitness_threshold: float = 0.45
    use_carving: bool = True


class RelocalizationModule(Module):
    config: Config
    global_map: In[PointCloud2]
    loaded_map: Out[PointCloud2]
    merged_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._premap: PointCloud2 | None = None
        self._current_map: str | None = None
        self._latest_local: PointCloud2 | None = None
        self._armed = False
        self._last_skip_log = 0.0
        self._world_to_map: Subject[Transform | None] = Subject()

    @rpc
    def start(self) -> None:
        super().start()
        # Arm the pipeline unconditionally so a map can be (re)loaded at runtime
        # (e.g. by RoomManager) even when none is configured at startup.
        self._arm()
        if self.config.map_file:
            self._load_premap(self.config.map_file)
        else:
            logger.info(
                "Relocalization module armed without a map — waiting for load_map()"
            )

    def _arm(self) -> None:
        """Wire the relocalize / merge / publish subscriptions exactly once."""
        if self._armed:
            return
        self._armed = True

        self.register_disposable(
            self.global_map.observable().subscribe(  # type: ignore[no-untyped-call]
                lambda m: setattr(self, "_latest_local", m)
            )
        )

        self.register_disposable(
            backpressure(
                self.global_map.observable().pipe(  # type: ignore[no-untyped-call]
                    ops.throttle_first(RELOC_INTERVAL),
                    ops.do_action(self._maybe_log_skip),
                    ops.filter(self._has_enough_points),
                )
            )
            .pipe(ops.map(self._try_relocalize))
            .subscribe(self._publish_tf)
        )

        self.register_disposable(
            backpressure(
                combine_latest(
                    self.global_map.observable(),  # type: ignore[no-untyped-call]
                    self._world_to_map.pipe(ops.start_with(None)),
                )
            ).subscribe(self._on_merge_input)
        )

        self.register_disposable(
            rx.interval(PUBLISH_INTERVAL)
            .pipe(ops.with_latest_from(self._world_to_map.pipe(ops.start_with(None))))
            .subscribe(self._publish_periodic)
        )

    def _load_premap(self, name: str) -> bool:
        """Decode a named premap and make it the active map. Drops any prior
        relocalization anchor so the next successful scan-match re-anchors."""
        try:
            path = resolve_named_path(name, MAP_SUFFIX)
            premap = PointCloud2.lcm_decode(path.read_bytes())
        except Exception:
            logger.exception("failed to load premap %r", name)
            return False
        premap.frame_id = FRAME_MAP
        self._premap = premap
        self._current_map = name
        # Drop the old world->map anchor; consumers treat None as "not localized".
        self._world_to_map.on_next(None)
        logger.info("relocalization map loaded: %r", name)
        return True

    @rpc
    def load_map(self, name: str) -> bool:
        """Load a named premap (.pc2.lcm) and commit the robot to that map.

        The next lidar scan that registers above ``fitness_threshold`` publishes
        a fresh ``world->map`` TF, re-anchoring the robot into the loaded map.
        """
        return self._load_premap(name)

    @rpc
    def score_map(self, name: str) -> float:
        """Attempt to register the latest live scan against a named premap and
        return the ICP fitness (0..1) WITHOUT committing the anchor. Used to pick
        which of several candidate room maps the robot is currently in."""
        local = self._latest_local
        if local is None:
            logger.warning("score_map(%r): no live scan yet", name)
            return 0.0
        try:
            path = resolve_named_path(name, MAP_SUFFIX)
            premap = PointCloud2.lcm_decode(path.read_bytes())
            _, fitness = _relocalize(premap.pointcloud, local.pointcloud)
        except Exception:
            logger.exception("score_map(%r) failed", name)
            return 0.0
        logger.info("score_map(%r): fitness=%.3f (n_pts=%d)", name, fitness, len(local))
        return float(fitness)

    @rpc
    def current_map(self) -> str | None:
        """Name of the currently loaded premap, or None if none is loaded."""
        return self._current_map

    def _maybe_log_skip(self, msg: PointCloud2) -> None:
        if self._has_enough_points(msg):
            return
        now = time.monotonic()
        if now - self._last_skip_log > 5.0:
            logger.warning(
                f"relocalize skipped: n_pts={len(msg)} < MIN_LOCAL_POINTS={MIN_LOCAL_POINTS}"
            )
            self._last_skip_log = now

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= MIN_LOCAL_POINTS

    def _publish_tf(self, tf: Transform | None) -> None:
        if tf is None:
            return
        self._world_to_map.on_next(tf)

    def _try_relocalize(self, msg: PointCloud2) -> Transform | None:
        if self._premap is None:
            return None  # armed but no map loaded yet
        t0 = time.monotonic()
        try:
            T, fitness = _relocalize(self._premap.pointcloud, msg.pointcloud)
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        n_pts = len(msg)

        if fitness < self.config.fitness_threshold:
            logger.warning(
                f"relocalize rejected: fitness={fitness:.3f} < threshold={self.config.fitness_threshold} "
                f"time_cost={dt:.1f}s n_pts={n_pts}"
            )
            return None

        # relocalize(scan, map) returns T such that scan_in_map_frame = T(scan_raw).
        # We are publishing a TF for map_in_scan_frame, notice that the base frame is `world`
        # so inverse the transform T here to get map_in_scan_frame
        T_inv = np.linalg.inv(T)
        new_tf = Transform(
            translation=Vector3(*T_inv[:3, 3]),
            rotation=Quaternion.from_rotation_matrix(T_inv[:3, :3]),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )
        logger.info(
            f"relocalize: fitness={fitness:.3f} time_cost={dt:.1f}s n_pts={n_pts} "
            f"reloc_t={T[:3, 3].round(3).tolist()} "
            f"TF {FRAME_WORLD!r} -> {FRAME_MAP!r} "
            f"published_t={T_inv[:3, 3].round(3).tolist()} "
        )
        return new_tf

    def _publish_periodic(self, pair: tuple[int, Transform | None]) -> None:
        _, tf = pair
        if self._premap is None or tf is None:
            return
        if self.config.publish_loaded_map:
            self.loaded_map.publish(self._premap)
        self.tf.publish(tf.now())

    def _on_merge_input(self, pair: tuple[PointCloud2, Transform | None]) -> None:
        local, tf = pair
        if self._premap is None:
            return
        if tf is None:
            # self.merged_map.publish(local)
            # costmap fallbacks to local map, skip publishing
            return
        premap_in_world = self._premap.transform(tf)
        if self.config.use_carving:
            grid = VoxelGrid(carve_columns=True, frame_id=local.frame_id, show_startup_log=False)
            try:
                grid.add_frame(premap_in_world)
                grid.add_frame(local)
                self.merged_map.publish(grid.get_global_pointcloud2())
            finally:
                grid.dispose()
        else:
            self.merged_map.publish(local + premap_in_world)
