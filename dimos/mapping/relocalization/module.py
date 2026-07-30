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
from dimos.mapping.relocalization.relocalize import track as _track
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
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

# Tracking mode: reuse the last accepted transform as a seed for a cheap
# ICP instead of a full FPFH+RANSAC search (see relocalize.track()). These
# gate how long/how far we trust that seed before forcing a fresh full
# search via `relocalize()`.
MAX_TRACKING_AGE = 30.0  # seconds since last_good before forcing a full search
MAX_TRACKING_MISSES = 3  # consecutive tracking misses before forcing a full search
# Untested edge case: no eval run so far has ever hit this -- every tracking
# attempt across every run has been accepted, so miss_count has never once
# incremented in practice. Worth treating as unvalidated, not proven.
SANITY_CHECK_DISTANCE = 2.0  # meters of straight-line odom displacement since
# the last full search before forcing another one, even if tracking is
# otherwise succeeding -- new map area carved in since then may have shifted
# the best-fit alignment slightly. Tuned empirically across several eval
# runs at 3.0/2.0/1.5/1.0m: tightening below ~2.0 buys back little to no
# accuracy (both global-only and tracking accuracy drift similarly across a
# session -- see README) while giving back most of the compute savings, so
# 2.0 is the settled value, not the tightest one tried.
SANITY_CHECK_INTERVAL = 45.0  # seconds since the last full search before
# forcing another one regardless of distance -- catches a robot picked up
# and moved while stationary, which a distance-only trigger would miss.
# Unlike SANITY_CHECK_DISTANCE, this has never actually been the trigger in
# any eval run (the test recording keeps moving, so distance always fires
# first) -- kept close to MAX_TRACKING_AGE for internal consistency rather
# than validated against data.
YAW_FAN_DEG = (0.0, 5.0, -5.0, 10.0, -10.0)  # tried in this order; first hit wins
# ^ kept as the max-width fallback fan (see `yaw_fan_for_travel` below) --
# used only once distance/time-based sizing alone would already justify a
# wide search.
ASSUMED_YAW_DRIFT_DEG_PER_M = 0.3  # coarse assumed heading-uncertainty growth
# per meter of odom-measured straight-line travel since the last full search
# -- not a real per-embodiment IMU/covariance model, just enough to keep the
# common short-interval attempt cheap (fan of 1) while still widening the
# search as the seed gets stale. Keyed on distance rather than clock time:
# new map area gets carved in roughly proportional to distance traveled, not
# elapsed time, so a stationary robot adds no new drift risk regardless of
# the clock.
TRACKING_AGE_BACKSTOP = 15.0  # seconds -- if distance-based sizing alone would
# keep the fan at a single candidate (robot appears stationary in odom) but
# this much time has passed anyway, still try a small residual fan. Catches
# "picked up and moved while appearing stationary," which distance can't see.


# Yaw-fan sizing history (design decisions kept as comments, not dead code):
#   v1: fixed fan (YAW_FAN_DEG) tried on every attempt regardless of
#       staleness. Wasteful: heading drift over a couple of seconds is
#       normally well under a degree, so 0deg alone clears almost every
#       short-interval attempt.
#   v2: fan widened with elapsed time since last_good, centered at 0.
#   v3: fan centered on odom's own measured heading change instead of 0.
#       REGRESSED badly (yaw errors up to ~100deg in eval): a real robot
#       turn is already correctly represented in the accumulated map via
#       per-frame odometry placement -- the world_raw frame itself doesn't
#       rotate -- so centering the search on "how much did the robot turn"
#       actively rotates a good seed by however much the robot legitimately
#       turned. `odom.orientation` is body-in-world_raw; `last_good.T`'s
#       rotation is world_raw-in-map -- different frame pairing, not
#       interchangeable.
#   v4: reverted to center=0; width driven by distance/time since
#       `last_good`. Self-defeating: `last_good` updates on every tracking
#       success too, so this reset every ~2s hop and the fan almost never
#       actually widened across a streak even as true cumulative drift grew.
#   current: width driven by distance/time since the last FULL SEARCH
#       instead (only resets on an absolute reacquisition), so it widens
#       correctly across a whole tracking streak.
def yaw_fan_for_travel(distance_m: float, age_s: float) -> tuple[float, ...]:
    """Yaw offsets to try, widening with odom-measured straight-line distance
    traveled since the last full search (primary driver), plus a small
    time-based backstop (`TRACKING_AGE_BACKSTOP`) for the "picked up while
    appearing stationary" case distance alone can't see. Always centered at
    0 -- see the version history above. Public (no leading underscore) so
    `eval.py` can reuse the exact same logic the live module runs, instead
    of a parallel copy that could drift.
    """
    step = ASSUMED_YAW_DRIFT_DEG_PER_M * distance_m
    if age_s > TRACKING_AGE_BACKSTOP:
        step = max(step, 2.0)
    if step < 1.0:
        return (0.0,)
    if step < 5.0:
        return (0.0, step, -step)
    return YAW_FAN_DEG


def should_force_full_search(
    now_ts: float,
    last_good: tuple[np.ndarray, float, float] | None,
    miss_count: int,
    last_full_search_ts: float,
    last_full_search_pos: np.ndarray | None,
    latest_odom_xy: np.ndarray | None,
) -> bool:
    """Decide SEARCHING (full relocalize()) vs TRACKING (seeded track()).

    Free function, not a method, so `eval.py`'s tracking-mode benchmark can
    exercise the exact same gating logic the live module runs, instead of a
    parallel reimplementation that could silently diverge. `now_ts` should
    be the message's own timestamp, not wall-clock time, so this behaves
    the same live and in replay/eval.
    """
    if last_good is None:
        return True
    _, _, last_good_ts = last_good
    if now_ts - last_good_ts > MAX_TRACKING_AGE:
        return True
    if miss_count >= MAX_TRACKING_MISSES:
        return True
    if now_ts - last_full_search_ts > SANITY_CHECK_INTERVAL:
        return True
    if latest_odom_xy is not None and last_full_search_pos is not None:
        # Straight-line displacement, not accumulated path length -- a cheap
        # proxy for "how much new map area has likely been carved since the
        # last full search," not a real odometry integral.
        if float(np.linalg.norm(latest_odom_xy - last_full_search_pos)) > SANITY_CHECK_DISTANCE:
            return True
    return False


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    publish_loaded_map: bool = False
    fitness_threshold: float = 0.45
    tracking_fitness_threshold: float = 0.5
    use_carving: bool = True


class RelocalizationModule(Module):
    config: Config
    global_map: In[PointCloud2]
    odom: In[PoseStamped]
    loaded_map: Out[PointCloud2]
    merged_map: Out[PointCloud2]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._premap: PointCloud2 | None = None
        self._last_skip_log = 0.0
        self._world_to_map: Subject[Transform | None] = Subject()

        # Tracking-mode state (see module constants above + relocalize.track()).
        self._latest_odom: PoseStamped | None = None
        self._last_good: tuple[np.ndarray, float, float] | None = None  # (T, fitness, ts)
        self._miss_count: int = 0
        self._last_full_search_ts: float = 0.0
        self._last_full_search_pos: np.ndarray | None = None  # xy at last full search

    @rpc
    def start(self) -> None:
        super().start()

        if not self.config.map_file:
            logger.info("Relocalization module disabled (no map_file configured)")
            return

        path = resolve_named_path(self.config.map_file, MAP_SUFFIX)
        self._premap = PointCloud2.lcm_decode(path.read_bytes())
        self._premap.frame_id = FRAME_MAP

        self.register_disposable(self.odom.observable().subscribe(self._on_odom))  # type: ignore[no-untyped-call]

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
            .pipe(ops.with_latest_from(self._world_to_map))
            .subscribe(self._publish_periodic)
        )

        logger.info(
            f"Relocalization module started: map_file={self.config.map_file!r}  "
            f"loaded_map.frame_id={self._premap.frame_id!r}"
        )

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

    def _on_odom(self, msg: PoseStamped) -> None:
        self._latest_odom = msg

    def _should_force_full_search(self, now_ts: float) -> bool:
        """Decide SEARCHING (full relocalize()) vs TRACKING (seeded track()).

        Thin wrapper around the free function `should_force_full_search`
        above -- see its docstring for why this isn't inlined.
        """
        latest_odom_xy = (
            np.array([self._latest_odom.position.x, self._latest_odom.position.y])
            if self._latest_odom is not None
            else None
        )
        return should_force_full_search(
            now_ts,
            self._last_good,
            self._miss_count,
            self._last_full_search_ts,
            self._last_full_search_pos,
            latest_odom_xy,
        )

    def _try_relocalize(self, msg: PointCloud2) -> Transform | None:
        assert self._premap is not None
        t0 = time.monotonic()

        # Mode switch: cheap seeded tracking vs. the untouched global search.
        # See `yaw_fan_for_travel`'s docstring for the design history behind
        # how the tracking-mode seed/fan is computed.
        force_full = self._should_force_full_search(msg.ts)
        try:
            if not force_full and self._last_good is not None:
                seed_T, _, last_good_ts = self._last_good
                distance_since_full_search = 0.0
                if self._latest_odom is not None and self._last_full_search_pos is not None:
                    now_xy = np.array([self._latest_odom.position.x, self._latest_odom.position.y])
                    distance_since_full_search = float(
                        np.linalg.norm(now_xy - self._last_full_search_pos)
                    )
                fan = yaw_fan_for_travel(
                    distance_since_full_search, msg.ts - self._last_full_search_ts
                )
                T, fitness = _track(
                    self._premap.pointcloud, msg.pointcloud, seed_T, yaw_fan_deg=fan
                )
                mode = "tracking"
                threshold = self.config.tracking_fitness_threshold
            else:
                T, fitness = _relocalize(self._premap.pointcloud, msg.pointcloud)
                mode = "search"
                threshold = self.config.fitness_threshold
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        n_pts = len(msg)

        if fitness < threshold:
            if mode == "tracking":
                self._miss_count += 1
                logger.warning(
                    f"tracking miss ({self._miss_count}/{MAX_TRACKING_MISSES}): "
                    f"fitness={fitness:.3f} < threshold={threshold} time_cost={dt:.1f}s n_pts={n_pts}"
                )
            else:
                logger.warning(
                    f"relocalize rejected: fitness={fitness:.3f} < threshold={threshold} "
                    f"time_cost={dt:.1f}s n_pts={n_pts}"
                )
            return None

        # Carry state forward so the next attempt can use tracking mode, and
        # so periodic sanity checks know when/where the last full search was.
        self._miss_count = 0
        self._last_good = (T, fitness, msg.ts)
        if mode == "search":
            self._last_full_search_ts = msg.ts
            if self._latest_odom is not None:
                self._last_full_search_pos = np.array(
                    [self._latest_odom.position.x, self._latest_odom.position.y]
                )

        # relocalize()/track() return T such that scan_in_map_frame = T(scan_raw).
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
            f"relocalize[{mode}]: fitness={fitness:.3f} time_cost={dt:.1f}s n_pts={n_pts} "
            f"reloc_t={T[:3, 3].round(3).tolist()} "
            f"TF {FRAME_WORLD!r} -> {FRAME_MAP!r} "
            f"published_t={T_inv[:3, 3].round(3).tolist()} "
        )
        return new_tf

    def _publish_periodic(self, pair: tuple[int, Transform]) -> None:
        _, tf = pair
        if self._premap is None:
            return
        if self.config.publish_loaded_map:
            self.loaded_map.publish(self._premap)
        self.tf.publish(TFMessage(tf.now()))

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
