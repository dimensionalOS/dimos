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

"""Relocalization: place the live ``world`` frame inside a prior map's ``map`` frame.

With a ``live_cloud`` it also matches that cloud against the premap (FPFH+RANSAC then ICP) and
publishes the fix; other strategies read ``self.premap`` after ``super().start()`` and call
:meth:`PremapModule.submit`.
Named locations live in the ``map`` frame; ``named_locations`` carries the whole list both ways
(last writer wins). They persist only in the map file, so edits made without a recording are lost.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import Any, Literal, cast
import uuid

import numpy as np
import reactivex as rx
from reactivex import Observable, Subject, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.mapping.relocalization.cloud_relocalizer import (
    MID360,
    CloudRelocalizer,
    RelocalizeConfig,
)
from dimos.mapping.relocalization.utils import (
    DEFAULT_MAP_FRAME,
    LiveMap,
    blend,
    load_locations,
    load_premap,
    moved,
    resolve_map,
    window,
    yaw_deg,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    named_pose,
    pose_of,
)
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


def _stamped(tf: Transform, ts: float) -> Transform:
    return Transform(
        translation=tf.translation,
        rotation=tf.rotation,
        frame_id=tf.frame_id,
        child_frame_id=tf.child_frame_id,
        ts=ts,
    )


def _scan_stamp(scan: PointCloud2) -> float | None:
    return scan.ts


class Config(ModuleConfig):
    # Premap `.db`/`.mcap` recording; None never fixes
    map_file: str | None = None
    # Cloud to load, overriding the recording's own `map/<name>/cloud`
    cloud_stream: str | None = None
    # Live fixed frame
    world_frame: str = "world"
    # Frame for the loaded premap; None: the frame its stream names
    map_frame: str | None = None
    # Stop attempting once a fix is accepted; otherwise every later attempt refines the current
    # fix by ICP alone, and with local_map matches only its non-premap voxels (needs the premap bit)
    relocalize_once: bool = True
    # No map_file: this run's world frame is the map
    new_map: bool = False
    # Live cloud to match against the premap: the ray tracer's local_map, a window of lidar scans, or none
    live_cloud: Literal["none", "local_map", "lidar"] = "none"
    # local_map: skip local maps sparser than this, too small a patch to match
    min_local_points: int = 20_000
    # Seconds live clouds may stay under min_local_points (normal while the map builds) before that warns, once
    sparse_cloud_grace_s: float = 10.0
    # Rig preset to align with, see cloud_relocalizer.PRESETS
    relocalize: RelocalizeConfig = MID360
    # relocalize_once off, local_map: the kept live map's voxel edge (m)
    live_voxel_m: float = 0.1
    # ... local_map: match kept live points within this xy distance (m) of the latest local map; 0 matches them all
    match_radius_m: float = 30.0
    # ... lidar: a scan's tf must be stamped within this of the scan (s)
    scan_tf_tolerance_s: float = 0.1
    # Refused ICP refinements in a row before searching the whole premap again; 0 never does
    global_after_refusals: int = 3
    # Least time (s) from one match's start to the next, 0 matches back to back
    match_interval_s: float = 1.0
    # relocalize_once off: time constant (s) for easing refinements into the fix, 0 applies them at once
    fix_smoothing_s: float = 2.0


class PremapModule(Module):
    """Loads the premap, keeps its named locations, and places it in the live world frame.

    With ``relocalize_once`` off and ``live_cloud="local_map"`` it matches its own map of
    local_map's live (non-premap) voxels, so the premap never confirms itself, and with
    ``live_cloud="lidar"`` the preset's window of scans. The first fix
    searches the whole premap and places it in the ray tracer; every later one refines the
    current fix by ICP and only moves ``world -> map``. world_frame must be the frame the
    ray tracer maps in.
    """

    config: Config
    tf: IO[TFMessage]
    loaded_map: Out[PointCloud2]
    # Every match attempt's fitness (0-1, the share of live points the premap explains), accepted or not
    relocalization_strength: Out[Float32]
    named_locations: IO[InteractiveMarkerUpdate]
    named_locations_request: In[String]
    local_map: In[PointCloud2]
    # TODO: clean up relocalization once the ray tracer outputs a CloudUpdate message: remove the scan input and only use that
    lidar: In[PointCloud2]

    _placed: bool = False

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.fixes: Subject[Transform] = Subject()
        self.premap: PointCloud2 | None = None
        # The last accepted world_frame -> map fix; split, where the first one placed the premap for good
        self.fix: Transform | None = None
        # Split: the latest map -> premap fix, which world -> map follows
        self._map_fix: Transform | None = None
        self._lock = threading.Lock()
        self._locations: dict[str, PoseStamped] = {}
        self._server_id = uuid.uuid4().hex
        self._revision = 0
        self._cloud_stream: str | None = None
        self._map_frame: str | None = None
        self._locations_stream: str | None = None
        self._relocalizer: CloudRelocalizer | None = None
        # When live clouds went under min_local_points, None while they have enough
        self._sparse_since: float | None = None
        self._sparse_warned = False
        self.live = LiveMap(self.config.live_voxel_m)
        # Fires whenever the kept live map grows, the continuous match's cue
        self._live_changed: Subject[None] = Subject()
        self._center: tuple[float, float] | None = None
        self._refusals = 0
        # Stamp of the cloud the last accepted fix came from
        self._fix_ts: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.map_file:
            self._resolve_map(self.config.map_file)
        self._start_tf()
        self.open_locations()
        # Clients connected before start() would otherwise never hear the list
        self.named_locations.publish(self.locations_snapshot())
        if not self.config.map_file:
            if self.config.new_map:
                logger.info("Relocalization: new map, this run's world frame is the map frame")
                self.submit(self._identity(), "new map")
            else:
                logger.info("Relocalization module disabled (no map_file configured)")
            return
        self._load_premap(self.config.map_file)
        logger.info(f"Relocalization module started: map_file={self.config.map_file!r}")
        if self.config.live_cloud != "none":
            self._start_matching()

    def _start_matching(self) -> None:
        assert self.premap is not None
        # Premap preprocessing is the dominant cost, paid once here.
        self._relocalizer = CloudRelocalizer(self.premap.pointcloud, self.config.relocalize)
        interval = self.config.match_interval_s
        throttle = [ops.throttle_first(interval)] if interval > 0 else []
        self.register_disposable(
            self._clouds()
            .pipe(
                ops.filter(lambda _: not (self._placed and self.config.relocalize_once)),
                ops.do_action(self._log_point_count_change),
                ops.filter(self._has_enough_points),
                *throttle,
            )
            .subscribe(self._relocalize)
        )

    def _clouds(self) -> Observable[PointCloud2]:
        """The live cloud to match, on one worker: each attempt takes the latest cloud the moment the last one ends, those between are dropped."""
        scans = self.lidar.observable()  # type: ignore[no-untyped-call]
        local_maps = self.local_map.observable()  # type: ignore[no-untyped-call]
        if self.config.relocalize_once:
            if self.config.live_cloud == "lidar":
                return window(scans, self.config.relocalize)
            return backpressure(local_maps)
        if self.config.live_cloud == "lidar":
            # Each scan is placed by the tf at its stamp as it lands
            placed = scans.pipe(
                ops.map(self._place_scan), ops.filter(lambda scan: scan is not None)
            )
            return window(cast("Observable[PointCloud2]", placed), self.config.relocalize)
        # A local map is the whole window around the robot, so the latest replaces any waiting during a fold
        self.register_disposable(backpressure(local_maps).subscribe(self._on_local_map))
        return backpressure(self._live_changed).pipe(
            ops.map(
                lambda _: self.live.cloud(
                    self._live_frame, self._center, self.config.match_radius_m
                )
            )
        )

    def _place_scan(self, scan: PointCloud2) -> PointCloud2 | None:
        """*scan* in the live frame; None when it has no tf at its stamp."""
        to_live = self.tfbuffer.get(
            self._live_frame, scan.frame_id, scan.ts, self.config.scan_tf_tolerance_s
        )
        if to_live is None:
            logger.warning(
                f"scan skipped: no transform from {scan.frame_id!r} to {self._live_frame!r} at its stamp"
            )
            return None
        points = moved(scan.points_f32().reshape(-1, 3), to_live)
        return PointCloud2.from_numpy(points, frame_id=self._live_frame, timestamp=scan.ts)

    def _on_local_map(self, cloud: PointCloud2) -> None:
        to_live = self.frame_to(self._live_frame, cloud.frame_id)
        if to_live is None:
            logger.warning(
                f"local_map skipped: no transform from {cloud.frame_id!r} to {self._live_frame!r}"
            )
            return
        self.live.fold(cloud, to_live)
        points = cloud.points_f32().reshape(-1, 3)
        if len(points):
            # The local map is a window around the robot, so its xy middle is close enough.
            low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
            middle = to_live.to_matrix() @ np.array(
                [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, 0.0, 1.0]
            )
            self._center = (float(middle[0]), float(middle[1]))
        self._live_changed.on_next(None)

    def _log_point_count_change(self, msg: PointCloud2) -> None:
        """Log when live clouds drop under min_local_points and when they reach it again; warn once if they stay under past the grace."""
        fields = {"n_pts": len(msg), "min_local_points": self.config.min_local_points}
        if self._has_enough_points(msg):
            if self._sparse_since is not None:
                logger.info("relocalize resumed: enough local points", **fields)
            self._sparse_since = None
            return
        now = time.monotonic()
        if self._sparse_since is None:
            self._sparse_since, self._sparse_warned = now, False
            logger.info("relocalize skipped until there are enough local points", **fields)
        elif (
            not self._sparse_warned and now - self._sparse_since >= self.config.sparse_cloud_grace_s
        ):
            self._sparse_warned = True
            logger.warning(
                f"relocalize still skipped after {self.config.sparse_cloud_grace_s:.0f}s: not enough local points",
                **fields,
            )

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return self.config.live_cloud != "local_map" or len(msg) >= self.config.min_local_points

    def _relocalize(self, msg: PointCloud2) -> None:
        assert self._relocalizer is not None
        t0 = time.monotonic()
        # The cloud is in the frame its mapper works in: world, or the map frame the fixes move
        world_to_cloud = self.frame_to(self.config.world_frame, msg.frame_id)
        if world_to_cloud is None:
            logger.warning(
                f"relocalize skipped: no transform to the cloud's frame {msg.frame_id!r}"
            )
            return
        n_global = self.config.global_after_refusals
        lost = n_global > 0 and self._refusals >= n_global
        initial_fix = None
        if not self.config.relocalize_once and self.fix is not None and not lost:
            initial_fix = world_to_cloud.inverse() + self.fix
        try:
            attempt = self._relocalizer.attempt(
                msg.pointcloud, msg.frame_id, self.map_frame, initial_fix
            )
        except Exception:
            logger.exception("relocalize() failed")
            return
        self.relocalization_strength.publish(Float32(float(attempt.result.fitness)))
        tf = None if attempt.fix is None else world_to_cloud + attempt.fix
        dt = time.monotonic() - t0
        if tf is None:
            self._refusals += 1
            logger.info(
                f"relocalize lidar: refused after {dt:.1f}s n_pts={len(msg)} "
                f"(below fitness_threshold={self.config.relocalize.fitness_threshold})"
            )
            if n_global > 0 and self._refusals == n_global and self.fix is not None:
                logger.warning(
                    f"relocalize: {n_global} refusals in a row, searching the whole premap again"
                )
            return
        self._refusals = 0
        logger.info(f"relocalize lidar: time_cost={dt:.1f}s n_pts={len(msg)}")
        if initial_fix is not None:
            tf = self._smoothed(tf, msg.ts)
        self._fix_ts = msg.ts
        self.submit(tf, "lidar")

    def _smoothed(self, tf: Transform, ts: float) -> Transform:
        """The current fix moved toward *tf*, further the longer since the last fix."""
        tau = self.config.fix_smoothing_s
        if tau <= 0 or self.fix is None or self._fix_ts is None:
            return tf
        return blend(self.fix, tf, 1.0 - math.exp(-max(ts - self._fix_ts, 0.0) / tau))

    def _start_tf(self) -> None:
        # Each scan carries the edges stamped with its own stamp, so the ray tracer's lookup at that stamp finds them exactly; a fix goes out at once
        self.publish_tf()
        self.register_disposable(
            rx.merge(
                self.lidar.observable().pipe(ops.map(_scan_stamp)),
                self.fixes.pipe(ops.map(lambda _: None)),
            ).subscribe(self.publish_tf)
        )

    @property
    def _split(self) -> bool:
        """Odometry is in map, a frame apart from world and the premap: world -> map and world -> premap."""
        return DEFAULT_MAP_FRAME not in (self.config.world_frame, self.map_frame)

    @property
    def _live_frame(self) -> str:
        """Split: live voxels are kept in map, the odometry frame no fix moves, so they never go stale."""
        return DEFAULT_MAP_FRAME if self._split else self.config.world_frame

    def _world_to_map(self) -> Transform:
        """world -> map: identity until the first fix, then whatever puts the latest map -> premap fix on the placed premap."""
        if self.fix is None or self._map_fix is None:
            return Transform(frame_id=self.config.world_frame, child_frame_id=DEFAULT_MAP_FRAME)
        return self.fix + self._map_fix.inverse()

    @property
    def map_frame(self) -> str:
        return self.config.map_frame or self._map_frame or DEFAULT_MAP_FRAME

    def _identity(self) -> Transform:
        return Transform(frame_id=self.config.world_frame, child_frame_id=self.map_frame)

    def _resolve_map(self, map_file: str) -> None:
        from dimos.memory.cli.dataset import open_store

        path = resolve_named_path(map_file, Path(map_file).suffix)
        with open_store(path, decode_metadata=True) as store:
            streams = store.list_streams()
        self._cloud_stream, self._map_frame, self._locations_stream = resolve_map(
            streams, self.config.cloud_stream
        )
        logger.info(
            "map chosen",
            stream=self._cloud_stream,
            map_frame=self.map_frame,
            locations_stream=self._locations_stream,
        )

    def _load_premap(self, map_file: str) -> None:
        assert self._cloud_stream is not None, "_resolve_map runs before this"
        premap = load_premap(map_file, self._cloud_stream)

        premap.frame_id = self.map_frame
        self.premap = premap

        # Only the first fix places the premap, once: split, later ones move world -> map instead.
        self.register_disposable(
            self.fixes.pipe(ops.take(1)).subscribe(lambda _: self.loaded_map.publish(premap))
        )

    def publish_tf(self, ts: float | None = None) -> None:
        """Split: world -> map always, world -> premap once placed. Otherwise world -> premap once fixed. Stamped *ts*, or now."""
        stamp = time.time() if ts is None else ts
        if self._split:
            edges = [self._world_to_map()] + ([self.fix] if self.fix is not None else [])
            self.tf.publish(TFMessage(*(_stamped(edge, stamp) for edge in edges)))
        elif self.fix is not None:
            self.tf.publish(TFMessage(_stamped(self.fix, stamp)))

    @property
    def placed(self) -> bool:
        """Whether any fix has been accepted yet."""
        return self._placed

    def submit(self, tf: Transform, source: str = "") -> None:
        """Publish a ``world_frame -> map`` fix the implementation already decided to believe."""
        world, map_frame = self.config.world_frame, self.map_frame
        assert (tf.frame_id, tf.child_frame_id) == (world, map_frame), (
            f"relocalize {source}: expected {world!r} -> {map_frame!r}, "
            f"got {tf.frame_id!r} -> {tf.child_frame_id!r}"
        )
        logger.info(
            "relocalize fix",
            source=source,
            world=world,
            map_frame=map_frame,
            translation=str(tf.translation),
            yaw_deg=round(yaw_deg(tf), 1),
        )
        if self._split:
            # tf is world -> premap under the current world -> map; keep its map -> premap part
            self._map_fix = self._world_to_map().inverse() + tf
            if self.fix is None:
                self.fix = tf
        else:
            self.fix = tf
        self.fixes.on_next(tf)
        if not self._placed and self.config.relocalize_once:
            logger.info(f"relocalize {source}: placed, no further attempts (relocalize_once)")
        self._placed = True

    def open_locations(self) -> None:
        if self.config.map_file:
            source = str(
                resolve_named_path(self.config.map_file, Path(self.config.map_file).suffix)
            )
            assert self._locations_stream is not None, "_resolve_map runs before this"
            self._locations = load_locations(source, self._locations_stream)
            # Locations are in premap coordinates whatever frame their writer named
            for pose in self._locations.values():
                pose.frame_id = self.map_frame
            logger.info(
                f"Loaded {len(self._locations)} named locations from {source}: {list(self._locations)}"
            )

    async def handle_named_locations(self, published: InteractiveMarkerUpdate) -> None:
        locations: dict[str, PoseStamped] = {}
        for named in published.poses:
            pose = self.reexpress(pose_of(named), self.map_frame)
            if pose is None:
                # The publisher waits on an echo
                logger.warning(
                    f"named locations ignored: no transform from {named.header.frame_id!r} to {self.map_frame!r}"
                )
                self.named_locations.publish(self.locations_snapshot())
                return
            locations[named.name] = pose
        with self._lock:
            # An echo of our own list is a no-op
            if locations == self._locations:
                return
            self._locations = locations
            self._revision += 1
        logger.info(f"named locations: {list(locations)}")
        self.named_locations.publish(self.locations_snapshot())

    async def handle_named_locations_request(self, _: String) -> None:
        """Resend the list; zenoh keeps no history for late subscribers."""
        self.named_locations.publish(self.locations_snapshot())

    def frame_to(self, frame: str, child: str) -> Transform | None:
        """``frame -> child``, preferring our own fix over tf for world/map."""
        world, map_frame = self.config.world_frame, self.map_frame
        if frame == child:
            return Transform(frame_id=frame, child_frame_id=child)
        if self.fix is not None and (frame, child) == (world, map_frame):
            return self.fix
        if self.fix is not None and (frame, child) == (map_frame, world):
            return self.fix.inverse()
        if self._split and (frame, child) == (world, DEFAULT_MAP_FRAME):
            return self._world_to_map()
        if self._split and (frame, child) == (DEFAULT_MAP_FRAME, world):
            return self._world_to_map().inverse()
        return self.tfbuffer.get(frame, child)

    def reexpress(self, pose: PoseStamped, frame: str) -> PoseStamped | None:
        """*pose* in *frame*; None without a transform yet."""
        if pose.frame_id == frame:
            return pose
        frame_to_pose = self.frame_to(frame, pose.frame_id)
        if frame_to_pose is None:
            return None
        placed = frame_to_pose + Transform.from_pose("placed", pose)
        return placed.to_pose(ts=pose.ts)

    def locations_snapshot(self) -> InteractiveMarkerUpdate:
        with self._lock:
            return InteractiveMarkerUpdate(
                [named_pose(name, pose) for name, pose in self._locations.items()],
                server_id=self._server_id,
                seq_num=self._revision,
            )

    @rpc
    def location(self, name: str) -> PoseStamped:
        """The named location's pose in the map frame. Raises LookupError for an unknown name."""
        with self._lock:
            pose = self._locations.get(name)
            if pose is None:
                known = ", ".join(sorted(self._locations)) or "none"
                raise LookupError(f"no location named {name!r}, the map has: {known}")
        return pose
