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

The base loads ``map_file`` into ``self.premap`` (the last ``map_stream`` cloud of a
``.db``/``.mcap`` recording), publishes accepted ``world -> map`` fixes on ``tf`` and the
premap on ``loaded_map`` once a fix lands. An implementation reads ``self.premap`` after
``super().start()`` (``None`` when no map is configured), picks its inputs, decides when a
fix is good enough, and hands it to :meth:`PremapModule.submit`. See ``lidar/module.py``.

Named locations live here too, in the ``map`` frame, so every new fix moves them with the old
map. They are not tf frames: the list on ``named_locations`` is where they are read from.

``named_locations`` is in and out: the whole list, as an ``InteractiveMarkerUpdate`` whose
``poses`` are ``InteractiveMarkerPose`` {header.frame_id = ``map``, pose, name}. Anyone may
publish it and everyone who does also listens, so adding, moving, renaming and deleting are
one operation -- a new whole list. Any update that is not a no-op is applied and republished as
the next revision, and listeners replace their cache with what comes back, so the browser page,
this module and the Controller's backend converge by hearing each other. ``server_id``/``seq_num``
name the run and the revision so an out-of-order echo can be ignored; they do not gate an edit.
Last writer wins, so a publisher that has not yet heard a deletion can bring a location back for
one propagation delay, then hears the list it caused and agrees. Going to a location is the
viewer's job: it sends the pose on the planner's ``goal`` in ``map_frame``, which the planner
re-expresses through tf every replan so the goal follows a later fix.

Locations are seeded from the last ``locations_stream`` snapshot of the ``map_file`` recording --
the premap is the only place they live. A recording of ``named_locations`` keeps them for the next
run, so **an edit made with no recording running is for this session only**. With no map and
``new_map`` set, this run's world frame is the map: the module submits an identity fix itself.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
from typing import Any
import uuid

import reactivex as rx
from reactivex import Observable, Subject, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, Out
from dimos.mapping.relocalization.named_locations import (
    load_locations,
    same_locations,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    named_pose,
    pose_of,
)
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RECORDING_SUFFIXES = (".db", ".mcap")


def find_stream(streams: list[str], wanted: str) -> str | None:
    """The stream named *wanted*, ignoring a leading ``/`` (or the ``_`` an .mcap topic becomes)."""
    return next((name for name in streams if name.lstrip("/_") == wanted.lstrip("/_")), None)


def load_premap(map_file: str, stream: str) -> PointCloud2:
    """The last *stream* cloud of a ``.db``/``.mcap`` recording."""
    if Path(map_file).suffix not in RECORDING_SUFFIXES:
        raise ValueError(
            f"map_file {map_file!r} must be a {' or '.join(RECORDING_SUFFIXES)} recording"
        )
    from dimos.memory.cli.dataset import open_store

    path = resolve_named_path(map_file, Path(map_file).suffix)
    with open_store(path, decode_metadata=True) as store:
        name = find_stream(store.list_streams(), stream)
        if name is None:
            raise ValueError(f"{path} has no {stream!r} stream, have {store.list_streams()}")
        cloud: PointCloud2 = store.stream(name).last().data
    return cloud


def yaw_deg(tf: Transform) -> float:
    return math.degrees(tf.rotation.euler.z)


def fix_error(fix: Transform, recorded: Transform) -> tuple[float, float]:
    """Yaw in degrees and translation in meters between two fixes of the same frame pair."""
    dyaw = (yaw_deg(fix) - yaw_deg(recorded) + 180.0) % 360.0 - 180.0
    return dyaw, (fix.translation - recorded.translation).length()


def fix_stream(fixes: Observable[Transform], interval: float) -> Observable[Transform]:
    """Every accepted fix as it lands, then again every ``interval`` s (once only if <= 0)."""

    def now_and_again(fix: Transform) -> Observable[Transform]:
        again = rx.interval(interval).pipe(ops.map(lambda _: fix)) if interval > 0 else rx.empty()
        return rx.concat(rx.of(fix), again)

    return fixes.pipe(ops.switch_map(now_and_again))


class Config(ModuleConfig):
    # Premap: a `.db`/`.mcap` recording whose last `map_stream` cloud is the map; None never fixes
    map_file: str | None = None
    # Recording stream holding the map cloud
    map_stream: str = "map_cloud"
    # Live fixed frame
    world_frame: str = "world"
    # Frame for the loaded premap
    map_frame: str = "map"
    # Seconds between tf republishes of the accepted fix (tf is not latched)
    tf_interval: float = 10.0
    # Seconds between `loaded_map` republishes; 0 publishes once per fix
    republish_loaded_map: float = 0.0
    # Stop attempting once a fix is accepted.
    relocalize_once: bool = True
    # No map_file: this run's world frame is the map (submit an identity fix)
    new_map: bool = False
    # Stream (recorded topic) the named-location snapshots live on
    locations_stream: str = "named_locations"


class PremapModule(Module):
    config: Config
    tf: IO[TFMessage]
    loaded_map: Out[PointCloud2]
    named_locations: IO[InteractiveMarkerUpdate]

    _placed: bool = False

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.fixes: Subject[Transform] = Subject()
        self.premap: PointCloud2 | None = None
        self.fix: Transform | None = None
        self._lock = threading.Lock()
        self._locations: dict[str, PoseStamped] = {}
        self._server_id = uuid.uuid4().hex
        self._revision = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            fix_stream(self.fixes, self.config.tf_interval).subscribe(lambda _: self.publish_tf())
        )
        self.open_locations()
        # A client that connected before this module came up would never learn the list exists
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

    def _identity(self) -> Transform:
        return Transform(frame_id=self.config.world_frame, child_frame_id=self.config.map_frame)

    def _load_premap(self, map_file: str) -> None:
        premap = load_premap(map_file, self.config.map_stream)

        premap.frame_id = self.config.map_frame
        self.premap = premap

        self.register_disposable(
            fix_stream(self.fixes, self.config.republish_loaded_map).subscribe(
                lambda _: self.loaded_map.publish(premap)
            )
        )

    def publish_tf(self) -> None:
        """The current ``world -> map`` fix."""
        if self.fix is None:
            return
        self.tf.publish(TFMessage(self.fix.now()))

    @property
    def placed(self) -> bool:
        """Whether any fix has been accepted yet."""
        return self._placed

    def keep_relocalizing(self) -> bool:
        """Whether to keep attempting. Implementations gate their input on this."""
        return not (self._placed and self.config.relocalize_once)

    def submit(self, tf: Transform, source: str = "") -> None:
        """Publish a ``world_frame -> map`` fix the implementation already decided to believe."""
        world, map_frame = self.config.world_frame, self.config.map_frame
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
        self.fix = tf
        self.fixes.on_next(tf)
        if not self._placed and self.config.relocalize_once:
            logger.info(f"relocalize {source}: placed, no further attempts (relocalize_once)")
        self._placed = True

    def open_locations(self) -> None:
        """Load the locations recorded in the map file."""
        if self.config.map_file:
            source = str(
                resolve_named_path(self.config.map_file, Path(self.config.map_file).suffix)
            )
            self._locations = load_locations(source, self.config.locations_stream)
            # A premap's locations are in its coordinates whatever frame its writer named
            for pose in self._locations.values():
                pose.frame_id = self.config.map_frame
            logger.info(
                f"Loaded {len(self._locations)} named locations from {source}: {list(self._locations)}"
            )

    async def handle_named_locations(self, published: InteractiveMarkerUpdate) -> None:
        locations: dict[str, PoseStamped] = {}
        for named in published.poses:
            pose = self.reexpress(pose_of(named), self.config.map_frame)
            if pose is None:
                # The publisher is waiting on an echo: give it the truth, not silence
                logger.warning(
                    f"named locations ignored: no transform from {named.header.frame_id!r} to {self.config.map_frame!r}"
                )
                self.named_locations.publish(self.locations_snapshot())
                return
            locations[named.name] = pose
        with self._lock:
            if same_locations(locations, self._locations):
                return
            self._locations = locations
            self._revision += 1
        logger.info(f"named locations: {list(locations)}")
        self.named_locations.publish(self.locations_snapshot())

    def frame_to(self, frame: str, child: str) -> Transform | None:
        """``frame -> child``: from this module's own fix for world/map, else from tf."""
        world, map_frame = self.config.world_frame, self.config.map_frame
        if self.fix is not None and (frame, child) == (world, map_frame):
            return self.fix
        if self.fix is not None and (frame, child) == (map_frame, world):
            return self.fix.inverse()
        return self.tfbuffer.get(frame, child)

    def reexpress(self, pose: PoseStamped, frame: str) -> PoseStamped | None:
        """*pose* re-expressed in *frame*; None without a transform yet."""
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
