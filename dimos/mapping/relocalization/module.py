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

Implementations read ``self.premap`` after ``super().start()`` and call :meth:`PremapModule.submit`.
Named locations live in the ``map`` frame; ``named_locations`` carries the whole list both ways
(last writer wins). They persist only in the map file, so edits made without a recording are lost.
"""

from __future__ import annotations

import math
from pathlib import Path
import re
import threading
from typing import Any
import uuid

import reactivex as rx
from reactivex import Observable, Subject, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.mapping.relocalization.named_locations import (
    load_locations,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
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

LOCALIZED_PERIOD_S = 1.0


def stream_key(name: str) -> str:
    """*name* with ``/`` folded to ``_``, since a ``.db`` stream name is a SQL identifier."""
    return name.lstrip("/_").replace("/", "_")


def find_stream(streams: list[str], wanted: str) -> str | None:
    return next((name for name in streams if stream_key(name) == stream_key(wanted)), None)


DEFAULT_MAP_FRAME = "map"

MAP_CLOUD_RE = re.compile(rf"^{re.escape(DEFAULT_MAP_FRAME)}_(?P<name>.+)_cloud$")

# A cloud_stream that is not a named map has no locations of its own
OVERRIDE_LOCATIONS = "named_locations"


def named_map_frame(name: str) -> str:
    return f"{DEFAULT_MAP_FRAME}/{name}"


def map_cloud_stream(name: str, slashed: bool = True) -> str:
    stream = f"{named_map_frame(name)}/cloud"
    return stream if slashed else stream_key(stream)


def map_names(streams: list[str]) -> dict[str, str]:
    """Map name -> its cloud stream."""
    found: dict[str, str] = {}
    for name in streams:
        match = MAP_CLOUD_RE.match(stream_key(name))
        if match:
            found[match["name"]] = name
    return found


def resolve_map(streams: list[str], wanted: str | None) -> tuple[str, str, str]:
    """(cloud stream, map frame, locations stream); *wanted* overrides the lone ``map/<name>/cloud``."""
    named = map_names(streams)
    if wanted is not None:
        name = next((n for n, s in named.items() if stream_key(s) == stream_key(wanted)), None)
        if name is None:
            if find_stream(streams, wanted) is None:
                raise ValueError(
                    f"the recording has no {wanted!r} stream, it holds {sorted(streams)}"
                )
            return wanted, wanted.lstrip("/"), OVERRIDE_LOCATIONS
    elif len(named) == 1:
        name = next(iter(named))
    elif not named:
        raise ValueError(
            f"no map in the recording: nothing in {sorted(streams)} is a {map_cloud_stream('<name>')}"
        )
    else:
        raise ValueError(
            f"{len(named)} maps in the recording ({', '.join(sorted(named))}): "
            "pass cloud_stream to say which"
        )
    frame = named_map_frame(name)
    return named[name], frame, f"{frame}/named_locations"


def load_premap(map_file: str, stream: str | None = None) -> PointCloud2:
    """The last cloud of the recording's map, or of *stream*."""
    if Path(map_file).suffix not in RECORDING_SUFFIXES:
        raise ValueError(
            f"map_file {map_file!r} must be a {' or '.join(RECORDING_SUFFIXES)} recording"
        )
    from dimos.memory.cli.dataset import open_store

    path = resolve_named_path(map_file, Path(map_file).suffix)
    with open_store(path, decode_metadata=True) as store:
        streams = store.list_streams()
        if stream is None:
            stream, _, _ = resolve_map(streams, None)
        name = find_stream(streams, stream)
        if name is None:
            raise ValueError(f"{path} has no {stream!r} stream, have {streams}")
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
    # Premap `.db`/`.mcap` recording; None never fixes
    map_file: str | None = None
    # Cloud to load, overriding the recording's own `map/<name>/cloud`
    cloud_stream: str | None = None
    # Live fixed frame
    world_frame: str = "world"
    # Frame for the loaded premap; None: the frame its stream names
    map_frame: str | None = None
    # Seconds between tf republishes of the accepted fix (tf is not latched)
    tf_interval: float = 10.0
    # Seconds between `loaded_map` republishes; 0 publishes once per fix
    republish_loaded_map: float = 0.0
    # Stop attempting once a fix is accepted.
    relocalize_once: bool = True
    # No map_file: this run's world frame is the map
    new_map: bool = False


class PremapModule(Module):
    config: Config
    tf: IO[TFMessage]
    loaded_map: Out[PointCloud2]
    named_locations: IO[InteractiveMarkerUpdate]
    named_locations_request: In[String]
    localized: Out[Bool]

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
        self._cloud_stream: str | None = None
        self._map_frame: str | None = None
        self._locations_stream: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            rx.interval(LOCALIZED_PERIOD_S).subscribe(lambda _: self.publish_localized())
        )
        self.register_disposable(
            fix_stream(self.fixes, self.config.tf_interval).subscribe(lambda _: self.publish_tf())
        )
        if self.config.map_file:
            self._resolve_map(self.config.map_file)
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

        self.register_disposable(
            fix_stream(self.fixes, self.config.republish_loaded_map).subscribe(
                lambda _: self.loaded_map.publish(premap)
            )
        )

    def publish_tf(self) -> None:
        if self.fix is None:
            return
        self.tf.publish(TFMessage(self.fix.now()))

    def publish_localized(self) -> None:
        self.localized.publish(Bool(self.placed))

    @property
    def placed(self) -> bool:
        """Whether any fix has been accepted yet."""
        return self._placed

    def keep_relocalizing(self) -> bool:
        """Whether to keep attempting. Implementations gate their input on this."""
        return not (self._placed and self.config.relocalize_once)

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
        if self.fix is not None and (frame, child) == (world, map_frame):
            return self.fix
        if self.fix is not None and (frame, child) == (map_frame, world):
            return self.fix.inverse()
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
