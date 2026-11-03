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

The base loads ``map_file`` into ``self.premap`` (a ``.pc2.lcm``, or the last
``map_stream`` cloud of a ``.db``/``.mcap`` recording), publishes accepted
``world -> map`` fixes on ``tf`` and the premap on ``loaded_map`` once a fix lands.
An implementation reads ``self.premap`` after ``super().start()`` (``None`` when
no map is configured), picks its inputs, decides when a fix is good enough, and
hands it to :meth:`RelocalizationModule.submit`. See ``lidar/module.py``.

Named locations live here too, kept in the ``map`` frame so every new fix moves them
with the old map. Each is also a tf frame under ``map``. Wire contract:

- ``named_locations`` (in and out): the whole list, as an ``InteractiveMarkerUpdate`` whose
  ``poses`` are ``InteractiveMarkerPose`` {header.frame_id = ``map``, pose, name}, and whose
  ``server_id``/``seq_num`` name the module run and the revision of the list. Anyone may
  publish it; adding, moving, renaming and deleting are all a new whole list, carrying the
  ``server_id``/``seq_num`` of the list it was edited from. The module adopts it only when that
  is its current revision, then publishes the result as the next revision; a list edited from
  an older revision, or from another run, is dropped and the current list republished, so a
  stale cache can never bring back a renamed or deleted location. A viewer replaces its cache
  with every list the module publishes, and sends ``named_locations_request`` when it connects.
- Going to a location is the viewer's job: it sends the location's pose on the planner's
  ``goal`` as a ``PointStamped`` in ``map_frame``, which the planner re-expresses through tf on
  every replan, so the goal follows a later fix.

The locations are seeded from the last ``locations_stream`` snapshot of ``locations_from``
(by default the ``map_file`` recording itself), and every list adopted is appended to
``save_locations_to``. With no map and ``new_map`` set, this run's world frame is the map:
the module submits an identity ``world -> map`` fix itself.

AprilTags come in two sets that never mix. The old map's tags are clustered from the
``tags_streams`` glimpses of the ``map_file`` recording, kept in ``map`` and published as
``old_april_tags`` plus tf frames ``tag_old_<id>`` under ``map``, so each new fix carries
them along. Tags seen this run (``marker_detections``) are clustered live in ``world`` and
published as ``new_april_tags`` plus frames ``tag_<id>`` under ``world``.
"""

from __future__ import annotations

from collections import defaultdict, deque
import math
from pathlib import Path
import threading
import time
from typing import Any
import uuid

import reactivex as rx
from reactivex import Observable, Subject, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.mapping.relocalization.apriltags import (
    TagClustering,
    TagDetection,
    TagGates,
    as_pose,
    cluster_tags,
    marker_poses,
    pose_list,
    recording_tags,
    tag_frame,
    tags_message,
)
from dimos.mapping.relocalization.named_locations import (
    load_locations,
    location_frame,
    same_locations,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    named_pose,
    pose_of,
)
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MAP_SUFFIX = ".pc2.lcm"
RECORDING_SUFFIXES = (".db", ".mcap")


def is_recording(path: str) -> bool:
    return Path(path).suffix in RECORDING_SUFFIXES


def find_stream(streams: list[str], wanted: str) -> str | None:
    """The stream named *wanted*, ignoring a leading ``/`` (or the ``_`` an .mcap topic becomes)."""
    return next((name for name in streams if name.lstrip("/_") == wanted.lstrip("/_")), None)


def load_premap(map_file: str, stream: str) -> PointCloud2:
    """A ``.pc2.lcm`` premap, or the last *stream* cloud of a ``.db``/``.mcap`` recording."""
    if not is_recording(map_file):
        return PointCloud2.lcm_decode(resolve_named_path(map_file, MAP_SUFFIX).read_bytes())
    from dimos.memory.cli.dataset import open_store

    path = resolve_named_path(map_file, Path(map_file).suffix)
    with open_store(path) as store:
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
    # Premap: a `.pc2.lcm` stem or path (suffix appended if absent), or a `.db`/`.mcap`
    # recording whose last `map_stream` cloud is the map; None never attempts a fix
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
    # A .db/.mcap whose last `locations_stream` snapshot seeds the named locations;
    # None takes them from map_file when it is a recording
    locations_from: str | None = None
    # A .db every adopted list is appended to; None keeps them in memory only
    save_locations_to: str | None = None
    # Stream (recorded topic) the named-location snapshots live on
    locations_stream: str = "named_locations"
    # Recording streams (first present wins) of AprilTag glimpses that seed the old map's
    # tags: camera-frame PoseStamped with a `marker_id` tag, or a marker detector's
    # Detection3DArray; empty loads none
    tags_streams: tuple[str, ...] = ("april_tags_raw", "marker_detections")
    tag_gates: TagGates = TagGates()
    tag_clustering: TagClustering = TagClustering()
    old_tag_prefix: str = "tag_old_"
    new_tag_prefix: str = "tag_"
    # Live glimpses kept per tag for its pose (the most recent)
    max_tag_glimpses: int = 300


class RelocalizationModule(Module):
    config: Config
    tf: IO[TFMessage]
    loaded_map: Out[PointCloud2]
    named_locations: IO[InteractiveMarkerUpdate]
    named_locations_request: In[String]
    marker_detections: In[Detection3DArray]
    old_april_tags: Out[Detection3DArray]
    new_april_tags: Out[Detection3DArray]

    _placed: bool = False

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.fixes: Subject[Transform] = Subject()
        self.premap: PointCloud2 | None = None
        self.fix: Transform | None = None
        self._lock = threading.Lock()
        # name -> pose in map_frame, in the order they were added
        self._locations: dict[str, PoseStamped] = {}
        # The list's revision, under an id of its own per run so another run's revisions never match
        self._server_id = uuid.uuid4().hex
        self._revision = 0
        self._locations_store: SqliteStore | None = None
        # marker id -> clustered pose; old in map_frame, new in world_frame
        self.old_tags: dict[int, TagDetection] = {}
        self.new_tags: dict[int, TagDetection] = {}
        self._tag_glimpses: dict[int, deque[TagDetection]] = defaultdict(
            lambda: deque(maxlen=self.config.max_tag_glimpses)
        )

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            fix_stream(self.fixes, self.config.tf_interval).subscribe(lambda _: self.publish_tf())
        )
        self.open_locations()
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
        recorded_frame = premap.frame_id

        premap.frame_id = self.config.map_frame
        self.premap = premap
        if is_recording(map_file) and self.config.tags_streams:
            self.old_tags = recording_tags(
                str(resolve_named_path(map_file, Path(map_file).suffix)),
                self.config.tags_streams,
                recorded_frame,
                self.config.tag_gates,
                self.config.tag_clustering,
            )

        self.register_disposable(
            fix_stream(self.fixes, self.config.republish_loaded_map).subscribe(
                lambda _: self._publish_map_layers(premap)
            )
        )

    def _publish_map_layers(self, premap: PointCloud2) -> None:
        self.loaded_map.publish(premap)
        if self.old_tags:
            self.old_april_tags.publish(
                tags_message(self.old_tags, self.config.map_frame, time.time())
            )

    def map_anchored(self) -> list[Transform]:
        """Frames that hang off the old map, so every fix moves them: locations and old tags."""
        with self._lock:
            locations = list(self._locations.items())
        map_frame = self.config.map_frame
        return [
            *(Transform.from_pose(location_frame(name), pose).now() for name, pose in locations),
            *(
                Transform.from_pose(
                    tag_frame(marker_id, self.config.old_tag_prefix), as_pose(tag, map_frame)
                ).now()
                for marker_id, tag in self.old_tags.items()
            ),
        ]

    def world_anchored(self) -> list[Transform]:
        """Frames placed in this run's world: the tags seen live."""
        world = self.config.world_frame
        return [
            Transform.from_pose(
                tag_frame(marker_id, self.config.new_tag_prefix), as_pose(tag, world)
            ).now()
            for marker_id, tag in self.new_tags.items()
        ]

    def publish_tf(self) -> None:
        """The current fix with the old map's frames, and the live tags' frames."""
        transforms = [self.fix.now(), *self.map_anchored()] if self.fix is not None else []
        transforms += self.world_anchored()
        if transforms:
            self.tf.publish(TFMessage(*transforms))

    async def handle_marker_detections(self, message: Detection3DArray) -> None:
        """Live glimpses re-clustered per tag in the world frame; never merged with old tags."""
        changed = False
        for marker_id, pose in marker_poses(message):
            placed = self.reexpress(pose, self.config.world_frame)
            if placed is None:
                logger.warning(f"tag {marker_id}: no transform from {pose.frame_id!r} to world")
                continue
            glimpses = self._tag_glimpses[marker_id]
            glimpses.append(TagDetection(message.ts, marker_id, pose_list(placed)))
            clustered = cluster_tags(list(glimpses), self.config.tag_clustering).get(marker_id)
            if clustered is not None:
                self.new_tags[marker_id] = clustered
                changed = True
        if changed:
            self.new_april_tags.publish(
                tags_message(self.new_tags, self.config.world_frame, message.ts)
            )
            self.publish_tf()

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

    # Named locations

    def open_locations(self) -> None:
        """Load the saved locations and open the save file."""
        source = self.config.locations_from
        if source is None and self.config.map_file and is_recording(self.config.map_file):
            source = str(
                resolve_named_path(self.config.map_file, Path(self.config.map_file).suffix)
            )
        if source:
            self._locations = load_locations(source, self.config.locations_stream)
            logger.info(
                f"Loaded {len(self._locations)} named locations from {source}: {list(self._locations)}"
            )
        if self.config.save_locations_to:
            self._locations_store = self.register_disposable(
                SqliteStore(path=self.config.save_locations_to)
            )
            self._locations_store.start()

    async def handle_named_locations(self, published: InteractiveMarkerUpdate) -> None:
        locations: dict[str, PoseStamped] = {}
        for named in published.poses:
            pose = self.reexpress(pose_of(named), self.config.map_frame)
            if pose is None:
                logger.warning(
                    f"named locations ignored: no transform from {named.header.frame_id!r} to {self.config.map_frame!r}"
                )
                return
            locations[named.name] = pose
        with self._lock:
            if same_locations(locations, self._locations):
                return
            edited_from = (published.server_id, published.seq_num)
            current = (self._server_id, self._revision)
            if edited_from == current:
                self._locations = locations
                self._revision += 1
        if edited_from != current:
            logger.warning(
                f"named locations dropped: edited from revision {edited_from}, current is {current}"
            )
            self.named_locations.publish(self.locations_snapshot())
            return
        logger.info(f"named locations: {list(locations)}")
        snapshot = self.locations_snapshot()
        self._save_locations(snapshot)
        self.named_locations.publish(snapshot)
        self.publish_tf()

    async def handle_named_locations_request(self, _: String) -> None:
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

    def _save_locations(self, snapshot: InteractiveMarkerUpdate) -> None:
        if self._locations_store is not None:
            self._locations_store.stream(
                self.config.locations_stream, InteractiveMarkerUpdate
            ).append(snapshot, ts=time.time())
