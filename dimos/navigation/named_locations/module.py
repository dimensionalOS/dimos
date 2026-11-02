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

"""Named map locations: kept in the prior map's ``map`` frame, so they survive across runs.

Wire contract (no new message types):

- ``named_locations`` (in and out): the whole list, as an ``InteractiveMarkerUpdate`` whose
  ``poses`` are ``InteractiveMarkerPose`` {header.frame_id = ``map``, pose, name}. Anyone may
  publish it; adding, moving, renaming and deleting are all a new whole list. The module
  adopts a list that differs from its own (its own echo does not) and publishes its list only
  on a ``named_locations_request``, which a viewer sends when it connects.
- ``go_to_named_location`` (in): a name. Its pose goes out on ``goal`` in ``world_frame``,
  through the live ``world -> map`` transform (the relocalization fix).

Every list the module adopts is appended to ``save_to`` (a ``.db``). ``load_from`` (a ``.db`` or
``.mcap``: this module's own file or a ``--record`` recording) seeds the locations from
its last snapshot. With nothing to load the map is new, so this run's world frame is the
map frame and the module publishes an identity ``world -> map`` itself; the premap saved
from this run is then in that frame too.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    named_pose,
    pose_of,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class NamedLocationsConfig(ModuleConfig):
    # A .db or .mcap whose last `stream` snapshot seeds the locations; None starts a new map
    load_from: str | None = None
    # A .db every snapshot is appended to; None keeps them in memory only
    save_to: str | None = "named_locations.db"
    # Stream (recorded topic) the snapshots live on
    stream: str = "named_locations"
    # Live fixed frame, the one nav plans in
    world_frame: str = "world"
    # Prior map's frame, the one locations are kept in
    map_frame: str = "map"
    # Seconds between republishes of the identity world -> map of a new map (tf is not latched)
    tf_interval: float = 1.0


class NamedLocations(Module):
    """Stores named poses in the map frame, serves them to viewers, and sends the robot to one by name."""

    config: NamedLocationsConfig

    tf: IO[TFMessage]
    named_locations: IO[InteractiveMarkerUpdate]
    named_locations_request: In[String]
    go_to_named_location: In[String]
    goal: Out[PointStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        # name -> pose in map_frame, in the order they were added
        self._locations: dict[str, PoseStamped] = {}
        self._seq = 0
        self._store: SqliteStore | None = None

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        self.open()
        super().start()

    def open(self) -> None:
        """Load the saved locations, open the save file, and claim the map frame of a new map."""
        if self.config.load_from:
            self._locations = load_locations(self.config.load_from, self.config.stream)
            logger.info(
                f"Loaded {len(self._locations)} named locations from {self.config.load_from}: "
                f"{list(self._locations)}"
            )
        else:
            self.register_disposable(
                rx.interval(self.config.tf_interval).subscribe(
                    lambda _: self._publish_identity_map()
                )
            )
            self._publish_identity_map()
        if self.config.save_to:
            self._store = self.register_disposable(SqliteStore(path=self.config.save_to))
            self._store.start()

    def _publish_identity_map(self) -> None:
        self.tfbuffer.publish(
            Transform(
                frame_id=self.config.world_frame,
                child_frame_id=self.config.map_frame,
                ts=time.time(),
            )
        )

    async def handle_named_locations(self, published: InteractiveMarkerUpdate) -> None:
        locations: dict[str, PoseStamped] = {}
        for named in published.poses:
            pose = self.to_map(pose_of(named))
            if pose is None:
                logger.warning(
                    f"named locations ignored: no transform from {named.header.frame_id!r} to {self.config.map_frame!r}"
                )
                return
            locations[named.name] = pose
        with self._lock:
            if same_locations(locations, self._locations):
                return
            self._locations = locations
        logger.info(f"named locations: {list(locations)}")
        self._save(self.snapshot())

    async def handle_named_locations_request(self, _: String) -> None:
        self._publish_snapshot()

    async def handle_go_to_named_location(self, name: String) -> None:
        with self._lock:
            pose = self._locations.get(name.data)
        if pose is None:
            logger.warning(
                f"go to {name.data!r}: no such named location, have {list(self._locations)}"
            )
            return
        goal = self.to_world(pose)
        if goal is None:
            logger.warning(
                f"go to {name.data!r}: no {self.config.world_frame!r} -> {self.config.map_frame!r} yet"
            )
            return
        logger.info(f"go to {name.data!r}: goal {goal}")
        self.goal.publish(PointStamped(goal.x, goal.y, goal.z, frame_id=goal.frame_id))

    def to_map(self, pose: PoseStamped) -> PoseStamped | None:
        """*pose* re-expressed in the map frame, via tf."""
        return self._reexpress(pose, self.config.map_frame)

    def to_world(self, pose: PoseStamped) -> PoseStamped | None:
        """*pose* re-expressed in the world frame, via tf."""
        return self._reexpress(pose, self.config.world_frame)

    def _reexpress(self, pose: PoseStamped, frame: str) -> PoseStamped | None:
        if pose.frame_id == frame:
            return pose
        frame_to_pose = self.tfbuffer.get(frame, pose.frame_id)
        if frame_to_pose is None:
            return None
        placed = frame_to_pose + Transform.from_pose("named", pose)
        return placed.to_pose(ts=pose.ts)

    def snapshot(self) -> InteractiveMarkerUpdate:
        with self._lock:
            self._seq += 1
            return InteractiveMarkerUpdate(
                [named_pose(name, pose) for name, pose in self._locations.items()],
                server_id=type(self).__name__,
                seq_num=self._seq,
            )

    def _publish_snapshot(self) -> None:
        self.named_locations.publish(self.snapshot())

    def _save(self, snapshot: InteractiveMarkerUpdate) -> None:
        if self._store is not None:
            self._store.stream(self.config.stream, InteractiveMarkerUpdate).append(
                snapshot, ts=time.time()
            )


def same_locations(a: dict[str, PoseStamped], b: dict[str, PoseStamped]) -> bool:
    """The same names at the same poses (a list comes back over the wire bit for bit)."""
    return a.keys() == b.keys() and all(
        a[name].position == b[name].position and a[name].orientation == b[name].orientation
        for name in a
    )


def load_locations(path: str, stream: str) -> dict[str, PoseStamped]:
    """The locations in the last snapshot on *stream* of a .db or .mcap."""
    from dimos.memory.cli.dataset import open_store

    if not Path(path).exists():
        logger.warning(f"named locations file {path} does not exist yet, starting with none")
        return {}
    with open_store(path) as store:
        if stream not in store.list_streams():
            logger.warning(f"{path} has no {stream!r} stream, starting with no named locations")
            return {}
        last: InteractiveMarkerUpdate = store.stream(stream).last().data
    return {named.name: pose_of(named) for named in last.poses}
