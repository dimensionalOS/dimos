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

"""Module owning the durable saved-location store.

Bridges :class:`~dimos.memory2.locations.LocationStore` — which is deliberately
pure, taking a tf lookup and a status callable — to the running system: the
robot's current pose, the camera keyframe, the relocalization status channel,
and the tf tree.

It also owns auto-promotion. Locations tagged before relocalization converges are
saved run-local; when a confident fix arrives, every one of them is re-anchored
into ``map``. That trigger lives here rather than in a skill because it is a
property of the *system* reaching a state, not of the user asking for anything.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import uuid

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.mapping.relocalization.spec import RelocalizationSpec
from dimos.memory2.locations import (
    FRAME_WORLD,
    LocationError,
    LocationStore,
    RelocStatus,
    SavedLocation,
)
from dimos.memory2.module import MemoryModule, MemoryModuleConfig
from dimos.models.embedding.base import EmbeddingModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class LocationMemoryConfig(MemoryModuleConfig):
    db_path: str | Path = "locations.db"
    capture_keyframes: bool = True
    embedding_model: type[EmbeddingModel] | None = None
    promote_min_fitness: float = 0.6
    stale_after: float = 30.0


class LocationMemory(MemoryModule):
    """Implements :class:`~dimos.memory2.location_spec.LocationMemorySpec`.

    Exposes no skills of its own. The agent-facing surface lives in
    ``NavigationSkillContainer``, which already owns ``tag_location`` and is where
    navigation has to be integrated anyway — two modules both offering a
    ``tag_location`` tool would collide in the MCP tool list.
    """

    config: LocationMemoryConfig

    odom: In[PoseStamped]
    color_image: In[Image]

    _reloc: RelocalizationSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._run_id = f"run_{uuid.uuid4().hex[:8]}"
        self._latest_odom: PoseStamped | None = None
        self._latest_image: Image | None = None
        self._locations: LocationStore | None = None
        self._model: EmbeddingModel | None = None
        self._promoted_for_map: str | None = None

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.embedding_model is not None:
            self._model = self.register_disposable(self.config.embedding_model())
            self._model.start()

        self._locations = LocationStore(
            self.store,
            run_id=self._run_id,
            tf=self.tf,
            status=self._status,
            stale_after=self.config.stale_after,
            promote_min_fitness=self.config.promote_min_fitness,
        )
        logger.info("Location memory started", run_id=self._run_id, db=str(self.config.db_path))

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._latest_odom = msg

    async def handle_color_image(self, msg: Image) -> None:
        self._latest_image = msg

    def _status(self) -> RelocStatus:
        """Current relocalization health, or a permanently-unrelocalized stub.

        A missing relocalization module is not an error — it means every location
        is run-local — so this reports "no fix" rather than raising.
        """
        if self._reloc is None:
            return RelocStatus()
        try:
            return self._reloc.reloc_status()
        except Exception:
            logger.warning("Could not read relocalization status", exc_info=True)
            return RelocStatus()

    def _sync_map(self) -> RelocStatus:
        """Track the active map, and promote run-local tags on the first good fix."""
        status = self._status()
        store = self._store_or_raise()

        if status.map_id and store.map_id != status.map_id:
            store.set_map_lineage((status.map_id,))

        if status.relocalized and status.map_id and self._promoted_for_map != status.map_id:
            promoted = store.promote_pending()
            if promoted:
                logger.info(
                    "Anchored run-local locations to map",
                    count=len(promoted),
                    map_id=status.map_id,
                )
            if promoted or not store.list_all(include_deleted=False):
                self._promoted_for_map = status.map_id
        return status

    @rpc
    def save_location(self, name: str) -> SavedLocation:
        """Tag the robot's current pose with *name*."""
        store = self._store_or_raise()
        self._sync_map()

        if self._latest_odom is None:
            raise LookupError("No odometry received yet — cannot tag a location")

        keyframe = self._latest_image if self.config.capture_keyframes else None
        embedding = None
        if keyframe is not None and self._model is not None:
            try:
                embedding = self._model.embed(keyframe)
            except Exception:
                logger.warning("Could not embed keyframe", exc_info=True)

        return store.save(
            name,
            self._latest_odom,
            frame_id=FRAME_WORLD,
            keyframe=keyframe,
            embedding=embedding,
        )

    @rpc
    def resolve_location(self, name: str, target_frame: str = FRAME_WORLD) -> PoseStamped:
        """Pose of *name* in *target_frame*, or a :class:`LocationError`.

        ``target_frame`` defaults to ``world`` because that is the costmap's frame
        and the planner consumes goal coordinates raw, ignoring ``frame_id``.
        """
        store = self._store_or_raise()
        self._sync_map()
        return store.resolve(name, target_frame)

    @rpc
    def find_locations(self, query: str, limit: int = 5) -> list[SavedLocation]:
        """Search by meaning when an embedding model is configured, else by name."""
        store = self._store_or_raise()
        self._sync_map()

        exact = store.get(query)
        if exact is not None:
            return [exact]

        if self._model is not None:
            try:
                return store.search(self._model.embed_text(query), k=limit)
            except Exception:
                logger.warning("Embedding search failed, falling back to name match", exc_info=True)

        needle = query.strip().casefold()
        matches = [loc for loc in store.list_all() if needle in loc.name]
        return matches[:limit]

    @rpc
    def list_locations(self) -> list[SavedLocation]:
        self._sync_map()
        return self._store_or_raise().list_all()

    @rpc
    def delete_location(self, name: str) -> bool:
        self._sync_map()
        return self._store_or_raise().delete(name)

    def _store_or_raise(self) -> LocationStore:
        if self._locations is None:
            raise RuntimeError(f"{type(self).__name__}.start() has not run")
        return self._locations


__all__ = ["LocationError", "LocationMemory", "LocationMemoryConfig"]
