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

"""Cold spatial memory for independently reset DimSim evaluation tasks."""

import re
from threading import RLock
from typing import Any, Protocol

from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.perception.spatial_memory_spec import SpatialMemorySpec
from dimos.perception.spatial_perception import SpatialMemory


class DimSimSpatialMemorySpec(SpatialMemorySpec, Protocol):
    def clear_eval_memory(self) -> int: ...

    def eval_memory_generation(self) -> int: ...

    def record_detection_viewpoint(
        self,
        descriptions: list[str],
        pose: PoseStamped,
        frame_ts: float,
    ) -> None: ...

    def query_detection_viewpoint(self, query: str) -> PoseStamped | None: ...


class DimSimSpatialMemory(SpatialMemory):
    """Spatial memory that can be cleared at a DimSim task boundary."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._eval_memory_lock = RLock()
        self._eval_memory_generation = 0
        self._detection_viewpoints: dict[str, tuple[float, PoseStamped]] = {}

    def _process_frame(self) -> None:
        # A periodic embedding already in flight must finish before the reset
        # deletes its vector, otherwise a pre-reset viewpoint can be inserted
        # after clear_eval_memory() returns.
        with self._eval_memory_lock:
            super()._process_frame()

    @rpc
    def clear_eval_memory(self) -> int:
        """Remove observations and tags left by an earlier simulator pose.

        DimSim resets the robot without restarting the process. Semantic-map
        entries are camera viewpoints, so retaining entries across a reset can
        send a new task toward an unrelated pose from the previous run.
        """
        with self._eval_memory_lock:
            image_collection = self.vector_db.image_collection
            image_ids = image_collection.get(include=[]).get("ids", [])
            if image_ids:
                image_collection.delete(ids=image_ids)

            location_collection = self.vector_db.location_collection
            location_ids = location_collection.get(include=[]).get("ids", [])
            if location_ids:
                location_collection.delete(ids=location_ids)

            if self._visual_memory is not None:
                self._visual_memory.clear()

            self.robot_locations.clear()
            self._latest_video_frame = None
            self.last_position = None
            self.last_record_time = None
            self.frame_count = 0
            self.stored_frame_count = 0
            self._detection_viewpoints.clear()
            self._eval_memory_generation += 1
            return len(image_ids) + len(location_ids)

    @rpc
    def eval_memory_generation(self) -> int:
        """Return the current independently-reset eval memory generation."""
        with self._eval_memory_lock:
            return self._eval_memory_generation

    @rpc
    def record_detection_viewpoint(
        self,
        descriptions: list[str],
        pose: PoseStamped,
        frame_ts: float,
    ) -> None:
        """Remember where a positive lookout frame was actually captured."""
        with self._eval_memory_lock:
            for description in descriptions:
                key = _normalize_detection_query(description)
                if key:
                    self._detection_viewpoints[key] = (frame_ts, pose)

    @rpc
    def query_detection_viewpoint(self, query: str) -> PoseStamped | None:
        """Return the newest exact lookout viewpoint matching ``query``."""
        normalized_query = _normalize_detection_query(query)
        if not normalized_query:
            return None

        with self._eval_memory_lock:
            matches = [
                value
                for description, value in self._detection_viewpoints.items()
                if description in normalized_query or normalized_query in description
            ]
            if not matches:
                return None
            return max(matches, key=lambda match: match[0])[1]


def _normalize_detection_query(query: str) -> str:
    return " ".join(re.findall(r"[a-z0-9]+", query.lower()))
