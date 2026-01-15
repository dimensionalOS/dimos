# Copyright 2025-2026 Dimensional Inc.
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

import threading
import time
from typing import TYPE_CHECKING, Any

import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Vector3
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


class ObjectDB:
    """Spatial memory database for 3D object detections.

    Maintains two tiers of objects internally:
    - _pending_objects: Recently detected objects (detection_count < threshold)
    - _objects: Confirmed permanent objects (detection_count >= threshold)

    Deduplication uses two heuristics:
    1. track_id match from YOLOE tracker (recent match)
    2. Center distance within threshold (spatial proximity match)
    """

    def __init__(
        self,
        distance_threshold: float = 0.1,
        min_detections_for_permanent: int = 6,
        pending_ttl_s: float = 5.0,
        track_id_ttl_s: float = 2.0,
    ) -> None:
        self._distance_threshold = distance_threshold
        self._min_detections = min_detections_for_permanent
        self._pending_ttl_s = pending_ttl_s
        self._track_id_ttl_s = track_id_ttl_s

        # Internal storage - keyed by object_id
        self._pending_objects: dict[str, Object] = {}
        self._objects: dict[str, Object] = {}  # Permanent objects

        # track_id -> object_id mapping for fast lookup
        self._track_id_map: dict[int, str] = {}
        self._last_add_stats: dict[str, int] = {}

        self._lock = threading.RLock()

    # ─────────────────────────────────────────────────────────────────
    # Public Methods
    # ─────────────────────────────────────────────────────────────────

    def add_objects(self, objects: list[Object]) -> list[Object]:
        """Add multiple objects to the database with deduplication.

        Args:
            objects: List of Object instances from object_scene_registration

        Returns:
            List of updated/created Object instances
        """
        stats = {
            "input": len(objects),
            "created": 0,
            "updated": 0,
            "promoted": 0,
            "matched_track": 0,
            "matched_distance": 0,
        }

        results: list[Object] = []
        now = time.time()
        with self._lock:
            self._prune_stale_pending(now)
            for obj in objects:
                matched, reason = self._match(obj, now)
                if matched is None:
                    results.append(self._insert_pending(obj, now))
                    stats["created"] += 1
                    continue

                self._update_existing(matched, obj, now)
                results.append(matched)
                stats["updated"] += 1
                if reason == "track":
                    stats["matched_track"] += 1
                elif reason == "distance":
                    stats["matched_distance"] += 1
                if self._check_promotion(matched):
                    stats["promoted"] += 1

        stats["pending"] = len(self._pending_objects)
        stats["permanent"] = len(self._objects)
        self._last_add_stats = stats
        return results

    def get_last_add_stats(self) -> dict[str, int]:
        with self._lock:
            return dict(self._last_add_stats)

    def get_objects(self) -> list[Object]:
        """Get all permanent objects (detection_count >= threshold)."""
        with self._lock:
            return list(self._objects.values())

    def get_all_objects(self) -> list[Object]:
        """Get all objects (both pending and permanent)."""
        with self._lock:
            return list(self._pending_objects.values()) + list(self._objects.values())

    def promote(self, object_id: str) -> bool:
        """Promote an object from pending to permanent."""
        with self._lock:
            if object_id in self._pending_objects:
                self._objects[object_id] = self._pending_objects.pop(object_id)
                return True
            return object_id in self._objects

    def find_by_name(self, name: str) -> list[Object]:
        """Find all permanent objects with matching name."""
        with self._lock:
            return [obj for obj in self._objects.values() if obj.name == name]

    def find_nearest(
        self,
        position: Vector3,
        name: str | None = None,
    ) -> Object | None:
        """Find nearest permanent object to a position, optionally filtered by name.

        Args:
            position: Position to search from
            name: Optional name filter

        Returns:
            Nearest Object or None if no objects found
        """
        with self._lock:
            candidates = [
                obj
                for obj in self._objects.values()
                if obj.center is not None and (name is None or obj.name == name)
            ]

            if not candidates:
                return None

            return min(candidates, key=lambda obj: position.distance(obj.center))  # type: ignore[arg-type]

    def clear(self) -> None:
        """Clear all objects from the database."""
        with self._lock:
            # Drop Open3D pointcloud references before clearing to reduce shutdown warnings.
            for obj in list(self._pending_objects.values()) + list(self._objects.values()):
                obj.pointcloud = PointCloud2(
                    pointcloud=o3d.geometry.PointCloud(),
                    frame_id=obj.pointcloud.frame_id,
                    ts=obj.pointcloud.ts,
                )
            self._pending_objects.clear()
            self._objects.clear()
            self._track_id_map.clear()
            logger.info("ObjectDB cleared")

    def get_stats(self) -> dict[str, int]:
        """Get statistics about the database."""
        with self._lock:
            return {
                "pending_count": len(self._pending_objects),
                "permanent_count": len(self._objects),
                "total_count": len(self._pending_objects) + len(self._objects),
            }

    # ─────────────────────────────────────────────────────────────────
    # Internal Methods
    # ─────────────────────────────────────────────────────────────────

    def _match(self, obj: Object, now: float) -> tuple[Object | None, str | None]:
        if obj.track_id >= 0:
            matched = self._match_by_track_id(obj.track_id, now)
            if matched is not None:
                return matched, "track"

        matched = self._match_by_distance(obj)
        if matched is not None:
            return matched, "distance"
        return None, None

    def _insert_pending(self, obj: Object, now: float) -> Object:
        if not obj.ts:
            obj.ts = now
        self._pending_objects[obj.object_id] = obj
        if obj.track_id >= 0:
            self._track_id_map[obj.track_id] = obj.object_id
        logger.info(f"Created new pending object {obj.object_id} ({obj.name})")
        return obj

    def _update_existing(self, existing: Object, obj: Object, now: float) -> None:
        existing.update_object(obj)
        existing.ts = obj.ts or now
        if obj.track_id >= 0:
            self._track_id_map[obj.track_id] = existing.object_id

    def _match_by_track_id(self, track_id: int, now: float) -> Object | None:
        """Find object with matching track_id from YOLOE."""
        if track_id < 0:
            return None

        object_id = self._track_id_map.get(track_id)
        if object_id is None:
            return None

        # Check in permanent objects first
        if object_id in self._objects:
            obj = self._objects[object_id]
        elif object_id in self._pending_objects:
            obj = self._pending_objects[object_id]
        else:
            del self._track_id_map[track_id]
            return None

        last_seen = obj.ts if obj.ts else now
        if now - last_seen > self._track_id_ttl_s:
            del self._track_id_map[track_id]
            return None

        return obj

    def _match_by_distance(self, obj: Object) -> Object | None:
        """Find object within distance threshold."""
        if obj.center is None:
            return None

        # Combine all objects and filter by valid center
        all_objects = list(self._objects.values()) + list(self._pending_objects.values())
        candidates = [
            o
            for o in all_objects
            if o.center is not None and obj.center.distance(o.center) < self._distance_threshold
        ]

        if not candidates:
            return None

        return min(candidates, key=lambda o: obj.center.distance(o.center))  # type: ignore[union-attr]

    def _prune_stale_pending(self, now: float) -> None:
        if self._pending_ttl_s <= 0:
            return
        cutoff = now - self._pending_ttl_s
        stale_ids = [
            obj_id for obj_id, obj in self._pending_objects.items() if (obj.ts or now) < cutoff
        ]
        for obj_id in stale_ids:
            del self._pending_objects[obj_id]
            for track_id, mapped_id in list(self._track_id_map.items()):
                if mapped_id == obj_id:
                    del self._track_id_map[track_id]

    def _check_promotion(self, obj: Object) -> bool:
        """Move object from pending to permanent if threshold met."""
        if obj.detections_count >= self._min_detections:
            # Check if it's in pending
            if obj.object_id in self._pending_objects:
                # Promote to permanent
                del self._pending_objects[obj.object_id]
                self._objects[obj.object_id] = obj
                logger.info(
                    f"Promoted object {obj.object_id} ({obj.name}) to permanent "
                    f"with {obj.detections_count} detections"
                )
                return True
        return False

    # ─────────────────────────────────────────────────────────────────
    # Agent encoding
    # ─────────────────────────────────────────────────────────────────

    def agent_encode(self) -> list[dict[str, Any]]:
        """Encode permanent objects for agent consumption."""
        with self._lock:
            return [obj.agent_encode() for obj in self._objects.values()]

    def __len__(self) -> int:
        """Return number of permanent objects."""
        with self._lock:
            return len(self._objects)

    def __repr__(self) -> str:
        with self._lock:
            return f"ObjectDB(permanent={len(self._objects)}, pending={len(self._pending_objects)})"


__all__ = ["ObjectDB"]
