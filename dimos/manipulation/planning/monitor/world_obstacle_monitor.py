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

"""
World Obstacle Monitor

Monitors obstacle updates and applies them to a WorldSpec instance.
This is the WorldSpec-based replacement for WorldGeometryMonitor.

Example:
    monitor = WorldObstacleMonitor(world, lock)
    monitor.start()
    monitor.on_collision_object(collision_msg)  # Called by subscriber
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from dimos.manipulation.planning.spec import (
    CollisionObjectMessage,
    Obstacle,
    ObstacleType,
)
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    import threading

    from dimos.manipulation.planning.spec import WorldSpec
    from dimos.msgs.vision_msgs import Detection3D
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


class WorldObstacleMonitor:
    """Monitors world obstacles and updates WorldSpec.

    This class handles updates from:
    - Explicit collision objects (CollisionObjectMessage)
    - Perception detections (Detection3D from dimos.msgs.vision_msgs)

    ## Thread Safety

    All obstacle operations are protected by the provided lock.
    Callbacks can be called from any thread.

    ## Comparison with WorldGeometryMonitor

    - WorldGeometryMonitor: Works with PlanningScene ABC
    - WorldObstacleMonitor: Works with WorldSpec Protocol
    """

    def __init__(
        self,
        world: WorldSpec,
        lock: threading.RLock,
        detection_timeout: float = 2.0,
        use_mesh_obstacles: bool = True,
    ):
        """Create a world obstacle monitor.

        Args:
            world: WorldSpec instance to update
            lock: Shared lock for thread-safe access
            detection_timeout: Time before removing stale detections (seconds)
            use_mesh_obstacles: Use convex hull meshes from pointclouds instead of bounding boxes
        """
        self._world = world
        self._lock = lock
        self._detection_timeout = detection_timeout
        self._use_mesh_obstacles = use_mesh_obstacles

        # Track obstacles from different sources
        self._collision_objects: dict[str, str] = {}  # msg_id -> obstacle_id
        self._perception_objects: dict[str, str] = {}  # detection_id -> obstacle_id
        self._perception_timestamps: dict[str, float] = {}  # detection_id -> timestamp

        # Object-based cache (from ObjectDB, keyed by object_id)
        # object_id -> (Object, first_seen, last_seen)
        self._object_cache: dict[str, tuple[Object, float, float]] = {}
        # object_id -> obstacle_id (objects currently added to Drake world)
        self._object_obstacles: dict[str, str] = {}

        # Running state
        self._running = False

        # Callbacks: (operation, obstacle_id, obstacle) where operation is "add"/"update"/"remove"
        self._obstacle_callbacks: list[Callable[[str, str, Obstacle | None], None]] = []

    def start(self) -> None:
        """Start the obstacle monitor."""
        self._running = True
        logger.info("World obstacle monitor started")

    def stop(self) -> None:
        """Stop the obstacle monitor."""
        self._running = False
        logger.info("World obstacle monitor stopped")

    def is_running(self) -> bool:
        """Check if monitor is running."""
        return self._running

    def on_collision_object(self, msg: CollisionObjectMessage) -> None:
        """Handle explicit collision object message.

        Args:
            msg: Collision object message
        """
        if not self._running:
            return

        with self._lock:
            if msg.operation == "add":
                self._add_collision_object(msg)
            elif msg.operation == "remove":
                self._remove_collision_object(msg.id)
            elif msg.operation == "update":
                self._update_collision_object(msg)
            else:
                logger.warning(f"Unknown collision object operation: {msg.operation}")

    def _add_collision_object(self, msg: CollisionObjectMessage) -> None:
        """Add a collision object from message."""
        if msg.id in self._collision_objects:
            logger.debug(f"Collision object '{msg.id}' already exists, updating")
            self._update_collision_object(msg)
            return

        obstacle = self._msg_to_obstacle(msg)
        if obstacle is None:
            logger.warning(f"Failed to create obstacle from message: {msg.id}")
            return

        obstacle_id = self._world.add_obstacle(obstacle)
        self._collision_objects[msg.id] = obstacle_id

        logger.debug(f"Added collision object '{msg.id}' as '{obstacle_id}'")

        # Notify callbacks
        for callback in self._obstacle_callbacks:
            try:
                callback("add", obstacle_id, obstacle)
            except Exception as e:
                logger.error(f"Obstacle callback error: {e}")

    def _remove_collision_object(self, msg_id: str) -> None:
        """Remove a collision object."""
        if msg_id not in self._collision_objects:
            logger.debug(f"Collision object '{msg_id}' not found")
            return

        obstacle_id = self._collision_objects[msg_id]
        self._world.remove_obstacle(obstacle_id)
        del self._collision_objects[msg_id]

        logger.debug(f"Removed collision object '{msg_id}'")

        # Notify callbacks
        for callback in self._obstacle_callbacks:
            try:
                callback("remove", obstacle_id, None)
            except Exception as e:
                logger.error(f"Obstacle callback error: {e}")

    def _update_collision_object(self, msg: CollisionObjectMessage) -> None:
        """Update a collision object pose."""
        if msg.id not in self._collision_objects:
            # Treat as add if doesn't exist
            self._add_collision_object(msg)
            return

        obstacle_id = self._collision_objects[msg.id]

        if msg.pose is not None:
            self._world.update_obstacle_pose(obstacle_id, msg.pose)
            logger.debug(f"Updated collision object '{msg.id}' pose")

        # Notify callbacks
        for callback in self._obstacle_callbacks:
            try:
                callback("update", obstacle_id, None)
            except Exception as e:
                logger.error(f"Obstacle callback error: {e}")

    def _msg_to_obstacle(self, msg: CollisionObjectMessage) -> Obstacle | None:
        """Convert collision object message to Obstacle."""
        if msg.primitive_type is None or msg.pose is None or msg.dimensions is None:
            return None

        type_map = {
            "box": ObstacleType.BOX,
            "sphere": ObstacleType.SPHERE,
            "cylinder": ObstacleType.CYLINDER,
        }

        obstacle_type = type_map.get(msg.primitive_type.lower())
        if obstacle_type is None:
            logger.warning(f"Unknown primitive type: {msg.primitive_type}")
            return None

        return Obstacle(
            name=msg.id,
            obstacle_type=obstacle_type,
            pose=msg.pose,
            dimensions=msg.dimensions,
            color=msg.color,
        )

    def on_detections(self, detections: list[Detection3D]) -> None:
        """Handle perception detection results.

        Updates obstacles based on detections:
        - Adds new obstacles for new detections
        - Updates existing obstacles
        - Removes obstacles for detections that are no longer present

        Args:
            detections: List of Detection3D messages from dimos.msgs.vision_msgs
        """
        if not self._running:
            return

        with self._lock:
            current_time = time.time()
            seen_ids = set()

            for detection in detections:
                det_id = detection.id
                seen_ids.add(det_id)

                pose = self._detection3d_to_pose(detection)

                if det_id in self._perception_objects:
                    # Update existing obstacle
                    obstacle_id = self._perception_objects[det_id]
                    self._world.update_obstacle_pose(obstacle_id, pose)
                    self._perception_timestamps[det_id] = current_time
                else:
                    # Add new obstacle
                    obstacle = self._detection_to_obstacle(detection)
                    obstacle_id = self._world.add_obstacle(obstacle)
                    self._perception_objects[det_id] = obstacle_id
                    self._perception_timestamps[det_id] = current_time

                    logger.debug(f"Added perception object '{det_id}' as '{obstacle_id}'")

                    # Notify callbacks
                    for callback in self._obstacle_callbacks:
                        try:
                            callback("add", obstacle_id, obstacle)
                        except Exception as e:
                            logger.error(f"Obstacle callback error: {e}")

            # Remove stale detections
            self._cleanup_stale_detections(current_time, seen_ids)

    def _detection3d_to_pose(self, detection: Detection3D) -> PoseStamped:
        """Convert Detection3D bbox.center to PoseStamped."""
        center = detection.bbox.center
        return PoseStamped(
            position=center.position,
            orientation=center.orientation,
        )

    def _detection_to_obstacle(self, detection: Detection3D) -> Obstacle:
        """Convert Detection3D to Obstacle."""
        pose = self._detection3d_to_pose(detection)
        size = detection.bbox.size
        return Obstacle(
            name=f"detection_{detection.id}",
            obstacle_type=ObstacleType.BOX,
            pose=pose,
            dimensions=(size.x, size.y, size.z),
            color=(0.2, 0.8, 0.2, 0.6),  # Green for perception objects
        )

    def _cleanup_stale_detections(
        self,
        current_time: float,
        seen_ids: set[str],
    ) -> None:
        """Remove detections that haven't been seen recently."""
        stale_ids = []

        for det_id, timestamp in self._perception_timestamps.items():
            age = current_time - timestamp
            if det_id not in seen_ids and age > self._detection_timeout:
                stale_ids.append(det_id)

        for det_id in stale_ids:
            obstacle_id = self._perception_objects[det_id]
            removed = self._world.remove_obstacle(obstacle_id)
            if not removed:
                logger.warning(f"Obstacle '{obstacle_id}' not found in world during cleanup")
            del self._perception_objects[det_id]
            del self._perception_timestamps[det_id]

            logger.debug(f"Removed stale perception object '{det_id}'")

            # Notify callbacks
            for callback in self._obstacle_callbacks:
                try:
                    callback("remove", obstacle_id, None)
                except Exception as e:
                    logger.error(f"Obstacle callback error: {e}")

    def add_static_obstacle(
        self,
        name: str,
        obstacle_type: str,
        pose: PoseStamped,
        dimensions: tuple[float, ...],
        color: tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8),
    ) -> str:
        """Manually add a static obstacle.

        Args:
            name: Unique name for the obstacle
            obstacle_type: Type ("box", "sphere", "cylinder")
            pose: Pose of the obstacle in world frame
            dimensions: Type-specific dimensions
            color: RGBA color

        Returns:
            Obstacle ID
        """
        msg = CollisionObjectMessage(
            id=name,
            operation="add",
            primitive_type=obstacle_type,
            pose=pose,
            dimensions=dimensions,
            color=color,
        )
        self.on_collision_object(msg)
        return self._collision_objects.get(name, "")

    def remove_static_obstacle(self, name: str) -> bool:
        """Remove a static obstacle by name.

        Args:
            name: Name of the obstacle

        Returns:
            True if removed
        """
        if name not in self._collision_objects:
            return False

        msg = CollisionObjectMessage(id=name, operation="remove")
        self.on_collision_object(msg)
        return True

    def clear_all_obstacles(self) -> None:
        """Remove all tracked obstacles."""
        with self._lock:
            # Clear collision objects
            for msg_id in list(self._collision_objects.keys()):
                self._remove_collision_object(msg_id)

            # Clear perception objects
            for det_id, obstacle_id in list(self._perception_objects.items()):
                self._world.remove_obstacle(obstacle_id)
                del self._perception_objects[det_id]
                del self._perception_timestamps[det_id]

    def get_obstacle_count(self) -> int:
        """Get total number of tracked obstacles."""
        with self._lock:
            return len(self._collision_objects) + len(self._perception_objects)

    def add_obstacle_callback(
        self,
        callback: Callable[[str, str, Obstacle | None], None],
    ) -> None:
        """Add callback for obstacle changes.

        Args:
            callback: Function called with (operation, obstacle_id, obstacle)
                     where operation is "add", "update", or "remove"
        """
        self._obstacle_callbacks.append(callback)

    def remove_obstacle_callback(
        self,
        callback: Callable[[str, str, Obstacle | None], None],
    ) -> None:
        """Remove an obstacle callback."""
        if callback in self._obstacle_callbacks:
            self._obstacle_callbacks.remove(callback)

    # ============= Object-Based Perception (from ObjectDB) =============

    def on_objects(self, objects: list[object]) -> None:
        """Cache objects from ObjectDB (preserves stable object_id).

        Unlike on_detections(), this receives Object instances with stable IDs
        from ObjectDB deduplication, making the cache trivially keyed by object_id.

        Args:
            objects: List of Object instances from ObjectDB
        """
        if not self._running:
            return

        from dimos.perception.detection.type.detection3d.object import Object

        now = time.time()
        seen: set[str] = set()

        with self._lock:
            for obj in objects:
                if not isinstance(obj, Object):
                    continue
                oid = obj.object_id
                seen.add(oid)
                if oid in self._object_cache:
                    _, first, _ = self._object_cache[oid]
                    self._object_cache[oid] = (obj, first, now)
                else:
                    self._object_cache[oid] = (obj, now, now)

            # Remove objects no longer reported by ObjectDB
            stale = [oid for oid in self._object_cache if oid not in seen]
            for oid in stale:
                del self._object_cache[oid]

    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]:
        """Full sync: remove all object obstacles, re-add from cache.

        Args:
            min_duration: Minimum seconds an object must have been seen to be included

        Returns:
            List of added obstacles with object_id, obstacle_id, name, center, size
        """
        from dimos.perception.detection.type.detection3d.object import Object

        # Step 1: snapshot eligible objects under lock (fast)
        eligible: list[tuple[str, Object]] = []
        with self._lock:
            for oid, (obj, first_seen, last_seen) in self._object_cache.items():
                if not isinstance(obj, Object):
                    continue
                if last_seen - first_seen < min_duration:
                    continue
                eligible.append((oid, obj))

        # Step 2: compute obstacles OUTSIDE lock (convex hull can be slow)
        prepared: list[tuple[str, Object, Obstacle]] = []
        for oid, obj in eligible:
            obstacle = self._object_to_obstacle(obj)
            prepared.append((oid, obj, obstacle))

        # Step 3: apply to Drake world under lock (fast)
        with self._lock:
            for obs_id in self._object_obstacles.values():
                self._world.remove_obstacle(obs_id)
            self._object_obstacles.clear()

            result: list[dict[str, Any]] = []
            for oid, obj, obstacle in prepared:
                assert isinstance(obj, Object)
                obs_id = self._world.add_obstacle(obstacle)
                self._object_obstacles[oid] = obs_id
                result.append(
                    {
                        "object_id": oid,
                        "obstacle_id": obs_id,
                        "name": obj.name,
                        "center": [float(obj.center.x), float(obj.center.y), float(obj.center.z)],
                        "size": [float(obj.size.x), float(obj.size.y), float(obj.size.z)],
                    }
                )
                logger.debug(f"Added object obstacle '{oid}' ({obj.name}) as '{obs_id}'")

            return result

    def clear_perception_obstacles(self) -> int:
        """Remove all object obstacles from the planning world.

        Returns:
            Number of obstacles removed
        """
        with self._lock:
            count = len(self._object_obstacles)
            for obs_id in self._object_obstacles.values():
                self._world.remove_obstacle(obs_id)
            self._object_obstacles.clear()
            return count

    def get_perception_status(self) -> dict[str, int]:
        """Get perception obstacle status."""
        with self._lock:
            return {
                "cached": len(self._object_cache),
                "added": len(self._object_obstacles),
            }

    def get_cached_objects(self) -> list[Object]:
        """Get cached Object instances from perception.

        Returns raw Object instances for typed access to .name, .center, .size etc.
        """
        from dimos.perception.detection.type.detection3d.object import Object as _Object

        with self._lock:
            return [obj for obj, _, _ in self._object_cache.values() if isinstance(obj, _Object)]

    def list_cached_detections(self) -> list[dict[str, Any]]:
        """List cached detections from perception."""
        from dimos.perception.detection.type.detection3d.object import Object

        with self._lock:
            result: list[dict[str, Any]] = []
            for oid, (obj, first_seen, last_seen) in self._object_cache.items():
                if not isinstance(obj, Object):
                    continue
                result.append(
                    {
                        "object_id": oid,
                        "name": obj.name,
                        "center": [float(obj.center.x), float(obj.center.y), float(obj.center.z)],
                        "size": [float(obj.size.x), float(obj.size.y), float(obj.size.z)],
                        "duration": round(last_seen - first_seen, 1),
                        "in_world": oid in self._object_obstacles,
                    }
                )
            return result

    def list_added_obstacles(self) -> list[dict[str, Any]]:
        """List perception obstacles currently in the planning world."""
        from dimos.perception.detection.type.detection3d.object import Object

        with self._lock:
            result: list[dict[str, Any]] = []
            for oid, obs_id in self._object_obstacles.items():
                entry = self._object_cache.get(oid)
                if entry is None:
                    continue
                obj, first_seen, last_seen = entry
                if not isinstance(obj, Object):
                    continue
                result.append(
                    {
                        "object_id": oid,
                        "obstacle_id": obs_id,
                        "name": obj.name,
                        "center": [float(obj.center.x), float(obj.center.y), float(obj.center.z)],
                        "size": [float(obj.size.x), float(obj.size.y), float(obj.size.z)],
                    }
                )
            return result

    def _object_to_obstacle(self, obj: object) -> Obstacle:
        """Convert Object to obstacle. Uses bounding box by default, convex hull if use_mesh_obstacles=True."""
        from dimos.perception.detection.type.detection3d.object import Object

        assert isinstance(obj, Object)
        name = f"object_{obj.object_id}"

        # Try convex hull from pointcloud (opt-in)
        if self._use_mesh_obstacles and obj.pointcloud is not None:
            try:
                from dimos.manipulation.planning.utils.mesh_utils import (
                    pointcloud_to_convex_hull_obj,
                )

                points, _ = obj.pointcloud.as_numpy()
                if points is not None and points.shape[0] >= 4:
                    mesh_path = pointcloud_to_convex_hull_obj(points)
                    if mesh_path is not None:
                        return Obstacle(
                            name=name,
                            obstacle_type=ObstacleType.MESH,
                            pose=obj.pose,
                            color=(0.2, 0.8, 0.2, 0.6),
                            mesh_path=mesh_path,
                        )
            except Exception as e:
                logger.debug(f"Convex hull failed for {name}, falling back to box: {e}")

        # Default: bounding box
        return Obstacle(
            name=name,
            obstacle_type=ObstacleType.BOX,
            pose=obj.pose or PoseStamped(position=obj.center),
            dimensions=(float(obj.size.x), float(obj.size.y), float(obj.size.z)),
            color=(0.2, 0.8, 0.2, 0.6),
        )
