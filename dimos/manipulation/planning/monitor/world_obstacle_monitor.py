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
from typing import TYPE_CHECKING

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
    ):
        """Create a world obstacle monitor.

        Args:
            world: WorldSpec instance to update
            lock: Shared lock for thread-safe access
            detection_timeout: Time before removing stale detections (seconds)
        """
        self._world = world
        self._lock = lock
        self._detection_timeout = detection_timeout

        # Track obstacles from different sources
        self._collision_objects: dict[str, str] = {}  # msg_id -> obstacle_id
        self._perception_objects: dict[str, str] = {}  # detection_id -> obstacle_id
        self._perception_timestamps: dict[str, float] = {}  # detection_id -> timestamp

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
