# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Staging and obstacle-lifecycle synchronization for planning voxel snapshots."""

from __future__ import annotations

import threading

import numpy as np

from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PlanningCollisionSnapshot:
    """Keep the latest planning cloud and commit it through a WorldMonitor."""

    def __init__(self, resolution: float = 0.05, planning_frame: str = "world") -> None:
        if resolution <= 0:
            raise ValueError("Planning collision snapshot resolution must be positive")
        if not planning_frame:
            raise ValueError("Planning collision snapshot planning_frame must not be empty")
        self.resolution = resolution
        self.planning_frame = planning_frame
        self._lock = threading.RLock()
        self._staged: PointCloud2 | None = None
        self._committed: PointCloud2 | None = None
        self._staged_generation = 0
        self._committed_generation = 0
        self._active_obstacle_id: str | None = None

    def stage(self, cloud: PointCloud2) -> None:
        """Validate and stage a complete cloud, replacing older staged input."""
        if cloud.frame_id != self.planning_frame:
            raise ValueError(
                f"Planning collision snapshot frame '{cloud.frame_id}' does not match "
                f"planning frame '{self.planning_frame}'"
            )
        points, _ = cloud.as_numpy()
        staged = PointCloud2.from_numpy(
            np.asarray(points, dtype=np.float64).copy(),
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
        )
        with self._lock:
            self._staged = staged
            self._staged_generation += 1

    def committed(self) -> PointCloud2 | None:
        """Return the fully committed cloud, or None before first commit."""
        with self._lock:
            return self._committed

    def staged(self) -> PointCloud2 | None:
        """Return the latest validated input, including uncommitted input."""
        with self._lock:
            return self._staged

    def synchronize(self, world_monitor: WorldMonitor) -> PointCloud2 | None:
        """Commit the latest staged cloud through stable-ID add/update/remove methods."""
        with self._lock:
            if self._staged is None or self._staged_generation == self._committed_generation:
                return self.committed()

            staged = self._staged
            generation = self._staged_generation
            points, _ = staged.as_numpy()
            points = np.asarray(points, dtype=np.float64).copy()

            if len(points) == 0:
                if self._active_obstacle_id is not None:
                    removed = world_monitor.remove_obstacle(self._active_obstacle_id)
                    if not removed:
                        raise RuntimeError(
                            "Failed to remove planning collision obstacle "
                            f"'{self._active_obstacle_id}'"
                        )
                self._active_obstacle_id = None
                self._committed = staged
                self._committed_generation = generation
                return self.committed()

            obstacle = Obstacle(
                name="planning-collision",
                obstacle_type=ObstacleType.OCTREE,
                pose=PoseStamped(frame_id=self.planning_frame),
                points=points,
                octree_resolution=self.resolution,
            )
            if self._active_obstacle_id is None:
                active_obstacle_id = world_monitor.add_obstacle(obstacle)
                if not active_obstacle_id:
                    raise RuntimeError("Failed to register planning collision obstacle")
                self._active_obstacle_id = active_obstacle_id
            elif not world_monitor.update_obstacle(self._active_obstacle_id, obstacle):
                raise RuntimeError(
                    f"Planning collision obstacle '{self._active_obstacle_id}' is missing"
                )
            self._committed = staged
            self._committed_generation = generation
            return self.committed()
