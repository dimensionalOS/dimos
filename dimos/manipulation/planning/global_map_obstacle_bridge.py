# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Publish the latest mapping snapshot as one planning-world obstacle."""

from __future__ import annotations

import asyncio
from typing import Literal, Protocol

import numpy as np
from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GLOBAL_MAP_OBSTACLE_ID = "mapping/global-voxel-map"
_RETRY_DELAY_S = 1.0


class PlanningObstacleMutationSpec(Spec, Protocol):
    """Typed planning-world obstacle mutation RPCs."""

    def add_obstacle(self, obstacle: Obstacle) -> str: ...

    def update_obstacle(self, obstacle: Obstacle) -> bool: ...

    def remove_obstacle(self, obstacle_id: str) -> bool: ...


class GlobalMapObstacleBridgeConfig(ModuleConfig):
    resolution: float = Field(default=0.05, gt=0.0)
    planning_frame: str = Field(default="world", min_length=1)
    world_backend: Literal["roboplan"] = "roboplan"
    default_rpc_timeout: float = 2.0


class GlobalMapObstacleBridge(Module):
    """Reconcile complete mapper snapshots through a stable obstacle ID."""

    config: GlobalMapObstacleBridgeConfig  # type: ignore[assignment]
    global_map: In[PointCloud2]
    _planning_obstacles: PlanningObstacleMutationSpec

    async def handle_global_map(self, cloud: PointCloud2) -> None:
        """Apply the newest complete map; the module dispatcher coalesces backlog."""
        while True:
            try:
                await asyncio.to_thread(self._reconcile, cloud)
                return
            except ValueError:
                logger.exception("Rejected invalid global map at stamp %.6f", cloud.ts)
                return
            except Exception:
                logger.exception(
                    "Failed to reconcile global map obstacle at stamp %.6f; retrying",
                    cloud.ts,
                )
                await asyncio.sleep(_RETRY_DELAY_S)

    def _reconcile(self, cloud: PointCloud2) -> None:
        if cloud.frame_id != self.bridge_config.planning_frame:
            raise ValueError(
                f"Global map frame '{cloud.frame_id}' does not match planning frame "
                f"'{self.bridge_config.planning_frame}'"
            )
        points = np.asarray(cloud.points_f32(), dtype=np.float64)
        if not np.isfinite(points).all():
            raise ValueError("Global map contains non-finite points")
        if len(points) == 0:
            self._planning_obstacles.remove_obstacle(GLOBAL_MAP_OBSTACLE_ID)
            return

        obstacle = Obstacle(
            name=GLOBAL_MAP_OBSTACLE_ID,
            obstacle_type=ObstacleType.OCTREE,
            pose=PoseStamped(frame_id=self.bridge_config.planning_frame),
            points=points.copy(),
            octree_resolution=self.bridge_config.resolution,
        )
        if self._planning_obstacles.update_obstacle(obstacle):
            return
        obstacle_id = self._planning_obstacles.add_obstacle(obstacle)
        if obstacle_id != GLOBAL_MAP_OBSTACLE_ID:
            raise RuntimeError(f"Failed to register global map obstacle '{GLOBAL_MAP_OBSTACLE_ID}'")

    @property
    def bridge_config(self) -> GlobalMapObstacleBridgeConfig:
        return self.config  # type: ignore[return-value]


global_map_obstacle_bridge = GlobalMapObstacleBridge.blueprint
