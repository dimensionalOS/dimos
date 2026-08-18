# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import cast

import numpy as np
import pytest

from dimos.manipulation.planning.global_map_obstacle_bridge import (
    GLOBAL_MAP_OBSTACLE_ID,
    GlobalMapObstacleBridge,
    PlanningObstacleMutationSpec,
)
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass
class _PlanningObstacles:
    obstacles: dict[str, Obstacle] = field(default_factory=dict)
    updates: int = 0
    additions: int = 0

    def add_obstacle(self, obstacle: Obstacle) -> str:
        self.additions += 1
        if obstacle.name in self.obstacles:
            return ""
        self.obstacles[obstacle.name] = obstacle
        return obstacle.name

    def update_obstacle(self, obstacle: Obstacle) -> bool:
        self.updates += 1
        if obstacle.name not in self.obstacles:
            return False
        self.obstacles[obstacle.name] = obstacle
        return True

    def remove_obstacle(self, obstacle_id: str) -> bool:
        return self.obstacles.pop(obstacle_id, None) is not None


@pytest.fixture
def bridge() -> Iterator[tuple[GlobalMapObstacleBridge, _PlanningObstacles]]:
    module = GlobalMapObstacleBridge(resolution=0.05)
    planning = _PlanningObstacles()
    module._planning_obstacles = cast("PlanningObstacleMutationSpec", planning)
    yield module, planning
    module.dispose()


def _cloud(points: list[list[float]], frame_id: str = "world") -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)),
        frame_id=frame_id,
        timestamp=12.5,
    )


def test_reconcile_adds_then_updates_one_octree(
    bridge: tuple[GlobalMapObstacleBridge, _PlanningObstacles],
) -> None:
    module, planning = bridge

    module._reconcile(_cloud([[1.0, 2.0, 3.0]]))
    module._reconcile(_cloud([[4.0, 5.0, 6.0]]))

    assert planning.additions == 1
    assert planning.updates == 2
    obstacle = planning.obstacles[GLOBAL_MAP_OBSTACLE_ID]
    assert obstacle.obstacle_type is ObstacleType.OCTREE
    assert obstacle.octree_resolution == 0.05
    np.testing.assert_allclose(obstacle.points, [[4.0, 5.0, 6.0]])


def test_empty_map_removes_obstacle_idempotently(
    bridge: tuple[GlobalMapObstacleBridge, _PlanningObstacles],
) -> None:
    module, planning = bridge
    module._reconcile(_cloud([[1.0, 2.0, 3.0]]))

    module._reconcile(_cloud([]))
    module._reconcile(_cloud([]))

    assert planning.obstacles == {}


def test_wrong_frame_is_rejected(
    bridge: tuple[GlobalMapObstacleBridge, _PlanningObstacles],
) -> None:
    module, planning = bridge

    with pytest.raises(ValueError, match="does not match"):
        module._reconcile(_cloud([[1.0, 2.0, 3.0]], frame_id="odom"))

    assert planning.obstacles == {}


def test_non_roboplan_backend_is_rejected() -> None:
    with pytest.raises(ValueError):
        GlobalMapObstacleBridge(world_backend="drake")
