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

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import cast

import numpy as np
import pytest

from dimos.manipulation.planning.global_map_obstacle_bridge import (
    GLOBAL_MAP_OBSTACLE_ID,
    GlobalMapObstacleBridge,
    VoxelObstacleSpec,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass
class _Planning:
    calls: list[tuple[str, list[tuple[float, float, float]], float, str]] = field(
        default_factory=list
    )
    fail_times: int = 0

    def set_voxel_obstacle(
        self,
        name: str,
        points: list[tuple[float, float, float]],
        resolution: float,
        frame_id: str = "world",
    ) -> bool:
        if self.fail_times:
            self.fail_times -= 1
            raise RuntimeError("planning world is busy")
        self.calls.append((name, points, resolution, frame_id))
        return True


@pytest.fixture
def bridge() -> Iterator[tuple[GlobalMapObstacleBridge, _Planning]]:
    module = GlobalMapObstacleBridge(resolution=0.05)
    planning = _Planning()
    module._planning = cast("VoxelObstacleSpec", planning)
    yield module, planning
    module.dispose()


def _cloud(points: list[list[float]], frame_id: str = "world") -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)),
        frame_id=frame_id,
        timestamp=1.0,
    )


def test_a_snapshot_becomes_one_octree_obstacle(
    bridge: tuple[GlobalMapObstacleBridge, _Planning],
) -> None:
    module, planning = bridge

    asyncio.run(module.handle_global_map(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0]])))

    name, points, resolution, frame_id = planning.calls[0]
    assert name == GLOBAL_MAP_OBSTACLE_ID
    np.testing.assert_allclose(points, [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)], atol=1e-7)
    assert resolution == 0.05
    assert frame_id == "world"


def test_every_snapshot_reuses_the_same_obstacle_id(
    bridge: tuple[GlobalMapObstacleBridge, _Planning],
) -> None:
    # The mapper republishes a complete map, not a delta, so the planner must
    # see a replacement rather than an accumulating pile of obstacles.
    module, planning = bridge

    asyncio.run(module.handle_global_map(_cloud([[0.0, 0.0, 0.0]])))
    asyncio.run(module.handle_global_map(_cloud([[1.0, 0.0, 0.0]])))

    assert [call[0] for call in planning.calls] == [GLOBAL_MAP_OBSTACLE_ID] * 2
    np.testing.assert_allclose(planning.calls[-1][1], [(1.0, 0.0, 0.0)], atol=1e-7)


def test_an_empty_map_clears_the_obstacle(
    bridge: tuple[GlobalMapObstacleBridge, _Planning],
) -> None:
    module, planning = bridge

    asyncio.run(module.handle_global_map(_cloud([])))

    assert planning.calls == [(GLOBAL_MAP_OBSTACLE_ID, [], 0.05, "world")]


def test_a_cloud_in_the_wrong_frame_is_rejected_not_reinterpreted(
    bridge: tuple[GlobalMapObstacleBridge, _Planning],
) -> None:
    module, planning = bridge

    asyncio.run(module.handle_global_map(_cloud([[0.0, 0.0, 0.0]], frame_id="odom")))

    assert planning.calls == []


def test_a_non_finite_map_is_rejected(
    bridge: tuple[GlobalMapObstacleBridge, _Planning],
) -> None:
    module, planning = bridge

    asyncio.run(module.handle_global_map(_cloud([[0.0, 0.0, float("inf")]])))

    assert planning.calls == []


def test_an_oversized_map_is_dropped_rather_than_pushed_through_rpc() -> None:
    # Every point crosses worker RPC as a pickled tuple. The last good map stays
    # in the planner instead of the pipe stalling on one that will not fit.
    module = GlobalMapObstacleBridge(resolution=0.05, max_points=4)
    planning = _Planning()
    module._planning = cast("VoxelObstacleSpec", planning)
    try:
        asyncio.run(module.handle_global_map(_cloud([[float(i), 0.0, 0.0] for i in range(5)])))
        assert planning.calls == []

        asyncio.run(module.handle_global_map(_cloud([[float(i), 0.0, 0.0] for i in range(4)])))
        assert len(planning.calls) == 1
    finally:
        module.dispose()


def test_a_transient_planning_failure_is_retried(
    bridge: tuple[GlobalMapObstacleBridge, _Planning],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module, planning = bridge
    planning.fail_times = 2
    monkeypatch.setattr(
        "dimos.manipulation.planning.global_map_obstacle_bridge._RETRY_DELAY_S", 0.0
    )

    asyncio.run(module.handle_global_map(_cloud([[0.0, 0.0, 0.0]])))

    assert len(planning.calls) == 1
