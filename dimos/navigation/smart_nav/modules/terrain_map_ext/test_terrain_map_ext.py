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

import threading
import time
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smart_nav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt


def _terrain_map_ext() -> TerrainMapExt:
    module = TerrainMapExt.__new__(TerrainMapExt)

    class _Cfg:
        voxel_size = 0.4
        decay_time = 8.0
        publish_rate = 1000.0
        max_range = 40.0
        robot_exclusion_radius = 0.0
        obstacle_height_threshold = 0.2

    module.config = _Cfg()  # type: ignore[assignment]
    module._lock = threading.Lock()
    module._voxels = {}
    module._robot_x = 0.0
    module._robot_y = 0.0
    module._has_odom = False
    module._running = False
    return module


def test_on_terrain_preserves_point_intensity() -> None:
    module = _terrain_map_ext()
    points = np.array([[0.0, 0.0, -0.45], [1.0, 0.0, 0.10]], dtype=np.float32)
    intensities = np.array([0.0, 0.31], dtype=np.float32)

    module._on_terrain(
        PointCloud2.from_numpy(points, frame_id="map", timestamp=1.0, intensity=intensities)
    )

    stored = sorted(value[3] for value in module._voxels.values())
    np.testing.assert_allclose(stored, [0.0, 0.31])


def test_on_terrain_skips_robot_exclusion_radius() -> None:
    module = _terrain_map_ext()
    module.config.robot_exclusion_radius = 1.0
    module._has_odom = True
    points = np.array([[0.6, 0.0, -0.45], [1.5, 0.0, 0.10]], dtype=np.float32)
    intensities = np.array([0.5, 0.5], dtype=np.float32)

    module._on_terrain(
        PointCloud2.from_numpy(points, frame_id="map", timestamp=1.0, intensity=intensities)
    )

    assert len(module._voxels) == 1
    stored = next(iter(module._voxels.values()))
    assert stored[0] == 1.5
    assert stored[3] == 0.5


def test_on_terrain_clears_stale_obstacle_voxel_in_clear_column() -> None:
    module = _terrain_map_ext()
    module._voxels[(0, 0, 1)] = (0.1, 0.1, 0.4, 0.5, time.time())

    points = np.array([[0.1, 0.1, -0.1]], dtype=np.float32)
    intensities = np.array([0.01], dtype=np.float32)

    module._on_terrain(
        PointCloud2.from_numpy(points, frame_id="map", timestamp=1.0, intensity=intensities)
    )

    assert all(
        value[3] < module.config.obstacle_height_threshold for value in module._voxels.values()
    )


def test_on_terrain_keeps_obstacle_column_when_current_obstacle_is_present() -> None:
    module = _terrain_map_ext()
    module._voxels[(0, 0, 1)] = (0.1, 0.1, 0.4, 0.5, time.time())
    points = np.array([[0.1, 0.1, -0.1], [0.1, 0.1, 0.4]], dtype=np.float32)
    intensities = np.array([0.01, 0.5], dtype=np.float32)

    module._on_terrain(
        PointCloud2.from_numpy(points, frame_id="map", timestamp=1.0, intensity=intensities)
    )

    assert any(
        value[3] >= module.config.obstacle_height_threshold for value in module._voxels.values()
    )


def test_publish_loop_preserves_point_intensity() -> None:
    module = _terrain_map_ext()
    now = time.time()
    module._voxels = {
        (0, 0, -2): (0.0, 0.0, -0.45, 0.0, now),
        (2, 0, 0): (1.0, 0.0, 0.10, 0.31, now),
    }
    module._running = True

    class _Out:
        def __init__(self) -> None:
            self.messages: list[Any] = []

        def publish(self, msg: Any) -> None:
            self.messages.append(msg)
            module._running = False

    module.terrain_map_ext = _Out()  # type: ignore[assignment]

    module._publish_loop()

    assert len(module.terrain_map_ext.messages) == 1
    np.testing.assert_allclose(
        np.sort(module.terrain_map_ext.messages[0].intensity_f32()),
        [0.0, 0.31],
    )
