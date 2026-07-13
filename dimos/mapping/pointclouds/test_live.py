#!/usr/bin/env python3
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

import numpy as np

from dimos.mapping.pointclouds.live import LidarPointCloudClient
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_empty_snapshot_is_zero_by_three() -> None:
    client = LidarPointCloudClient()
    snapshot = client.snapshot()
    assert snapshot.shape == (0, 3)
    assert snapshot.dtype == np.float32
    assert client.message_count == 0


def test_lidar_chunks_accumulate() -> None:
    client = LidarPointCloudClient()
    client._on_lidar(PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32)))
    client._on_lidar(PointCloud2.from_numpy(np.array([[2.0, 0.0, 0.0]], dtype=np.float32)))

    snapshot = client.snapshot()
    assert snapshot.shape == (2, 3)
    assert client.message_count == 2


def test_global_map_included_alongside_lidar_chunks() -> None:
    client = LidarPointCloudClient()
    client._on_lidar(PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32)))
    client._on_global_map(
        PointCloud2.from_numpy(
            np.array([[5.0, 5.0, 5.0], [6.0, 6.0, 6.0]], dtype=np.float32)
        )
    )

    snapshot = client.snapshot()
    assert snapshot.shape == (3, 3)


def test_global_map_is_latest_wins() -> None:
    client = LidarPointCloudClient()
    client._on_global_map(PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32)))
    client._on_global_map(PointCloud2.from_numpy(np.array([[2.0, 0.0, 0.0]], dtype=np.float32)))

    snapshot = client.snapshot()
    assert snapshot.shape == (1, 3)
    assert snapshot[0, 0] == 2.0


def test_clear_drops_lidar_chunks_but_keeps_global_map() -> None:
    client = LidarPointCloudClient()
    client._on_lidar(PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32)))
    client._on_global_map(PointCloud2.from_numpy(np.array([[2.0, 0.0, 0.0]], dtype=np.float32)))

    client.clear()

    snapshot = client.snapshot()
    assert snapshot.shape == (1, 3)
    assert snapshot[0, 0] == 2.0
    assert client.message_count == 1


def test_empty_pointcloud_message_is_ignored() -> None:
    client = LidarPointCloudClient()
    client._on_lidar(PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32)))

    assert client.message_count == 0
    assert client.snapshot().shape == (0, 3)
