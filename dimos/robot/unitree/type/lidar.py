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

"""Unitree WebRTC lidar message parsing utilities."""

import time
from typing import TypedDict

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs import PointCloud2

# Backwards compatibility alias for pickled data
LidarMessage = PointCloud2


class RawLidarPoints(TypedDict):
    points: np.ndarray  # type: ignore[type-arg]  # Shape (N, 3) array of 3D points [x, y, z]


class RawLidarData(TypedDict):
    """Data portion of the LIDAR message"""

    frame_id: str
    origin: list[float]
    resolution: float
    src_size: int
    stamp: float
    width: list[int]
    data: RawLidarPoints


class RawLidarMsg(TypedDict):
    """Static type definition for raw LIDAR message from Unitree WebRTC."""

    type: str
    topic: str
    data: RawLidarData


def pointcloud2_from_webrtc_lidar(raw_message: RawLidarMsg, ts: float | None = None) -> PointCloud2:
    """Convert a raw Unitree WebRTC lidar message to PointCloud2.

    Args:
        raw_message: Raw lidar message from Unitree WebRTC API
        ts: Optional timestamp override. If None, uses current time.

    Returns:
        PointCloud2 message with the lidar points
    """
    data = raw_message["data"]
    points = data["data"]["points"]

    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(points)

    return PointCloud2(
        pointcloud=pointcloud,
        # webrtc stamp is broken (e.g., "stamp": 1.758148e+09), use current time
        ts=ts if ts is not None else time.time(),
        frame_id="world",
    )
