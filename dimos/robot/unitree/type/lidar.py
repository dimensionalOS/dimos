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

from collections.abc import Callable
from typing import TypedDict, TypeVar

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.types.timestamped import Timestamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Backwards compatibility alias for pickled data
LidarMessage = PointCloud2


class RawLidarPoints(TypedDict):
    points: np.ndarray  # Shape (N, 3) array of 3D points [x, y, z]


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
        ts: Optional timestamp override. If None, uses the sensor stamp from
            ``raw_message["data"]["stamp"]``.

    The sensor stamp is authoritative when valid but Unitree's publisher
    occasionally re-emits a stale value on fresh scans — see
    :func:`repair_stale_ts` for the downstream repair.
    """
    data = raw_message["data"]
    points = data["data"]["points"]

    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(points)

    return PointCloud2(
        pointcloud=pointcloud,
        ts=ts if ts is not None else data["stamp"],
        frame_id="world",
    )


T = TypeVar("T", bound=Timestamped)


def repair_stale_ts(default_period: float = 0.130) -> Callable[[Observable[T]], Observable[T]]:
    """Repair Unitree's stale-stamp bug by forward-extrapolating non-monotonic stamps.

    Unitree's WebRTC publisher occasionally emits the same stale ``stamp`` on
    fresh scans (8 frames over 600s in the HK office recording, all sharing
    one stamp predating recording start). This pipeable operator detects a
    non-monotonic stamp and rewrites it to ``prev_good.ts + default_period``.
    Zero latency — emits each item immediately. Successive bad frames each
    advance by another ``default_period``.
    """
    prev_good: list[float | None] = [None]

    def _repair(item: T) -> T:
        if prev_good[0] is not None and item.ts <= prev_good[0]:
            old = item.ts
            item.ts = prev_good[0] + default_period
            logger.warning("repair_stale_ts: stale stamp %.6f → %.6f", old, item.ts)
        prev_good[0] = item.ts
        return item

    return ops.map(_repair)
