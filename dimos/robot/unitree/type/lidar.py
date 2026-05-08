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
import time
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


def repair_stale_ts(
    default_period: float = 0.130,
    calibration_frames: int = 10,
    now: Callable[[], float] = time.time,
) -> Callable[[Observable[T]], Observable[T]]:
    """Repair Unitree's stale-stamp bug.

    Older firmware doesn't update timestamps for the point clouds. In this case we set to system time.

    On new firmware, occasionally frames will revert back to the initial timestamp. In these cases, we update based on the default period.

    We calibrate through the first few frames to determine which correction method to use. Once it's been determined, it does not change.
    """
    prev_good: float | None = None
    prev_raw: float | None = None
    n_seen = 0
    calibrated = False
    use_system_time = False

    def _repair(item: T) -> T:
        nonlocal prev_good, prev_raw, n_seen, calibrated, use_system_time

        if use_system_time:
            item.ts = now()
            return item

        if not calibrated:
            if prev_raw is not None and item.ts != prev_raw:
                calibrated = True
                # lidar stamps advancing — using lidar time",
            prev_raw = item.ts
            n_seen += 1

        if prev_good is not None and item.ts <= prev_good:
            old = item.ts
            item.ts = prev_good + default_period
            logger.warning("repair_stale_ts: stale stamp %.6f → %.6f", old, item.ts)

        prev_good = item.ts

        if not calibrated and n_seen >= calibration_frames:
            calibrated = True
            use_system_time = True
            logger.warning(
                "repair_stale_ts: lidar timestmaps frozen (%d calibration stamps equal) — using system time, upgrade your GO2 firmware",
                calibration_frames,
            )

        return item

    return ops.map(_repair)
