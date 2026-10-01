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
from typing import Protocol, TypedDict, TypeVar

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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


def pointcloud2_from_webrtc_lidar(
    raw_message: RawLidarMsg, *, stamp: Time | None = None
) -> PointCloud2:
    """Copy decoded WebRTC XYZ points into a generated cloud in the world frame.

    The raw sensor's floating-second stamp is used unless the connection supplies
    an explicit arrival stamp. No Open3D object or legacy message is constructed.
    """
    data = raw_message["data"]
    return pointcloud_from_xyz(
        data["data"]["points"],
        header=Header(
            frame_id="world", stamp=stamp if stamp is not None else time_from_seconds(data["stamp"])
        ),
    )


class StampedMessage(Protocol):
    header: Header


T = TypeVar("T", bound=StampedMessage)


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
            item.header.stamp = time_from_seconds(now())
            return item

        if not calibrated:
            if prev_raw is not None and to_seconds(item.header.stamp) != prev_raw:
                calibrated = True
                # lidar stamps advancing — using lidar time",
            prev_raw = to_seconds(item.header.stamp)
            n_seen += 1

        if prev_good is not None and to_seconds(item.header.stamp) <= prev_good:
            item.header.stamp = time_from_seconds(prev_good + default_period)

        prev_good = to_seconds(item.header.stamp)

        if not calibrated and n_seen >= calibration_frames:
            calibrated = True
            use_system_time = True
            logger.warning(
                "repair_stale_ts: lidar timestmaps frozen (%d calibration stamps equal) — using system time, upgrade your GO2 firmware",
                calibration_frames,
            )

        return item

    return ops.map(_repair)
