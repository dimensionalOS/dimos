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

"""LiDAR integration for M20 quadruped via DDS.

The M20 publishes PointCloud2 on /LIDAR/POINTS at 10Hz (sensor_msgs/msg/PointCloud2)
and IMU on /IMU at 200Hz (sensor_msgs/msg/Imu) via ROS 2 Foxy DDS.

This module uses CycloneDDS to subscribe to the M20's LiDAR topic directly,
converting the raw DDS PointCloud2 to dimos PointCloud2 format.

Reference: M20 Software Development Guide section 2.1 (Sensor Driver Topics)
"""

import logging
import struct
import threading
import time
from typing import Callable, Optional

import numpy as np
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.msgs.sensor_msgs import PointCloud2 as DimosPointCloud2

logger = logging.getLogger(__name__)

# DDS availability check — CycloneDDS is an optional dependency
try:
    from cyclonedds.core import Listener
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.annotations import key
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False


if DDS_AVAILABLE:
    from dataclasses import dataclass, field
    from cyclonedds.idl.types import sequence, uint8, uint32, int32, float64

    @dataclass
    class Time(IdlStruct):
        """ROS 2 builtin_interfaces/msg/Time"""
        sec: int32 = 0
        nanosec: uint32 = 0

    @dataclass
    class Header(IdlStruct):
        """ROS 2 std_msgs/msg/Header"""
        stamp: Time = field(default_factory=Time)
        frame_id: str = ""

    @dataclass
    class PointField(IdlStruct):
        """ROS 2 sensor_msgs/msg/PointField"""
        name: str = ""
        offset: uint32 = 0
        datatype: uint8 = 0
        count: uint32 = 0

    @dataclass
    class DDSPointCloud2(IdlStruct):
        """ROS 2 sensor_msgs/msg/PointCloud2 as CycloneDDS IDL type."""
        header: Header = field(default_factory=Header)
        height: uint32 = 0
        width: uint32 = 0
        fields: sequence[PointField] = field(default_factory=list)
        is_bigendian: bool = False
        point_step: uint32 = 0
        row_step: uint32 = 0
        data: sequence[uint8] = field(default_factory=list)
        is_dense: bool = False


def _dds_to_dimos_pointcloud(dds_msg: "DDSPointCloud2") -> DimosPointCloud2:
    """Convert a DDS PointCloud2 to dimos PointCloud2 format.

    Extracts XYZ float32 points from the raw byte data.
    """
    raw = np.frombuffer(bytes(dds_msg.data), dtype=np.uint8)
    n_points = dds_msg.width * dds_msg.height
    point_step = dds_msg.point_step

    if n_points == 0 or len(raw) < n_points * point_step:
        return DimosPointCloud2(
            frame_id=dds_msg.header.frame_id or "world",
            ts=dds_msg.header.stamp.sec + dds_msg.header.stamp.nanosec * 1e-9,
        )

    # Find x, y, z field offsets
    offsets = {}
    for f in dds_msg.fields:
        if f.name in ("x", "y", "z"):
            offsets[f.name] = f.offset

    if not all(k in offsets for k in ("x", "y", "z")):
        logger.warning("PointCloud2 missing x/y/z fields, cannot extract points")
        return DimosPointCloud2(
            frame_id=dds_msg.header.frame_id or "world",
            ts=dds_msg.header.stamp.sec + dds_msg.header.stamp.nanosec * 1e-9,
        )

    # Extract XYZ as Nx3 float32 array
    points = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        points[i, 0] = struct.unpack_from("<f", raw, base + offsets["x"])[0]
        points[i, 1] = struct.unpack_from("<f", raw, base + offsets["y"])[0]
        points[i, 2] = struct.unpack_from("<f", raw, base + offsets["z"])[0]

    # Filter NaN/Inf points
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    return DimosPointCloud2.from_numpy(
        points,
        frame_id=dds_msg.header.frame_id or "world",
        timestamp=dds_msg.header.stamp.sec + dds_msg.header.stamp.nanosec * 1e-9,
    )


class M20LidarDDS:
    """Subscribes to M20's /LIDAR/POINTS DDS topic and produces dimos PointCloud2.

    Requires CycloneDDS (`pip install cyclonedds`).
    The M20 and this machine must be on the same network with DDS multicast enabled.

    Usage:
        lidar = M20LidarDDS()
        lidar.start()
        stream = lidar.pointcloud_stream()
        stream.subscribe(lambda pc: ...)
        lidar.stop()
    """

    def __init__(
        self,
        topic_name: str = "/LIDAR/POINTS",
        domain_id: int = 0,
    ):
        if not DDS_AVAILABLE:
            raise RuntimeError(
                "CycloneDDS not installed. Install with: pip install 'dimos[dds]'"
            )

        self._topic_name = topic_name
        self._domain_id = domain_id
        self._subject: Subject[DimosPointCloud2] = Subject()

        self._participant: Optional[DomainParticipant] = None
        self._reader: Optional[DataReader] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def pointcloud_stream(self) -> Observable[DimosPointCloud2]:
        return self._subject

    def start(self) -> None:
        if self._running:
            return

        self._participant = DomainParticipant(domain=self._domain_id)
        topic = Topic(self._participant, self._topic_name, DDSPointCloud2)
        self._reader = DataReader(self._participant, topic)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info(f"M20 LiDAR DDS started on topic {self._topic_name}")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._reader = None
        self._participant = None
        logger.info("M20 LiDAR DDS stopped")

    def _read_loop(self) -> None:
        while self._running and self._reader:
            try:
                samples = self._reader.take(N=10)
                for sample in samples:
                    if sample is not None:
                        pc = _dds_to_dimos_pointcloud(sample)
                        self._subject.on_next(pc)
            except Exception as e:
                logger.error(f"DDS read error: {e}")

            time.sleep(0.01)  # 100Hz poll, LiDAR publishes at 10Hz
