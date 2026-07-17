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

"""Python NativeModule wrapper for the FAST-LIO2 + Livox Mid-360 binary.

Binds Livox SDK2 into FAST-LIO-NON-ROS for real-time LiDAR SLAM; outputs
sensor/body-frame point clouds (register via the odometry pose) and odometry
with covariance.

FAST-LIO tuning lives directly on ``FastLio2Config`` and is passed to the C++
binary as plain CLI args (no YAML).
"""

from __future__ import annotations

import os
import time
from typing import TYPE_CHECKING, Literal

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.lidar.livox.net import resolve_host_ip
from dimos.hardware.sensors.lidar.livox.ports import (
    SDK_CMD_DATA_PORT,
    SDK_HOST_CMD_DATA_PORT,
    SDK_HOST_IMU_DATA_PORT,
    SDK_HOST_LOG_DATA_PORT,
    SDK_HOST_POINT_DATA_PORT,
    SDK_HOST_PUSH_MSG_PORT,
    SDK_IMU_DATA_PORT,
    SDK_LOG_DATA_PORT,
    SDK_POINT_DATA_PORT,
    SDK_PUSH_MSG_PORT,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.frames import FRAME_ODOM
from dimos.spec import perception

# Human-readable enums; the C++ binary maps these strings to FAST-LIO's int codes.
LidarType = Literal["livox", "velodyne", "ouster"]
TimestampUnit = Literal["second", "millisecond", "microsecond", "nanosecond"]


class FastLio2Config(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/fastlio2_native"
    build_command: str | None = "nix build .#fastlio2_native"
    # Livox SDK hardware config. lidar_ip required; host_ip optional (auto-derived
    # from lidar_ip's subnet). Both fall back to DIMOS_FASTLIO_LIDAR_IP /
    # DIMOS_FASTLIO_HOST_IP.
    host_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_FASTLIO_HOST_IP"))
    lidar_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_FASTLIO_LIDAR_IP"))
    frequency: float = 10.0

    # Odometry is published as frame_id (fixed) -> sensor_frame_id (moving sensor),
    # and also broadcast on TF. The point cloud is stamped with sensor_frame_id
    frame_id: str = FRAME_ODOM
    sensor_frame_id: str = "mid360_link"

    # FAST-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    debug: bool = False

    # FAST-LIO tuning, passed to the binary as plain CLI args (read in main.cpp).
    # common
    time_sync_en: bool = False
    time_offset_lidar_to_imu: float = 0.0
    # preprocess
    lidar_type: LidarType = "livox"
    scan_line: int = 4
    scan_rate: int = 10  # velodyne only
    timestamp_unit: TimestampUnit = "microsecond"  # velodyne/ouster time field unit
    blind: float = 0.5  # spherical min range (m)
    # mapping
    # acc_cov down-weights the IMU accel prediction. 0.01 is high trust (fine for
    # drones); 1.0 is low trust (good for robot dogs that go up/down stairs).
    acc_cov: float = 1.0
    gyr_cov: float = 0.1
    b_acc_cov: float = 0.0001
    b_gyr_cov: float = 0.0001
    filter_size_surf: float = 0.1  # IESKF scan voxel leaf (m)
    filter_size_map: float = 0.1  # ikd-tree map voxel leaf (m)
    fov_degree: int = 360  # FAST-LIO reads this as an int
    det_range: float = 100.0
    extrinsic_est_en: bool = False  # online IMU-LiDAR extrinsic estimation
    extrinsic_t: list[float] = Field(default_factory=lambda: [-0.011, -0.02329, 0.04412])
    extrinsic_r: list[float] = Field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    # publish behaviour (passed to the binary as CLI args, not the YAML)
    scan_publish_en: bool = True  # false closes the lidar output
    dense_publish_en: bool = True  # false voxel-downsamples the published cloud

    # SDK port configuration (see livox/ports.py for defaults)
    cmd_data_port: int = SDK_CMD_DATA_PORT
    push_msg_port: int = SDK_PUSH_MSG_PORT
    point_data_port: int = SDK_POINT_DATA_PORT
    imu_data_port: int = SDK_IMU_DATA_PORT
    log_data_port: int = SDK_LOG_DATA_PORT
    host_cmd_data_port: int = SDK_HOST_CMD_DATA_PORT
    host_push_msg_port: int = SDK_HOST_PUSH_MSG_PORT
    host_point_data_port: int = SDK_HOST_POINT_DATA_PORT
    host_imu_data_port: int = SDK_HOST_IMU_DATA_PORT
    host_log_data_port: int = SDK_HOST_LOG_DATA_PORT


class FastLio2(NativeModule, perception.Lidar, perception.Odometry):
    config: FastLio2Config

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        self._validate_network()
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf))
        )

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.frame_id,
                child_frame_id=self.config.sensor_frame_id,
                translation=Vector3(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ),
                rotation=Quaternion(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ),
                ts=msg.ts or time.time(),
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _validate_network(self) -> None:
        lidar_ip = self.config.lidar_ip
        if not lidar_ip:
            raise RuntimeError(
                "FastLio2: lidar_ip not set — it's network-specific. Set it in the config "
                "or via the DIMOS_FASTLIO_LIDAR_IP env var."
            )
        # host_ip optional: derive the local NIC on lidar_ip's /24 when unset or
        # not one of our IPs (shared with the Mid360 driver).
        self.config.host_ip = resolve_host_ip(lidar_ip, self.config.host_ip, label="FastLio2")


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    FastLio2()
