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

"""Python NativeModule wrapper for the C++ Livox Mid-360 driver.

Usage::
    from dimos.hardware.sensors.lidar.livox.module import Mid360
    from dimos.core.blueprints import autoconnect

    autoconnect(
        Mid360.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from dimos.core import Out  # noqa: TC001
from dimos.core.native_module import NativeModule, NativeModuleConfig
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
from dimos.msgs.sensor_msgs.Imu import Imu  # noqa: TC001
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: TC001
from dimos.spec import perception


@dataclass(kw_only=True)
class Mid360Config(NativeModuleConfig):
    """Config for the C++ Mid-360 native module."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/mid360_native"
    build_command: str | None = "nix build .#mid360_native"

    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0
    enable_imu: bool = True
    frame_id: str = "lidar_link"
    imu_frame_id: str = "imu_link"

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


class Mid360(NativeModule, perception.Lidar, perception.IMU):
    """Livox Mid-360 LiDAR module backed by a native C++ binary.

    Ports:
        lidar (Out[PointCloud2]): Point cloud frames at configured frequency.
        imu (Out[Imu]): IMU data at ~200 Hz (if enabled).
    """

    config: Mid360Config
    default_config = Mid360Config

    lidar: Out[PointCloud2]
    imu: Out[Imu]


mid360_module = Mid360.blueprint

__all__ = [
    "Mid360",
    "Mid360Config",
    "mid360_module",
]

# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    Mid360()
