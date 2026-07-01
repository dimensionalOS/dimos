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
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        Mid360.blueprint(),  # host_ip auto-detected; set lidar_ip if not the factory default
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from pydantic import Field

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
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception


class Mid360Config(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/mid360_native"
    build_command: str | None = "nix build .#mid360_native"
    host_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_HOST_IP"))
    lidar_ip: str = Field(
        default_factory=lambda: os.environ.get("DIMOS_MID360_LIDAR_IP", "192.168.1.155")
    )
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
    config: Mid360Config

    lidar: Out[PointCloud2]
    imu: Out[Imu]

    @rpc
    def start(self) -> None:
        # Auto-derive host_ip from a local NIC on the lidar's subnet (shared with
        # Point-LIO) when the configured value isn't one of our IPs.
        self.config.host_ip = resolve_host_ip(
            self.config.lidar_ip, self.config.host_ip, label="Mid360"
        )
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    Mid360()
