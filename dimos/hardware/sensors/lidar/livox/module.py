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

"""Python NativeModule wrapper for the Rust Livox Mid-360 driver.

Usage::
    from dimos.hardware.sensors.lidar.livox.module import Mid360
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        Mid360.blueprint(),  # host_ip auto-detected, lidar_ip defaults to the factory IP
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import os
import sys
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field, field_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.lidar.livox.net import resolve_host_ip
from dimos.hardware.sensors.lidar.livox.ports import (
    SDK_CMD_DATA_PORT,
    SDK_HOST_CMD_DATA_PORT,
    SDK_HOST_IMU_DATA_PORT,
    SDK_HOST_POINT_DATA_PORT,
    SDK_IMU_DATA_PORT,
    SDK_MULTICAST_GROUP,
    SDK_POINT_DATA_PORT,
)
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception


class Mid360Config(NativeModuleConfig):
    cwd: str | None = "rust"
    # The crate is a workspace member, so cargo builds into the repo-root target dir.
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "mid360_native")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})
    host_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_HOST_IP"))
    lidar_ip: str = Field(
        default_factory=lambda: os.environ.get("DIMOS_MID360_LIDAR_IP", "192.168.1.155")
    )
    frequency: float = 10.0
    enable_imu: bool = True
    # Replay this capture instead of a live sensor. host_ip/lidar_ip are unused.
    pcap: str | None = None
    # Replay speed relative to capture time. None runs flat-out.
    replay_rate: float | None = Field(default=1.0, gt=0)
    # Multicast group the device streams to. None receives unicast only, which
    # loopback replay needs and macOS requires (see virtual_mid360).
    multicast_ip: str | None = Field(
        default_factory=lambda: None if sys.platform == "darwin" else SDK_MULTICAST_GROUP
    )
    # Wire layout per point:
    #   "minimal" x,y,z,offset_time                    - 16 B (default)
    #   "full"    x,y,z,intensity,offset_time,tag,line - 22 B
    #   "legacy"  x,y,z,intensity                      - 16 B
    # Scan-undistorting estimators (FAST-LIVO2 etc.) need offset_time. No LIO in
    # the stack reads intensity, which only feeds viz and map coloring.
    point_format: Literal["full", "minimal", "legacy"] = "minimal"
    frame_id: str = "lidar_link"
    imu_frame_id: str = "imu_link"

    # SDK port configuration (see livox/ports.py for defaults)
    cmd_data_port: int = SDK_CMD_DATA_PORT
    point_data_port: int = SDK_POINT_DATA_PORT
    imu_data_port: int = SDK_IMU_DATA_PORT
    host_cmd_data_port: int = SDK_HOST_CMD_DATA_PORT
    host_point_data_port: int = SDK_HOST_POINT_DATA_PORT
    host_imu_data_port: int = SDK_HOST_IMU_DATA_PORT

    @field_validator("pcap")
    @classmethod
    def _pcap_path_present(cls, value: str | None) -> str | None:
        if value == "":
            raise ValueError("pcap is empty; set DIMOS_MID360_PCAP or pass a path")
        return value

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # The rust struct has every key. None crosses as an explicit null.
        for key in ("host_ip", "pcap", "replay_rate", "multicast_ip"):
            config[key] = getattr(self, key)
        return config


def _resolved_host_ip(config: Mid360Config) -> str | None:
    """Live mode derives host_ip from a NIC on the lidar's subnet. Replay skips it."""
    if config.pcap is not None:
        return config.host_ip
    return resolve_host_ip(config.lidar_ip, config.host_ip, label="Mid360")


class Mid360(NativeModule, perception.Lidar, perception.IMU):
    config: Mid360Config

    lidar: Out[PointCloud2]
    imu: Out[Imu]

    @rpc
    def start(self) -> None:
        self.config.host_ip = _resolved_host_ip(self.config)
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    Mid360()
