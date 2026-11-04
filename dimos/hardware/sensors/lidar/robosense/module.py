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

"""Python NativeModule wrapper for the Rust RoboSense lidar driver.

Decodes RoboSense MSOP/DIFOP/IMU packets with the robosense-lidar crate
(https://github.com/jeff-hykin/robosense-lidar). Airy is verified on hardware;
Helios, Bpearl, RS32 and E1 are ported from RoboSense's rs_driver.

Usage::
    from dimos.hardware.sensors.lidar.robosense.module import RoboSenseLidar
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    ModuleCoordinator.build(autoconnect(
        RoboSenseLidar.blueprint(model="airy", lidar_ip="192.168.1.200"),
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import ipaddress
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field, field_validator

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception
from dimos.utils.generic import get_local_ips

# Factory-default ports (rs_driver's defaults; only the Airy sends IMU packets).
DEFAULT_MSOP_PORT = 6699
DEFAULT_DIFOP_PORT = 7788
DEFAULT_AIRY_IMU_PORT = 6688


class RoboSenseLidarConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/lidar/robosense/rust"
    executable: str = "result/bin/robosense_native"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})
    model: Literal["airy", "helios", "bpearl", "rs32", "e1"] = "airy"
    # Keep only packets from this sensor. None accepts any sender.
    lidar_ip: str | None = None
    # Local interface (IPv4 address or name) for the multicast join / unicast bind.
    # None derives it from lidar_ip: the local address on the lidar's /24.
    interface: str | None = None
    # Multicast group the sensor streams to. None receives unicast.
    multicast_ip: str | None = None
    msop_port: int = DEFAULT_MSOP_PORT
    difop_port: int = DEFAULT_DIFOP_PORT
    # None skips the IMU stream (models without one ignore it).
    imu_port: int | None = DEFAULT_AIRY_IMU_PORT
    # Range filter in metres; None uses the model's own range.
    min_distance: float | None = None
    max_distance: float | None = None
    # Stamp with the sensor clock instead of arrival time; only for a PTP/GPS-synced sensor.
    use_lidar_clock: bool = False
    frame_id: str = "lidar_link"
    # IMU samples are rotated into the lidar's axes (the Airy's DIFOP calibration).
    imu_frame_id: str = "lidar_link"
    # Replay this capture instead of a live sensor.
    pcap: str | None = None
    # Replay speed relative to capture time. None runs flat-out.
    replay_rate: float | None = Field(default=1.0, gt=0)
    # Seconds a replay holds its first packet, so consumers can subscribe first.
    replay_delay: float = Field(default=0.0, ge=0)

    @field_validator("pcap")
    @classmethod
    def _pcap_path_present(cls, value: str | None) -> str | None:
        if value == "":
            raise ValueError("pcap is empty; set DIMOS_ROBOSENSE_PCAP or pass a path")
        return value

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # The rust struct has every key. None crosses as an explicit null.
        for key in (
            "lidar_ip",
            "interface",
            "multicast_ip",
            "imu_port",
            "min_distance",
            "max_distance",
            "pcap",
            "replay_rate",
        ):
            config[key] = getattr(self, key)
        return config


def _resolved_interface(config: RoboSenseLidarConfig) -> str | None:
    """The configured interface, else the local address on lidar_ip's /24, else None."""
    if config.interface or config.pcap is not None or not config.lidar_ip:
        return config.interface
    lidar_net = ipaddress.IPv4Network(f"{config.lidar_ip}/24", strict=False)
    for ip, _name in get_local_ips():
        if ipaddress.IPv4Address(ip) in lidar_net:
            return ip
    return None


class RoboSenseLidar(NativeModule, perception.Lidar, perception.IMU):
    config: RoboSenseLidarConfig

    lidar: Out[PointCloud2]
    imu: Out[Imu]

    @rpc
    def start(self) -> None:
        self.config.interface = _resolved_interface(self.config)
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    RoboSenseLidar()
