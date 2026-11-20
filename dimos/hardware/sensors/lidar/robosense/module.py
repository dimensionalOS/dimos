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

"""Python NativeModule wrapper for the Rust RoboSense Airy driver (the robosense-airy crate).

Usage::
    from dimos.hardware.sensors.lidar.robosense.module import Airy
    from dimos.core.coordination.blueprints import autoconnect

    autoconnect(
        Airy.blueprint(group="224.10.10.201", lidar_ip="10.21.33.201", msop_port=6691, difop_port=7781, imu_port=6681),
        SomeConsumer.blueprint(),
    )

The vendor driver can keep running: the sockets share the ports and multicast delivers to both.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.lidar.livox.net import resolve_host_ip
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception


class AiryConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/lidar/robosense/rust"
    executable: str = "result/bin/airy_native"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})
    # Multicast group the lidar sends to. None listens for unicast.
    group: str | None = None
    # Address of the local NIC on the lidar's network. None derives it from lidar_ip.
    host_ip: str | None = None
    # Only used to find host_ip.
    lidar_ip: str | None = None
    # rs_driver's defaults; the Airy's DIFOP lists what a unit is set to.
    msop_port: int = 6699
    difop_port: int = 7788
    imu_port: int = 6688
    enable_imu: bool = True
    # Wire layout per point:
    #   "full"    x,y,z,intensity (f32), offset_time (u32 ns after the stamp), ring (u16) - 22 B (default)
    #   "minimal" x,y,z,offset_time                                                       - 16 B
    #   "legacy"  x,y,z,intensity                                                         - 16 B
    point_format: Literal["full", "minimal", "legacy"] = "full"
    frame_id: str = "lidar_link"
    # IMU messages' frame. With imu_in_lidar_frame the samples are rotated into the lidar's axes (DIFOP extrinsic),
    # so this is normally the lidar frame too.
    imu_frame_id: str = "lidar_link"
    imu_in_lidar_frame: bool = True
    # The frame the lidar is mounted on; with one set, mount_parent_frame_id -> frame_id goes out on tf with each scan.
    mount_parent_frame_id: str | None = None
    # The lidar's position (m) and roll, pitch, yaw (rad) in mount_parent_frame_id.
    mount_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mount_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Range filter in metres; both 0 keeps the Airy's own 0.1..60 m.
    min_distance: float = Field(default=0.0, ge=0)
    max_distance: float = Field(default=0.0, ge=0)
    # True stamps with the lidar's clock (PTP-synced), False with the host clock at receipt.
    use_lidar_clock: bool = False
    # Azimuth window in degrees (start == end keeps all) and the azimuth where a scan ends.
    start_angle: float = 0.0
    end_angle: float = 360.0
    split_angle: float = 0.0
    # Voxel edge in metres (first point per voxel kept), 0 = off.
    voxel: float = Field(default=0.0, ge=0)
    # Keep every Nth point of a scan / every Nth scan.
    decimate_points: int = Field(default=1, ge=1)
    decimate_scans: int = Field(default=1, ge=1)
    # Decoded scans / IMU samples waiting to publish; one that finds the queue full is dropped. The IMU's
    # comes at 200 Hz, so its queue absorbs a few publishes' jitter.
    scan_queue: int = Field(default=1, ge=1)
    imu_queue: int = Field(default=8, ge=1)
    # SO_RCVBUF for the point socket in bytes. None keeps the OS default.
    recv_buffer_bytes: int | None = None

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # The rust struct has every key. None crosses as an explicit null.
        for key in ("group", "host_ip", "lidar_ip", "recv_buffer_bytes", "mount_parent_frame_id"):
            config[key] = getattr(self, key)
        # JSON arrays, as the rust [f64; 3] reads them
        for key in ("mount_xyz", "mount_rpy"):
            config[key] = list(getattr(self, key))
        return config


def _resolved_host_ip(config: AiryConfig) -> str | None:
    """host_ip as given, else the local NIC on lidar_ip's subnet, else None (any interface)."""
    if config.host_ip is not None or not config.lidar_ip:
        return config.host_ip
    return resolve_host_ip(config.lidar_ip, None, label="Airy")


class Airy(NativeModule, perception.Lidar, perception.IMU):
    config: AiryConfig

    lidar: Out[PointCloud2]
    imu: Out[Imu]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        self.config.host_ip = _resolved_host_ip(self.config)
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    Airy()
