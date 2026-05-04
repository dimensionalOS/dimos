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

Binds Livox SDK2 directly into FAST-LIO-NON-ROS for real-time LiDAR SLAM.
Outputs registered (world-frame) point clouds and odometry with covariance.

Usage::

    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        FastLio2.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import ipaddress
from pathlib import Path
import socket
import time
from typing import TYPE_CHECKING, Annotated

from pydantic.experimental.pipeline import validate_as
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
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
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import mapping, perception
from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger

_CONFIG_DIR = Path(__file__).parent / "config"
_logger = setup_logger()


def _odom_to_body_tf(msg: Odometry) -> Transform:
    return Transform(
        frame_id="odom",
        child_frame_id="body",
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


def _find_candidate_ips(lidar_ip: str, local_ips: list[str]) -> list[str]:
    """Suggest local IPs on the same subnet as the lidar."""
    candidates: list[str] = []
    try:
        lidar_net = ipaddress.IPv4Network(f"{lidar_ip}/24", strict=False)
        for ip in local_ips:
            if ipaddress.IPv4Address(ip) in lidar_net:
                candidates.append(ip)
    except (ValueError, TypeError):
        pass
    return candidates


class FastLio2Config(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/fastlio2_native"
    build_command: str | None = "nix build .#fastlio2_native"
    # Livox SDK hardware config
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0

    # Sensor mount pose — position + orientation of the sensor relative to ground.
    # Converted to init_pose CLI arg [x, y, z, qx, qy, qz, qw] in model_post_init.
    mount: Pose = Pose()

    # Frame IDs for output messages.  "odom" reflects that FastLio2 provides
    # locally-smooth, continuous odometry (no loop-closure jumps).  PGO
    # publishes the map→odom correction via TF.
    frame_id: str = "odom"
    child_frame_id: str = "body"

    # FAST-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # Point cloud filtering
    voxel_size: float = 0.1
    sor_mean_k: int = 50
    sor_stddev: float = 1.0

    # Global voxel map (disabled when map_freq <= 0)
    map_freq: float = 0.0
    map_voxel_size: float = 0.1
    map_max_range: float = 100.0

    # FAST-LIO YAML config (relative to config/ dir, or absolute path)
    # C++ binary reads YAML directly via yaml-cpp
    config: Annotated[
        Path, validate_as(...).transform(lambda p: p if p.is_absolute() else _CONFIG_DIR / p)
    ] = Path("mid360.yaml")

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

    # Resolved in __post_init__, passed as --config_path to the binary
    config_path: str | None = None

    # init_pose is computed from mount; config is resolved to config_path
    init_pose: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    cli_exclude: frozenset[str] = frozenset({"config", "mount"})

    def model_post_init(self, __context: object) -> None:
        """Resolve config_path and compute init_pose from mount."""
        super().model_post_init(__context)
        cfg = self.config
        if not cfg.is_absolute():
            cfg = _CONFIG_DIR / cfg
        self.config_path = str(cfg.resolve())
        m = self.mount
        self.init_pose = [
            m.x,
            m.y,
            m.z,
            m.orientation.x,
            m.orientation.y,
            m.orientation.z,
            m.orientation.w,
        ]


class FastLio2(NativeModule, perception.Lidar, perception.Odometry, mapping.GlobalPointcloud):
    config: FastLio2Config

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        self._validate_network()
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
        )

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(_odom_to_body_tf(msg))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _validate_network(self) -> None:
        host_ip = self.config.host_ip
        lidar_ip = self.config.lidar_ip
        local_ips = [ip for ip, _iface in get_local_ips()]

        _logger.info(
            "FastLio2 network check",
            host_ip=host_ip,
            lidar_ip=lidar_ip,
            local_ips=local_ips,
        )

        # Check if host_ip is actually assigned to this machine.
        if host_ip not in local_ips:
            same_subnet = _find_candidate_ips(lidar_ip, local_ips)

            if same_subnet:
                picked = same_subnet[0]
                _logger.warning(
                    f"FastLio2: host_ip={host_ip!r} not found locally. "
                    f"Auto-correcting to {picked!r} (same subnet as lidar {lidar_ip}).",
                    configured_ip=host_ip,
                    corrected_ip=picked,
                    lidar_ip=lidar_ip,
                    local_ips=local_ips,
                )
                self.config.host_ip = picked
                host_ip = picked
            else:
                subnet_prefix = ".".join(lidar_ip.split(".")[:3])
                msg = (
                    f"FastLio2: host_ip={host_ip!r} is not assigned to any local interface.\n"
                    f"  Lidar IP: {lidar_ip}\n"
                    f"  Local IPs found: {', '.join(local_ips) or '(none)'}\n"
                    f"  No local IP found on the same subnet as lidar ({lidar_ip}).\n"
                    f"  The lidar network interface may be down or unconfigured.\n"
                    f"  → Check: ip addr | grep {subnet_prefix}\n"
                    f"  → Or assign an IP: "
                    f"sudo ip addr add {subnet_prefix}.5/24 dev <iface>\n"
                )
                _logger.error(msg)
                raise RuntimeError(msg)

        # Check if we can bind a UDP socket on host_ip (port 0 = ephemeral).
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((host_ip, 0))
        except OSError as e:
            _logger.error(
                f"FastLio2: Cannot bind UDP socket on host_ip={host_ip!r}: {e}\n"
                f"  Another process may be using the Livox SDK ports.\n"
                f"  → Check: ss -ulnp | grep {host_ip}"
            )
            raise RuntimeError(
                f"FastLio2: Cannot bind UDP on {host_ip}: {e}. "
                f"Check if another Livox/FastLio2 process is running."
            ) from e

        _logger.info(
            "FastLio2 network check passed",
            host_ip=host_ip,
            lidar_ip=lidar_ip,
        )


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    FastLio2()
