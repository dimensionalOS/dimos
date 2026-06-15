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

"""Python NativeModule wrapper for the Point-LIO + Livox Mid-360 binary.

Usage::

    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        PointLio.blueprint(host_ip="192.168.1.5", lidar_ip="192.168.1.155"),
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import ipaddress
import os
from pathlib import Path
import socket
from typing import TYPE_CHECKING, Annotated

from pydantic import Field
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
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_ODOM
from dimos.spec import perception
from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger

_CONFIG_DIR = Path(__file__).parent / "config"
_logger = setup_logger()


class PointLioConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/pointlio_native"
    build_command: str | None = "nix build .#pointlio_native"
    host_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_POINTLIO_HOST_IP"))
    lidar_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_POINTLIO_LIDAR_IP"))
    frequency: float = 10.0

    frame_id: str = "mid360_link"
    body_start_frame_id: str = FRAME_ODOM
    body_frame_id: str = "base_link"

    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # Point cloud filtering
    voxel_size: float = 0.1
    sor_mean_k: int = 50
    sor_stddev: float = 1.0

    # Point-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # relative to config/ dir, or absolute path
    config: Annotated[
        Path,
        validate_as(...).transform(lambda path: path if path.is_absolute() else _CONFIG_DIR / path),
    ] = Path("default.yaml")

    debug: bool = False

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

    cli_exclude: frozenset[str] = frozenset({"config", "body_start_frame_id"})

    def model_post_init(self, __context: object) -> None:
        super().model_post_init(__context)
        cfg = self.config
        if not cfg.is_absolute():
            cfg = _CONFIG_DIR / cfg
        self.config_path = str(cfg.resolve())


class PointLio(NativeModule, perception.Lidar, perception.Odometry):
    config: PointLioConfig

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        self._validate_network()
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
        )

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.config.body_start_frame_id,
                child_frame_id=self.config.body_frame_id,
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
                # Match the odometry message's own timestamp exactly so the TF
                # and the pose can't drift apart. No `or time.time()` fallback: a
                # real ts of 0.0 must not be silently replaced with wall time.
                ts=msg.ts,
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _validate_network(self) -> None:
        lidar_ip = self.config.lidar_ip
        if not lidar_ip:
            raise RuntimeError(
                "PointLio: lidar_ip not set — it's network-specific. Set it in the config "
                "or via the DIMOS_POINTLIO_LIDAR_IP env var."
            )
        local_ips = [ip for ip, _iface in get_local_ips()]

        # host_ip is optional — it's just the local NIC facing the lidar. When
        # it's unset (or not one of our IPs) derive it as the local IP on the
        # lidar's /24.
        configured = self.config.host_ip
        if configured and configured in local_ips:
            host_ip = configured
        else:
            try:
                lidar_net = ipaddress.IPv4Network(f"{lidar_ip}/24", strict=False)
                same_subnet = [ip for ip in local_ips if ipaddress.IPv4Address(ip) in lidar_net]
            except (ValueError, TypeError):
                same_subnet = []
            if not same_subnet:
                subnet_prefix = ".".join(lidar_ip.split(".")[:3])
                msg = (
                    f"PointLio: cannot resolve host_ip — no local IP on the lidar's subnet "
                    f"(lidar {lidar_ip}).\n"
                    f"  Local IPs found: {', '.join(local_ips) or '(none)'}\n"
                    f"  → Bring up the lidar NIC, or set host_ip explicitly.\n"
                    f"  → Check: ip addr | grep {subnet_prefix}\n"
                    f"  → Or assign: sudo ip addr add {subnet_prefix}.5/24 dev <iface>\n"
                )
                _logger.error(msg)
                raise RuntimeError(msg)
            host_ip = same_subnet[0]
            self.config.host_ip = host_ip
            if configured:
                _logger.warning(
                    f"PointLio: host_ip={configured!r} not local; using {host_ip!r} "
                    f"(on lidar {lidar_ip}'s subnet).",
                )

        _logger.info(
            "PointLio network check", host_ip=host_ip, lidar_ip=lidar_ip, local_ips=local_ips
        )

        # Check if we can bind a UDP socket on host_ip (port 0 = ephemeral).
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((host_ip, 0))
        except OSError as err:
            _logger.error(
                f"PointLio: Cannot bind UDP socket on host_ip={host_ip!r}: {err}\n"
                f"  Another process may be using the Livox SDK ports.\n"
                f"  → Check: ss -ulnp | grep {host_ip}"
            )
            raise RuntimeError(
                f"PointLio: Cannot bind UDP on {host_ip}: {err}. "
                f"Check if another Livox/PointLio process is running."
            ) from err

        _logger.info(
            "PointLio network check passed",
            host_ip=host_ip,
            lidar_ip=lidar_ip,
        )


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    PointLio()
