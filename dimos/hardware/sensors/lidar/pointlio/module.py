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

import os
from pathlib import Path
from typing import TYPE_CHECKING, Annotated

from pydantic import Field
from pydantic.experimental.pipeline import validate_as
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

_CONFIG_DIR = Path(__file__).parent / "config"


class PointLioConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/pointlio_native"
    build_command: str | None = "nix build .#pointlio_native"
    # lidar_ip required; host_ip optional (auto-derived from lidar_ip's subnet).
    # Both fall back to DIMOS_POINTLIO_LIDAR_IP / DIMOS_POINTLIO_HOST_IP.
    host_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_POINTLIO_HOST_IP"))
    lidar_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_POINTLIO_LIDAR_IP"))
    frequency: float = 10.0

    # Sensor frame for the cloud + odometry headers.
    frame_id: str = "mid360_link"
    # Published TF: body_start_frame_id -> body_frame_id.
    body_start_frame_id: str = FRAME_ODOM
    body_frame_id: str = "base_link"

    # Point-LIO internal processing rates (Hz)
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # Point-LIO YAML config (relative to config/ dir, or absolute path).
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
                # Match the odometry ts exactly; no `or time.time()` fallback (a
                # real ts of 0.0 must not become wall time).
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
        # host_ip optional: derive the local NIC on lidar_ip's /24 when unset or
        # not one of our IPs (shared with the Mid360 driver).
        self.config.host_ip = resolve_host_ip(lidar_ip, self.config.host_ip, label="PointLio")


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    PointLio()
