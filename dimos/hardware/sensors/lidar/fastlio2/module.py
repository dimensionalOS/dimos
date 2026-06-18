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

FAST-LIO tuning lives directly on ``FastLio2Config`` (no YAML files). On
``start()`` the fields are rendered to a throwaway YAML that the C++ binary
reads via ``--config_path``.
"""

from __future__ import annotations

import os
from pathlib import Path
import tempfile
import time
from typing import TYPE_CHECKING, Literal

from pydantic import Field
from reactivex.disposable import Disposable
import yaml

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
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_ODOM
from dimos.spec import perception

# FAST-LIO encodes these as ints/codes; expose human-readable names and translate.
LidarType = Literal["livox", "velodyne", "ouster"]
_LIDAR_TYPE_CODE = {"livox": 1, "velodyne": 2, "ouster": 3}

TimestampUnit = Literal["second", "millisecond", "microsecond", "nanosecond"]
_TIMESTAMP_UNIT_CODE = {"second": 0, "millisecond": 1, "microsecond": 2, "nanosecond": 3}

# Field name -> FAST-LIO YAML (section, key). Only these fields are rendered into
# the generated config; everything else on FastLio2Config is module plumbing.
_YAML_LAYOUT: dict[str, tuple[str, str]] = {
    "lid_topic": ("common", "lid_topic"),
    "imu_topic": ("common", "imu_topic"),
    "time_sync_en": ("common", "time_sync_en"),
    "time_offset_lidar_to_imu": ("common", "time_offset_lidar_to_imu"),
    "lidar_type": ("preprocess", "lidar_type"),
    "scan_line": ("preprocess", "scan_line"),
    "scan_rate": ("preprocess", "scan_rate"),
    "timestamp_unit": ("preprocess", "timestamp_unit"),
    "blind": ("preprocess", "blind"),
    "acc_cov": ("mapping", "acc_cov"),
    "gyr_cov": ("mapping", "gyr_cov"),
    "b_acc_cov": ("mapping", "b_acc_cov"),
    "b_gyr_cov": ("mapping", "b_gyr_cov"),
    "filter_size_surf": ("mapping", "filter_size_surf"),
    "filter_size_map": ("mapping", "filter_size_map"),
    "fov_degree": ("mapping", "fov_degree"),
    "det_range": ("mapping", "det_range"),
    "extrinsic_est_en": ("mapping", "extrinsic_est_en"),
    "extrinsic_t": ("mapping", "extrinsic_T"),
    "extrinsic_r": ("mapping", "extrinsic_R"),
    "path_en": ("publish", "path_en"),
    "scan_publish_en": ("publish", "scan_publish_en"),
    "dense_publish_en": ("publish", "dense_publish_en"),
    "scan_bodyframe_pub_en": ("publish", "scan_bodyframe_pub_en"),
    "pcd_save_en": ("pcd_save", "pcd_save_en"),
    "interval": ("pcd_save", "interval"),
}


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

    # "odom" frame: FastLio2 gives smooth continuous odometry; PGO publishes the
    # map→odom correction via TF.
    frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BODY

    # FAST-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    debug: bool = False

    # --- FAST-LIO tuning (rendered to the generated YAML; see _YAML_LAYOUT) ---
    # common
    lid_topic: str = "/livox/lidar"
    imu_topic: str = "/livox/imu"
    time_sync_en: bool = False
    time_offset_lidar_to_imu: float = 0.0
    # preprocess
    lidar_type: LidarType = "livox"
    scan_line: int = 4
    scan_rate: int = 10  # velodyne only
    timestamp_unit: TimestampUnit = "microsecond"  # velodyne/ouster time field unit
    blind: float = 0.5  # spherical min range (m)
    # mapping
    # acc_cov down-weights the IMU accel prediction; upstream 0.1 lets Go2 odom
    # diverge to km/s, 1.0 holds bounded. See jhist dimos-fastlio-velocity-spike.md.
    acc_cov: float = 1.0
    gyr_cov: float = 0.1
    b_acc_cov: float = 0.0001
    b_gyr_cov: float = 0.0001
    filter_size_surf: float = 0.1  # IESKF scan voxel; does not affect divergence
    filter_size_map: float = 0.1
    fov_degree: int = 360  # FAST-LIO reads this as an int
    det_range: float = 100.0
    extrinsic_est_en: bool = False  # online IMU-LiDAR extrinsic estimation
    extrinsic_t: list[float] = Field(default_factory=lambda: [-0.011, -0.02329, 0.04412])
    extrinsic_r: list[float] = Field(default_factory=lambda: [1, 0, 0, 0, 1, 0, 0, 0, 1])
    # publish
    path_en: bool = False
    scan_publish_en: bool = True
    dense_publish_en: bool = True
    scan_bodyframe_pub_en: bool = True
    # pcd_save
    pcd_save_en: bool = True
    interval: int = -1

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

    # Set in start() to the generated YAML; passed as --config_path to the binary.
    config_path: str | None = None

    # FAST-LIO tuning fields feed the generated YAML, not CLI args.
    cli_exclude: frozenset[str] = frozenset(_YAML_LAYOUT)

    def render_config_yaml(self) -> str:
        """Render the FAST-LIO tuning fields to YAML text the C++ binary reads."""
        doc: dict[str, dict[str, object]] = {}
        for field, (section, key) in _YAML_LAYOUT.items():
            val: object = getattr(self, field)
            if field == "lidar_type":
                val = _LIDAR_TYPE_CODE[val]  # type: ignore[index]
            elif field == "timestamp_unit":
                val = _TIMESTAMP_UNIT_CODE[val]  # type: ignore[index]
            doc.setdefault(section, {})[key] = val
        return yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)


class FastLio2(NativeModule, perception.Lidar, perception.Odometry):
    config: FastLio2Config

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    _config_file: str | None = None

    @rpc
    def start(self) -> None:
        self._validate_network()
        self._write_config()
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
        )

    def _write_config(self) -> None:
        """Render the config fields to a temp YAML and point the binary at it."""
        fd, path = tempfile.mkstemp(prefix="fastlio2_", suffix=".yaml")
        with os.fdopen(fd, "w") as f:
            f.write(self.config.render_config_yaml())
        self._config_file = path
        self.config.config_path = path

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.frame_id,
                child_frame_id=self.config.child_frame_id,
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
        if self._config_file is not None:
            Path(self._config_file).unlink(missing_ok=True)
            self._config_file = None

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
