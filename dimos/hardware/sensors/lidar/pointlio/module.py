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

Point-LIO tuning lives directly on ``PointLioConfig`` and is passed to the C++
binary as plain CLI args (no YAML).
"""

from __future__ import annotations

import os
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
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_ODOM
from dimos.spec import perception

# Human-readable enums; the C++ binary (main.cpp) maps these strings to
# Point-LIO's int codes.
# LID_TYPE enum (Point-LIO src/preprocess.h). avia = 1 selects the Livox branch
# the Mid-360 emits.
LidarType = Literal["avia", "velodyne", "ouster", "hesai", "unilidar"]
TimestampUnit = Literal["second", "millisecond", "microsecond", "nanosecond"]
# iVox local-map neighbour stencil.
IvoxNearbyType = Literal["center", "nearby6", "nearby18", "nearby26"]


class PointLioConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/pointlio_native"
    build_command: str | None = "nix build .#pointlio_native"
    # lidar_ip required; host_ip optional (auto-derived from lidar_ip's subnet).
    # Both fall back to DIMOS_POINTLIO_LIDAR_IP / DIMOS_POINTLIO_HOST_IP.
    host_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_POINTLIO_HOST_IP"))
    lidar_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_POINTLIO_LIDAR_IP"))
    frequency: float = 10.0

    # Odometry is published as frame_id (fixed) -> child_frame_id (moving body),
    # and also broadcast on TF. The point cloud is stamped with sensor_frame_id
    # (the lidar's own frame — get_body_cloud is the undistorted scan, not yet
    # transformed into the body frame).
    frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BODY
    sensor_frame_id: str = "mid360_link"

    # Point-LIO internal processing rates (Hz)
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    debug: bool = False

    # Point-LIO tuning, passed to the binary as plain CLI args (read in main.cpp).
    # common
    con_frame: bool = False
    con_frame_num: int = 1
    cut_frame: bool = False
    cut_frame_time_interval: float = 0.1
    time_lag_imu_to_lidar: float = 0.0
    # preprocess
    lidar_type: LidarType = "avia"  # 1 = AVIA (Livox) branch the Mid-360 emits
    scan_line: int = 4
    scan_rate: int = 10
    timestamp_unit: TimestampUnit = "nanosecond"
    blind: float = 0.5  # spherical min range (m)
    point_filter_num: int = 3  # pre-KF decimation: keep every Nth raw point (1 = all)
    # mapping
    use_imu_as_input: bool = False  # false = IMU-as-output model (robust path)
    prop_at_freq_of_imu: bool = True
    check_satu: bool = True
    init_map_size: int = 10
    space_down_sample: bool = True  # pre-KF voxel downsample (leaf = filter_size_surf)
    satu_acc: float = 3.0  # g; accel >= this is treated as saturated, bounding velocity
    satu_gyro: float = 35.0
    acc_norm: float = 1.0  # IMU accel unit: g
    plane_thr: float = 0.1
    filter_size_surf: float = 0.2  # pre-KF scan downsample leaf (m), iff space_down_sample
    filter_size_map: float = 0.5
    ivox_grid_resolution: float = 2.0  # iVox local-map grid (m)
    ivox_nearby_type: IvoxNearbyType = "nearby6"
    cube_side_length: float = 1000.0
    det_range: float = 100.0
    fov_degree: float = 360.0
    imu_en: bool = True
    start_in_aggressive_motion: bool = False
    extrinsic_est_en: bool = False
    imu_time_inte: float = 0.005
    lidar_meas_cov: float = 0.01
    acc_cov_input: float = 0.1
    vel_cov: float = 20.0
    gyr_cov_input: float = 0.01
    gyr_cov_output: float = 1000.0
    acc_cov_output: float = 500.0
    b_gyr_cov: float = 0.0001
    b_acc_cov: float = 0.0001
    imu_meas_acc_cov: float = 0.01
    imu_meas_omg_cov: float = 0.01
    match_s: float = 81.0
    gravity_align: bool = True
    gravity: list[float] = Field(default_factory=lambda: [0.0, 0.0, -9.81])
    gravity_init: list[float] = Field(default_factory=lambda: [0.0, 0.0, -9.81])
    extrinsic_t: list[float] = Field(default_factory=lambda: [-0.011, -0.02329, 0.04412])
    extrinsic_r: list[float] = Field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    # odometry
    publish_odometry_without_downsample: bool = False
    odom_only: bool = False

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
