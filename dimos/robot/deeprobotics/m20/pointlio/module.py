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

"""Native Point-LIO wrapper with direct M20 ROS lidar and IMU ingress."""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception

IvoxNearbyType = Literal["center", "nearby6", "nearby18", "nearby26"]


class M20PointLioConfig(NativeModuleConfig):
    """M20 Point-LIO input contract and robot-specific estimator tuning."""

    cwd: str | None = "cpp"
    executable: str = "build/m20_pointlio"
    build_command: str | None = "./build.sh"
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(
        default_factory=lambda: {
            "LD_LIBRARY_PATH": "/opt/ros/foxy/lib",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        }
    )
    # GOS isolates its RK3588 big cores. Cores 6-7 run the vendor lidar
    # drivers, so Point-LIO owns the otherwise-idle big cores 4-5.
    cpu_affinity: frozenset[int] | None = frozenset({4, 5})

    lidar_topic: str = "/LIDAR/POINTS"
    imu_topic: str = "/IMU"
    node_name: str = "dimos_m20_pointlio"
    world_frame: str = "odom"
    base_frame: str = "base_link"
    processing_rate_hz: float = Field(default=1000.0, gt=0.0)
    pointcloud_rate_hz: float = Field(default=10.0, gt=0.0)
    odometry_rate_hz: float = Field(default=50.0, gt=0.0)
    readiness_rate_hz: float = Field(default=10.0, gt=0.0)
    lidar_timeout_s: float = Field(default=0.5, gt=0.0)
    imu_timeout_s: float = Field(default=0.5, gt=0.0)
    estimate_timeout_s: float = Field(default=0.5, gt=0.0)
    max_scan_duration_s: float = Field(default=0.2, gt=0.0)
    # Live merged M20 frames contain roughly 100k returns. Point-LIO cannot
    # process that rate in real time on the RK3588, so the native adapter
    # uniformly selects this many returns before sorting and preprocessing.
    max_cloud_points: int = Field(default=20_000, gt=0, le=100_000)

    msr_freq: float = Field(default=200.0, gt=0.0)
    main_freq: float = Field(default=1000.0, gt=0.0)
    con_frame: bool = False
    con_frame_num: int = Field(default=1, gt=0)
    cut_frame: bool = False
    cut_frame_time_interval: float = Field(default=0.1, gt=0.0)
    time_lag_imu_to_lidar: float = 0.0
    # The vendor merged cloud offsets the second lidar's rings: front uses
    # 0-95 and rear uses 96-191. Treating this as a single 96-line lidar drops
    # the complete rear scan in the native adapter before Point-LIO sees it.
    scan_line: int = Field(default=192, gt=0)
    scan_rate: int = Field(default=10, gt=0)
    blind: float = Field(default=0.5, ge=0.0)
    point_filter_num: int = Field(default=3, gt=0)

    use_imu_as_input: bool = False
    prop_at_freq_of_imu: bool = True
    check_satu: bool = True
    init_map_size: int = Field(default=10, gt=0)
    space_down_sample: bool = True
    satu_acc: float = Field(default=3.0, gt=0.0)
    satu_gyro: float = Field(default=35.0, gt=0.0)
    # Point-LIO expects acceleration in g. The native adapter converts the
    # M20's ROS-standard m/s^2 values before feeding the estimator.
    acc_norm: float = Field(default=1.0, gt=0.0)
    plane_thr: float = Field(default=0.1, gt=0.0)
    filter_size_surf: float = Field(default=0.2, gt=0.0)
    filter_size_map: float = Field(default=0.5, gt=0.0)
    ivox_grid_resolution: float = Field(default=2.0, gt=0.0)
    ivox_nearby_type: IvoxNearbyType = "nearby6"
    cube_side_length: float = Field(default=1000.0, gt=0.0)
    det_range: float = Field(default=60.0, gt=0.0)
    fov_degree: float = Field(default=360.0, gt=0.0, le=360.0)
    imu_en: bool = True
    start_in_aggressive_motion: bool = False
    extrinsic_est_en: bool = False
    imu_time_inte: float = Field(default=0.005, gt=0.0)
    lidar_meas_cov: float = Field(default=0.01, gt=0.0)
    acc_cov_input: float = Field(default=0.1, gt=0.0)
    vel_cov: float = Field(default=20.0, gt=0.0)
    gyr_cov_input: float = Field(default=0.01, gt=0.0)
    gyr_cov_output: float = Field(default=1000.0, gt=0.0)
    acc_cov_output: float = Field(default=500.0, gt=0.0)
    b_gyr_cov: float = Field(default=0.0001, gt=0.0)
    b_acc_cov: float = Field(default=0.0001, gt=0.0)
    imu_meas_acc_cov: float = Field(default=0.01, gt=0.0)
    imu_meas_omg_cov: float = Field(default=0.01, gt=0.0)
    match_s: float = Field(default=81.0, gt=0.0)
    gravity_align: bool = True
    gravity: list[float] = Field(default_factory=lambda: [0.0, 0.0, -9.81])
    gravity_init: list[float] = Field(default_factory=lambda: [0.0, 0.0, -9.81])
    # Both public M20 streams are already expressed in base_link. Identity is
    # therefore intentional: Point-LIO owns odom -> base_link with no hidden
    # sensor/body transform.
    extrinsic_t: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    extrinsic_r: list[float] = Field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    publish_odometry_without_downsample: bool = False
    odom_only: bool = False
    debug: bool = False


class M20PointLio(NativeModule, perception.Lidar, perception.Odometry):
    """Run the pinned Point-LIO core directly on the M20's ROS sensor topics.

    The native process subscribes to the merged ``base_link`` cloud and 200 Hz
    ``base_link`` IMU itself, avoiding a full-payload LCM hop through the command
    bridge. It has no vendor odometry input and owns ``odom -> base_link``.
    """

    config: M20PointLioConfig

    lidar_ready: Out[Bool]
    localization_ready: Out[Bool]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    odometry: Out[Odometry]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    M20PointLio()
