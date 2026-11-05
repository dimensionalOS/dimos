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

"""Python NativeModule wrapper for the Rust Point-LIO binary.

Point-LIO consumes the Mid360 driver's raw cloud and IMU streams and publishes
its registered cloud, odometry and the odom -> sensor tf edge. Wire the driver
in with ``mid360_for_pointlio`` from ``pointlio_blueprints``::

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        mid360_for_pointlio(lidar_ip="192.168.1.155"),
        PointLio.blueprint(),
        SomeConsumer.blueprint(),
    )).loop()

Every config field is sent to the binary as stdin JSON.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception

# iVox local-map neighbor stencil. rust/src/module.rs maps the strings to
# Point-LIO's int codes.
IvoxNearbyType = Literal["center", "nearby6", "nearby18", "nearby26"]


class PointLioConfig(NativeModuleConfig):
    stdin_config: bool = True
    frame_id: str = "odom"
    # frame_id_prefix too: the module composes the namespaced frame itself,
    # since it publishes odometry and tf without going back through Python.
    base_fields: frozenset[str] = frozenset({"frame_id", "frame_id_prefix"})
    source_dir: str | None = "dimos/hardware/sensors/lidar/pointlio/rust"
    executable: str = "result/bin/pointlio_native"
    build_command: str | None = "nix build -L path:."

    # Odometry is published as frame_id (fixed) -> sensor_frame_id (moving sensor),
    # and also broadcast on TF. The point cloud is stamped with sensor_frame_id
    sensor_frame_id: str = "mid360_link"

    # Point-LIO internal processing rates (Hz)
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # Point-LIO tuning (read in rust/src/module.rs).
    # common
    con_frame: bool = False
    con_frame_num: int = 1
    cut_frame: bool = False
    cut_frame_time_interval: float = 0.1
    time_lag_imu_to_lidar: float = 0.0
    # preprocess
    scan_line: int = 4
    scan_rate: int = 10
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


class PointLio(NativeModule, perception.Lidar, perception.Odometry):
    """Point-LIO fed by the Mid360 driver's messages. Publishes tf itself."""

    config: PointLioConfig

    lidar_raw: In[PointCloud2]
    imu_raw: In[Imu]

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    PointLio()
