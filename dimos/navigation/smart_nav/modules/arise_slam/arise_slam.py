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

"""AriseSlam NativeModule: tightly-coupled LiDAR-inertial SLAM.

Reimplements https://github.com/YWL0720/ARISE-SLAM as a decoupled DimOS
NativeModule.  Consumes raw lidar scans + IMU and produces corrected
odometry and registered point clouds.

Key features:
  - Tightly-coupled LiDAR-inertial odometry with IMU preintegration
  - Multi-sensor support (Livox, Velodyne, Ouster)
  - Adaptive voxel sizing for mapping
  - Shift-based scan undistortion
  - Configurable blind-zone filtering for solid-state lidars
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class AriseSlamConfig(NativeModuleConfig):
    """Config for arise_slam native module.

    Defaults mirror the upstream livox_mid360.yaml config verbatim.
    Scene-specific tuning should be passed via the ``arise_slam``
    kwarg dict on ``smart_nav(...)``.
    """

    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/arise_slam"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    # ── Feature extraction ─────────────────────────────────────────
    scan_line: int = 4
    """Number of scan lines (4=Livox, 16/32/64=Velodyne)."""
    sensor: Literal["livox", "velodyne", "ouster"] = "livox"
    """Sensor type."""
    min_range: float = 0.1
    """Minimum valid point range (metres)."""
    max_range: float = 130.0
    """Maximum valid point range (metres)."""
    provide_point_time: int = 1
    """Whether point cloud includes per-point timestamps (1=yes, 0=no)."""
    skip_frame: int = 1
    """Process every Nth frame for mapping."""
    use_imu_roll_pitch: bool = False
    """Use IMU roll/pitch for initial orientation estimate."""
    lidar_flip: bool = False
    """Flip lidar orientation (180 degree rotation)."""
    point_filter_num: int = 1
    """Point filter decimation (keep every Nth point)."""
    use_dynamic_mask: bool = False
    """Enable dynamic object masking."""
    use_up_realsense_points: bool = False
    """Use upper RealSense depth camera points."""
    use_down_realsense_points: bool = False
    """Use lower RealSense depth camera points."""
    skip_realsense_points: int = 3
    """Decimation for RealSense depth points."""
    debug_view_enabled: bool = False
    """Enable debug visualization."""
    livox_pitch: float = 0.0
    """Livox sensor pitch offset (degrees)."""

    # ── Laser mapping ──────────────────────────────────────────────
    mapping_period: float = 0.1
    """Mapping period (seconds)."""
    mapping_line_resolution: float = 0.1
    """Voxel resolution for line features (metres)."""
    mapping_plane_resolution: float = 0.2
    """Voxel resolution for plane features (metres)."""
    max_iterations: int = 5
    """Maximum optimization iterations per scan."""
    max_surface_features: int = 2000
    """Maximum number of surface features to use."""
    velocity_failure_threshold: float = 30.0
    """Velocity threshold to detect tracking failure (m/s)."""
    auto_voxel_size: bool = True
    """Enable adaptive voxel sizing."""
    forget_far_chunks: bool = False
    """Forget far map chunks to save memory."""
    visual_confidence_factor: float = 1.0
    """Visual odometry confidence weighting factor."""
    pos_degeneracy_threshold: float = 1.0
    """Position degeneracy detection threshold."""
    ori_degeneracy_threshold: float = 1.0
    """Orientation degeneracy detection threshold."""
    shift_avg_ratio: float = 0.2
    """Shift averaging ratio for undistortion."""
    shift_undistortion: bool = True
    """Enable shift-based scan undistortion."""
    publish_only_feature_points: bool = False
    """Only publish feature points (not full cloud)."""
    yaw_ratio: float = 0.0
    """Yaw rate scaling factor (upstream calibration default: 0.0)."""
    relocalization_map_path: str = ""
    """Path to prior map for relocalization (empty=disabled)."""
    local_mode: bool = False
    """Enable localization mode (use prior map)."""
    init_x: float = 0.0
    """Initial X position for localization mode."""
    init_y: float = 0.0
    """Initial Y position for localization mode."""
    init_z: float = 0.0
    """Initial Z position for localization mode."""
    init_roll: float = 0.0
    """Initial roll for localization mode."""
    init_pitch: float = 0.0
    """Initial pitch for localization mode."""
    init_yaw: float = 0.0
    """Initial yaw for localization mode."""
    read_pose_file: bool = False
    """Read initial pose from file."""
    trust_fallback_odom: bool = False
    """Override SLAM position with fallback odometry (for sim testing)."""

    # ── IMU preintegration ─────────────────────────────────────────
    use_imu_preintegration: bool = True
    """Enable IMU preintegration (GTSAM iSAM2 factor graph)."""
    acc_n: float = 0.3994
    """Accelerometer noise density."""
    gyr_n: float = 0.001564
    """Gyroscope noise density."""
    acc_w: float = 0.006436
    """Accelerometer random walk."""
    gyr_w: float = 0.0000356
    """Gyroscope random walk."""
    g_norm: float = 9.80511
    """Local gravity magnitude (m/s^2)."""
    lidar_correction_noise: float = 0.01
    """Noise added to lidar pose corrections."""
    smooth_factor: float = 0.9
    """IMU preintegration smoothing factor."""

    # ── IMU bias offsets ───────────────────────────────────────────
    # Upstream YAML has two sets: feature_extraction (x=0.04,y=0,z=0,limits=1.0)
    # and imu_preintegration (x=0.04,y=0,z=0,limits=0.3/0.2/0.1).
    imu_acc_x_offset: float = 0.04
    """Accelerometer X-axis bias offset."""
    imu_acc_y_offset: float = 0.0
    """Accelerometer Y-axis bias offset."""
    imu_acc_z_offset: float = 0.0
    """Accelerometer Z-axis bias offset."""
    imu_acc_x_limit: float = 1.0
    """Accelerometer X-axis bias limit (feature extraction)."""
    imu_acc_y_limit: float = 1.0
    """Accelerometer Y-axis bias limit (feature extraction)."""
    imu_acc_z_limit: float = 1.0
    """Accelerometer Z-axis bias limit (feature extraction)."""

    # Upstream YAML: imu_preintegration_node has tighter limits than feature_extraction
    imu_preint_acc_x_limit: float = 0.3
    """Accelerometer X-axis bias limit (IMU preintegration)."""
    imu_preint_acc_y_limit: float = 0.2
    """Accelerometer Y-axis bias limit (IMU preintegration)."""
    imu_preint_acc_z_limit: float = 0.1
    """Accelerometer Z-axis bias limit (IMU preintegration)."""

    # ── Blind zone (for Livox) ─────────────────────────────────────
    blind_front: float = 0.2
    """Front blind zone distance (metres)."""
    blind_back: float = -0.2
    """Back blind zone bound (metres, negative = behind sensor)."""
    blind_left: float = 0.3
    """Left blind zone distance (metres)."""
    blind_right: float = -0.3
    """Right blind zone bound (metres, negative = right of sensor)."""
    blind_disk_radius: float = 0.5
    """Central disk blind zone radius (metres)."""
    blind_disk_low: float = -0.05
    """Blind disk lower Z bound (metres)."""
    blind_disk_high: float = 0.05
    """Blind disk upper Z bound (metres)."""

    # ── Diagnostic output toggles ─────────────────────────────────
    # Topics are always present as ports; these control whether data
    # is actually published (to save CPU/bandwidth when not debugging).
    publish_surround_map: bool = False
    """Publish local surround map (every 5 frames)."""
    publish_global_map: bool = False
    """Publish accumulated global map (every 20 frames)."""
    publish_laser_odometry: bool = False
    """Publish raw laser-only odometry."""
    publish_incremental_odometry: bool = False
    """Publish incremental odometry between frames."""
    publish_slam_stats: bool = False
    """Publish optimization statistics (feature counts, degeneracy, uncertainty)."""


class AriseSlam(NativeModule):
    """Tightly-coupled LiDAR-inertial SLAM.

    Consumes raw lidar scans and IMU measurements from any upstream sensor
    module and produces corrected odometry and world-frame registered
    point clouds.

    Ports:
        lidar (In[PointCloud2]): Raw lidar scan.
        imu (In[Imu]): IMU measurements.
        fallback_odometry (In[Odometry]): Optional fallback for degeneracy recovery.
        odometry (Out[Odometry]): Corrected pose (IMU-fused or laser-only).
        registered_scan (Out[PointCloud2]): World-frame registered cloud.
        surround_map (Out[PointCloud2]): Local map around robot (diagnostic).
        global_map (Out[PointCloud2]): Full accumulated map (diagnostic).
        laser_odometry (Out[Odometry]): Raw laser-only odometry (diagnostic).
        incremental_odometry (Out[Odometry]): Incremental frame-to-frame odometry (diagnostic).
        slam_stats (Out[Odometry]): Optimization stats packed in covariance (diagnostic).
    """

    config: AriseSlamConfig

    lidar: In[PointCloud2]
    imu: In[Imu]
    fallback_odometry: In[Odometry]
    odometry: Out[Odometry]
    registered_scan: Out[PointCloud2]
    # Diagnostic outputs — always present as ports, publishing controlled by config
    surround_map: Out[PointCloud2]
    global_map: Out[PointCloud2]
    laser_odometry: Out[Odometry]
    incremental_odometry: Out[Odometry]
    slam_stats: Out[Odometry]
