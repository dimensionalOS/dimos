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

"""AriseSLAM NativeModule: C++ LiDAR SLAM with feature-based scan matching.

Ported from arise_slam_mid360. Performs curvature-based feature extraction
(edge + planar), scan-to-map matching via Ceres optimization, and optional
IMU preintegration for motion prediction. Publishes world-frame registered
point clouds and odometry.
"""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class AriseSLAMConfig(NativeModuleConfig):
    """Config for the AriseSLAM native module."""

    cwd: str | None = "."
    executable: str = "result/bin/arise_slam"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-arise-slam/v0.1.0 --no-write-lock-file"
    )

    # C++ binary uses camelCase CLI args.
    cli_name_override: dict[str, str] = {
        "edge_threshold": "edgeThreshold",
        "surf_threshold": "surfThreshold",
        "scan_voxel_size": "scanVoxelSize",
        "line_resolution": "lineRes",
        "plane_resolution": "planeRes",
        "max_range": "maxRange",
        "max_icp_iterations": "maxICPIterations",
        "max_lm_iterations": "maxLMIterations",
        "use_imu": "useIMU",
        "min_publish_interval": "minPublishInterval",
        "publish_map": "publishMap",
        "map_publish_rate": "mapPublishRate",
    }

    # Feature extraction
    edge_threshold: float = 1.0
    surf_threshold: float = 0.1
    scan_voxel_size: float = 0.1

    # Local map
    line_resolution: float = 0.2
    plane_resolution: float = 0.4
    max_range: float = 100.0

    # Scan matching
    max_icp_iterations: int = 4
    max_lm_iterations: int = 15

    # IMU
    use_imu: bool = True
    gravity: float = 9.80511

    # Output
    min_publish_interval: float = 0.05
    publish_map: bool = False
    map_publish_rate: float = 0.2

    # Sensor mount pose — position + orientation of the sensor relative to ground.
    # Converted to init_x/y/z/roll/pitch/yaw CLI args in model_post_init.
    mount: Pose = Pose()

    # init_* fields are computed from mount; mount itself is not a CLI arg
    init_x: float = 0.0
    init_y: float = 0.0
    init_z: float = 0.0
    init_roll: float = 0.0
    init_pitch: float = 0.0
    init_yaw: float = 0.0

    cli_exclude: frozenset[str] = frozenset({"mount"})

    def model_post_init(self, __context: object) -> None:
        """Compute init_x/y/z/roll/pitch/yaw from mount."""
        super().model_post_init(__context)
        self.init_x = self.mount.x
        self.init_y = self.mount.y
        self.init_z = self.mount.z
        self.init_roll = self.mount.roll
        self.init_pitch = self.mount.pitch
        self.init_yaw = self.mount.yaw


class AriseSLAM(NativeModule):
    """LiDAR SLAM module with feature-based scan-to-map matching.

    Processes raw LiDAR point clouds through curvature-based feature
    extraction, matches against a rolling local map using Ceres
    optimization, and publishes world-frame registered scans + odometry.

    Ports:
        raw_points (In[PointCloud2]): Raw lidar point cloud (body frame).
        imu (In[Imu]): IMU data for motion prediction.
        registered_scan (Out[PointCloud2]): World-frame registered cloud.
        odometry (Out[Odometry]): SLAM-estimated odometry.
        local_map (Out[PointCloud2]): Local map visualization (optional).
    """

    default_config: type[AriseSLAMConfig] = AriseSLAMConfig  # type: ignore[assignment]

    raw_points: In[PointCloud2]
    imu: In[Imu]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    local_map: Out[PointCloud2]
