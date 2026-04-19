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

"""FastLio2 (LCM-input) NativeModule: FAST-LIO2 SLAM consuming external lidar+IMU.

Consumes PointCloud2 + Imu published by an upstream bridge (e.g.
DrddsLidarBridge on the M20) and produces world-frame registered scans +
odometry + optional global voxel map.

This differs from dimos.hardware.sensors.lidar.fastlio2.FastLio2, which
drives a Livox Mid-360 via the Livox SDK. Both wrap the same C++ binary
(`fastlio2_native`); the binary selects between modes via `--input_mode`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic.experimental.pipeline import validate_as

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# The C++ binary + YAML configs live under the hardware module (shared
# between the Livox-hardware wrapper and this LCM-input wrapper). Resolve
# absolute paths relative to this file so the wrapper works without
# needing a local symlink copy.
# __file__ is .../dimos/navigation/smart_nav/modules/fastlio2/fastlio2.py;
# parents[4] is .../dimos/.
_HARDWARE_CPP = (
    Path(__file__).resolve().parents[4] / "hardware" / "sensors" / "lidar" / "fastlio2" / "cpp"
)
_CONFIG_DIR = _HARDWARE_CPP.parent / "config"


class FastLio2Config(NativeModuleConfig):
    """Config for the LCM-input FAST-LIO2 native module."""

    # The binary + build tree live under the hardware module (shared between
    # both wrappers). Absolute paths so this Python wrapper runs correctly
    # regardless of where the smartnav blueprint resolves cwd.
    cwd: str | None = str(_HARDWARE_CPP)
    executable: str = str(_HARDWARE_CPP / "result" / "bin" / "fastlio2_native")
    build_command: str | None = None

    # Locks the C++ binary to LCM-input mode (subscribes to raw_points + imu
    # instead of opening Livox SDK sockets).
    input_mode: str = "lcm"

    # Trust sensor-provided timestamps on lidar + IMU directly, bypassing the
    # wall-clock-anchor + frame_ts-rewrite workarounds in the wrapper. Set
    # to True for the Airy integrated IMU path (PTP-locked with the lidar
    # clock); leave False for the yesense legacy path where the two streams
    # come from different clock domains and need the anchoring logic.
    native_clock: bool = False

    # Frame IDs for output messages
    frame_id: str = "map"
    child_frame_id: str = "body"

    # FAST-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # Point cloud filtering (applied to the world-frame registered cloud)
    voxel_size: float = 0.1
    sor_mean_k: int = 50
    sor_stddev: float = 1.0

    # Global voxel map (disabled when map_freq <= 0)
    map_freq: float = 0.0
    map_voxel_size: float = 0.1
    map_max_range: float = 100.0

    # FAST-LIO YAML config (relative to config/ dir, or absolute path).
    # C++ binary reads YAML directly via yaml-cpp.
    config: Annotated[
        Path, validate_as(...).transform(lambda p: p if p.is_absolute() else _CONFIG_DIR / p)
    ] = Path("velodyne.yaml")

    # Sensor mount pose — position + orientation of the sensor relative to
    # ground. Converted to init_pose CLI arg [x, y, z, qx, qy, qz, qw].
    mount: Pose = Pose()

    # Passed as --config_path to the binary (resolved from config in post-init)
    config_path: str | None = None

    # init_pose is computed from mount in post-init; config is resolved to config_path
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


class FastLio2(NativeModule[FastLio2Config]):
    """FAST-LIO2 SLAM module consuming external PointCloud2 + Imu streams.

    Wraps the `fastlio2_native` binary in LCM-input mode. Processes raw
    lidar + IMU, runs the FAST-LIO EKF-LOAM SLAM loop, and publishes
    world-frame registered scans + odometry.

    Ports:
        raw_points (In[PointCloud2]): Raw lidar point cloud (body frame).
        imu (In[Imu]): IMU data.
        registered_scan (Out[PointCloud2]): World-frame registered cloud.
        odometry (Out[Odometry]): SLAM-estimated odometry.
        global_map (Out[PointCloud2]): Global voxel map (optional, enable via map_freq > 0).
    """

    default_config: type[FastLio2Config] = FastLio2Config  # type: ignore[assignment]

    raw_points: In[PointCloud2]
    imu: In[Imu]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
