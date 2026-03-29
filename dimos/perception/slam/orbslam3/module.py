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

"""Python NativeModule wrapper for ORB-SLAM3 visual SLAM.

Wraps ORB-SLAM3 as a native subprocess that receives camera images and
outputs camera pose estimates (odometry).

Usage::

    from dimos.perception.slam.orbslam3.module import OrbSlam3
    from dimos.core.blueprints import autoconnect

    autoconnect(
        OrbSlam3.blueprint(sensor_mode=SensorMode.MONOCULAR),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

import enum
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception

_MODULE_DIR = Path(__file__).parent


class SensorMode(enum.StrEnum):
    MONOCULAR = "MONOCULAR"
    STEREO = "STEREO"
    RGBD = "RGBD"
    IMU_MONOCULAR = "IMU_MONOCULAR"
    IMU_STEREO = "IMU_STEREO"
    IMU_RGBD = "IMU_RGBD"


class OrbSlam3Config(NativeModuleConfig):
    """Config for the ORB-SLAM3 visual SLAM native module."""

    cwd: str | None = str(_MODULE_DIR)
    executable: str = "result/bin/orbslam3_native"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-orb-slam3/v0.2.0 --no-write-lock-file"
    )

    # ORB-SLAM3 sensor mode
    sensor_mode: SensorMode = SensorMode.MONOCULAR

    # Pangolin viewer (disable for headless)
    use_viewer: bool = False

    # Frame IDs for output messages
    frame_id: str = "world"
    child_frame_id: str = "camera_link"

    # Camera settings YAML (absolute path, or override with your own calibration)
    settings_path: str = str(
        _MODULE_DIR / "result" / "share" / "orbslam3" / "config" / "RealSense_D435i.yaml"
    )

    # Vocabulary path (None = use compiled-in default from nix build)
    vocab_path: str | None = None

    # How often to publish map points and keyframe path (every N frames)
    map_publish_interval: int = 10


class OrbSlam3(NativeModule[OrbSlam3Config], perception.Odometry):
    """ORB-SLAM3 visual SLAM module.

    Ports:
        color_image (In[Image]): Camera frames to track.
        odometry (Out[Odometry]): Camera pose as nav_msgs.Odometry.
        keypoints_image (Out[Image]): Color image with tracked keypoints overlay.
        map_points (Out[PointCloud2]): Sparse 3D map points.
        keyframe_path (Out[NavPath]): Keyframe pose graph as a path.
    """

    default_config = OrbSlam3Config
    color_image: In[Image]
    odometry: Out[Odometry]
    keypoints_image: Out[Image]
    map_points: Out[PointCloud2]
    keyframe_path: Out[NavPath]


orbslam3_module = OrbSlam3.blueprint

__all__ = [
    "OrbSlam3",
    "OrbSlam3Config",
    "SensorMode",
    "orbslam3_module",
]

# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    OrbSlam3()
