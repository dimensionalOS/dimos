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

"""Twist-driven Habitat robot connection: RGB-D, pose and a world-frame scan."""

from __future__ import annotations

from pydantic import Field

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class HabitatConnectionConfig(NativeModuleConfig):
    """Scene and camera settings for the Habitat native process."""

    # habitat-sim is python 3.9 conda-only, so it runs in its own env under
    # target/habitat (outside the package tree); the wrapper is the build sentinel.
    cwd: str | None = "nix"
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "habitat" / "habitat-native")
    build_command: str | None = "nix develop path:. -c ./install.sh"
    stdin_config: bool = True
    log_format: LogFormat = LogFormat.TEXT

    # Annotated HM3D house, no Matterport credentials needed.
    scene_dataset_config: str = str(
        DIMOS_PROJECT_ROOT
        / "target"
        / "habitat"
        / "data"
        / "versioned_data"
        / "hm3d-0.2"
        / "hm3d"
        / "example"
        / "hm3d_annotated_example_basis.scene_dataset_config.json"
    )
    scene_id: str = "00861-GLAQ4DNUx5U"

    # Defaults are Go2-ish, not GOAT-Bench's 1.41 m Stretch.
    width: int = 640
    height: int = 360
    hfov_deg: float = Field(default=90.0, gt=0.0, lt=180.0)
    camera_height_m: float = Field(default=0.45, gt=0.0)
    max_depth_m: float = Field(default=5.0, gt=0.0)

    sim_rate_hz: float = Field(default=10.0, gt=0.0)
    # Zero the command when stale, like the real connections.
    cmd_vel_timeout_s: float = Field(default=0.2, gt=0.0)
    # Subsample depth before unprojection; 2 is 4x fewer points.
    scan_stride: int = Field(default=2, ge=1)
    seed: int = 0
    publish_semantic: bool = False
    # Unprojection is the frame's main cost; off for teleop-only stacks.
    publish_scan: bool = True
    # "world" pre-registers the scan for VoxelGridMapper; "camera_optical" lets
    # RayTracingVoxelMap register it via tf.
    scan_frame: str = "world"


class HabitatConnection(NativeModule):
    """Drive a Habitat scene with Twist; publish what a depth robot would see."""

    config: HabitatConnectionConfig

    cmd_vel: In[Twist]

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    semantic_image: Out[Image]
