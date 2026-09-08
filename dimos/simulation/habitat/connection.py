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

    # Own flake: habitat-sim is python 3.9 conda-only, so it cannot share the
    # dimos interpreter. install.sh builds env/ and writes the wrapper, whose
    # existence is what NativeModule uses as the build sentinel.
    cwd: str | None = "nix"
    executable: str = "habitat-native"
    build_command: str | None = "nix develop path:. -c ./install.sh"
    stdin_config: bool = True
    log_format: LogFormat = LogFormat.TEXT

    # Relative to cwd, so the module carries its own scenes. hm3d_example is a
    # real annotated HM3D house and needs no Matterport credentials.
    scene_dataset_config: str = (
        "data/versioned_data/hm3d-0.2/hm3d/example"
        "/hm3d_annotated_example_basis.scene_dataset_config.json"
    )
    scene_id: str = "00861-GLAQ4DNUx5U"

    # Defaults are Go2-ish, not GOAT-Bench's 1.41 m Stretch.
    width: int = 640
    height: int = 360
    hfov_deg: float = Field(default=90.0, gt=0.0, lt=180.0)
    camera_height_m: float = Field(default=0.45, gt=0.0)
    max_depth_m: float = Field(default=5.0, gt=0.0)

    sim_rate_hz: float = Field(default=10.0, gt=0.0)
    # Subsample the depth image before unprojection: stride 2 is 4x fewer points
    # into the voxel grid, which is where the frame time goes.
    scan_stride: int = Field(default=2, ge=1)
    seed: int = 0
    publish_semantic: bool = False
    # Off for teleop-only stacks: unprojection is the frame's main cost.
    publish_scan: bool = True
    # Frame the scan is published in. "world" pre-registers it, which is what
    # VoxelGridMapper wants. RayTracingVoxelMap instead needs the sensor frame,
    # because it raytraces from the sensor origin and places the cloud itself by
    # the tf lookup world_frame -> cloud frame_id; set "camera_optical" for it.
    scan_frame: str = "world"


class HabitatConnection(NativeModule):
    """Drive a Habitat scene with Twist and publish what a depth robot would see.

    ``registered_scan`` is already in world frame, which is what
    :class:`~dimos.mapping.voxels.module.VoxelGridMapper` requires. Habitat hands
    the native exact ground-truth pose, so there is no odometry drift to correct.
    """

    config: HabitatConnectionConfig

    cmd_vel: In[Twist]

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    semantic_image: Out[Image]
