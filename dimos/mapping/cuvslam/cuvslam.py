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

from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.nvidia_env import driver_env, sdk_variant

logger = setup_logger()


class CuvslamConfig(NativeModuleConfig):
    cwd: str | None = "."
    executable: str = "result/bin/cuvslam_odometry"
    build_command: str | None = Field(
        default_factory=lambda: f"nix build github:dimensionalOS/dimSLAM/v0.3.2#{sdk_variant()}"
    )
    stdin_config: bool = True
    extra_env: dict[str, str] = Field(default_factory=driver_env)

    # no default: this choice changes what inputs are required
    # multisensor (experimental in cuVSLAM): any mix of RGB and RGB-D cameras, optional IMU;
    # cameras listed in frame_id_to_depth_units_per_meter are the depth-providing ones
    camera_mode: Literal["mono", "stereo", "rgbd", "multisensor"]
    # reject way-too-fast movements, like moving a large box that takes up 90% of the camera view
    speed_gate_max_linear: float = 5.0  # meters per sec
    speed_gate_max_angular: float = 12.0  # radians per sec
    # "reject movements that cuVSLAM is not confident about"
    # good movements usually have a confidence of around 0.01-0.3, degenerate ones are 5-330
    # units = standard deviations of translation in meters
    # 0 disables
    covariance_gate_translation_std: float = 1.0

    # note: this is auto detected (including left/right via tf),
    # basically only use this for multi-cam setups, or for selecting a subset of cameras by frame_id
    # if in stereo mode, first=left side, second=right side
    # and left/right means relative to the camera
    # e.g. flipping a camera upside down should not change the order here
    camera_frames: list[str] = Field(default_factory=list)
    # have the images been pre-un-distorted (ideally yes, the sensor or API should un-distort)
    rectified: bool = True
    # works with MacOS Metal (apple silicon only) and Nvidia
    use_gpu: bool = True
    # rgbd (required) and multisensor: raw depth units per metre keyed by depth image frame_id
    # (e.g. {"d455_color_optical_frame": 1000.0} for 16-bit millimetre depth)
    frame_id_to_depth_units_per_meter: dict[str, float] = Field(default_factory=dict)

    map_frame_id: str = "map"
    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # rig_frame_id will default to output_frame_id, which is right 99% of the time
    # sometimes this will need to be the head-frame rather than chest/base_link (ex: X2)
    rig_frame_id: str = ""

    enable_loop_closure: bool = True
    publish_map_to_odom: bool = True
    slam_async: bool = False
    # 0 = unlimited
    slam_max_poses: int = 300
    slam_throttling_ms: int = 0
    # unless you have a REALLY good IMU keep this off
    enable_imu: bool = False

    @model_validator(mode="after")
    def _check_mode_combinations(self) -> CuvslamConfig:
        if self.camera_mode == "rgbd" and not self.frame_id_to_depth_units_per_meter:
            raise ValueError(
                "camera_mode='rgbd' requires frame_id_to_depth_units_per_meter, "
                'e.g. {"d455_color_optical_frame": 1000.0}'
            )
        if self.enable_imu and self.camera_mode not in ("stereo", "multisensor"):
            raise ValueError(
                "enable_imu requires camera_mode='stereo' or 'multisensor'; "
                "cuVSLAM has no inertial mono or rgbd mode"
            )
        if self.camera_mode == "multisensor" and not self.camera_frames:
            raise ValueError(
                "camera_mode='multisensor' requires camera_frames; "
                "auto-discovery cannot know how many cameras the rig has"
            )
        return self


class CuvslamOdometry(NativeModule):
    """Visual odometry on one to thirty-two cameras.

    Usage
    1. Funnel all ir and rgb camera images to ``image``
    2. Funnel their respective camera info's to ``camera_info``
    3. Make sure all of those^ have correct frame_id's
    4. Publish static tf's to relate all those frame_id's
    5. Set `camera_mode` to mono/stereo/rgbd
    """

    config: CuvslamConfig

    image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    imu: In[Imu]
    imu_info: In[ImuInfo]

    odometry: Out[Odometry]
    corrected_odometry: Out[Odometry]
    tf: IO[TFMessage]
