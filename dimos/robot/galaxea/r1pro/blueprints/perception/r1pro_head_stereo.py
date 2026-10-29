# Copyright 2025-2026 Dimensional Inc.
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

"""Head-only RGB-D: no arms, torso, chassis, grippers or navigation control.

Starting this blueprint opens the head cameras and configures their FSYNC.
It does not publish robot base TF; the camera mount must be verified separately.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.stereo.module import StereoDepth
from dimos.robot.galaxea.r1pro.head_cameras import (
    HEAD_LEFT_FRAME,
    HEAD_RIGHT_FRAME,
    HEAD_STEREO_CALIBRATION,
    HeadLeftCamera,
    HeadRightCamera,
)

r1pro_head_stereo = (
    autoconnect(
        HeadLeftCamera.blueprint(),
        HeadRightCamera.blueprint(),
        StereoDepth.blueprint(
            calibration_path=HEAD_STEREO_CALIBRATION,
            left_frame=HEAD_LEFT_FRAME,
            right_frame=HEAD_RIGHT_FRAME,
            frame_id="r1pro_head_left_rectified_optical",
        ),
    )
    .remappings(
        [
            (HeadLeftCamera, "jpeg_out", "head_left_color"),
            (HeadRightCamera, "jpeg_out", "head_right_color"),
            (StereoDepth, "left", "head_left_color"),
            (StereoDepth, "right", "head_right_color"),
            (StereoDepth, "color_image", "head_rgbd_color"),
            (StereoDepth, "depth_image", "head_rgbd_depth"),
            (StereoDepth, "camera_info", "head_rgbd_info"),
            (StereoDepth, "confidence", "head_rgbd_confidence"),
        ]
    )
    .global_config(n_workers=3, viewer="none", transport="zenoh")
)
