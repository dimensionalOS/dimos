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

"""Ideal head-left and wrist cameras on the 2025 vendor mount frames.

FOV/aspect defaults are reused from the prior 2026 camera profile, not measured
2025 intrinsics. That hardware profile uses an ISX031 stereo RGB pair.
Unreal supplies ideal aligned depth; it does not simulate stereo estimation.
The 118 x 92 degree vendor FOV is an approximation until factory rectified
intrinsics are supplied. The old 640x480 office-camera calibration was not R1's.
"""

import math

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.sim2.sensors.camera.config import CameraConfig


def head_camera(rectified_info: CameraInfo | None = None) -> CameraConfig:
    width, height = 640, 512
    if rectified_info is None:
        fx = width / (2 * math.tan(math.radians(118 / 2)))
        fy = height / (2 * math.tan(math.radians(92 / 2)))
        cx, cy = width / 2, height / 2
    else:
        if any(rectified_info.D):
            raise ValueError(
                "Use rectified head-left calibration; Unreal is an ideal pinhole renderer"
            )
        if rectified_info.width <= 0 or rectified_info.height <= 0:
            raise ValueError("Calibration dimensions must be positive")
        sx, sy = width / rectified_info.width, height / rectified_info.height
        fx, fy = rectified_info.K[0] * sx, rectified_info.K[4] * sy
        cx, cy = rectified_info.K[2] * sx, rectified_info.K[5] * sy
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
            raise ValueError("Invalid camera intrinsics")
    return CameraConfig(
        "r1pro/camera_head_left_link", width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy
    )


# Ideal depth range from the prior D405 profile, not a sensor noise model.
WRIST_MIN_DEPTH_M = 0.07
WRIST_MAX_DEPTH_M = 0.50


def wrist_camera(side: str) -> CameraConfig:
    if side not in ("left", "right"):
        raise ValueError("Wrist side must be left or right")
    # Vendor D405 link: preserve its mount pose; treat +Z as the ideal optical
    # axis. Factory optical-center offsets/calibration are not available here.
    width, height = 320, 180
    return CameraConfig(
        f"r1pro/{side}_D405_link",
        width=width,
        height=height,
        fx=width / (2 * math.tan(math.radians(87 / 2))),
        fy=height / (2 * math.tan(math.radians(58 / 2))),
        cx=width / 2,
        cy=height / 2,
    )
