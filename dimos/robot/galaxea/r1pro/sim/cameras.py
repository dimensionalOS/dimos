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

"""Head and wrist cameras as the hardware publishes them, on the 2025 vendor mount frames.

Each camera has one color stream, its own: a head eye at 1920x1536 and a wrist
at 1280x720, as JPEG at 30 Hz. Beside it the robot's spatial sensors publish
depth for the mesh view, as an ideal pinhole image at 5 Hz: the rectified left
head view 480 pixels wide and both wrist views at 1280x720. The simulation has
the same streams. The renderer supplies ideal depth; it does not simulate stereo
estimation, and no lens distortion is rendered.

Head intrinsics come from a copy of the robot's factory stereo calibration when
HEAD_CALIBRATION exists: the eye's own for its color, and the rectified pair's
for its depth, exactly as the hardware rectifies it. Without that file the
118 x 92 degree vendor field of view stands in. Rectification keeps only valid
pixels, so the stand-in can be wider than what the robot publishes.
Wrist intrinsics are the D405's nominal 87 x 58 degrees: its factory values are
only in the robot's ROS camera_info.
"""

import math
from pathlib import Path
from typing import Any

from dimos.constants import CACHE_DIR
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.galaxea.r1pro.head_cameras import (
    HEAD_FPS,
    HEAD_HEIGHT,
    HEAD_WIDTH,
    head_camera_infos,
)
from dimos.robot.galaxea.r1pro.spatial_sensors import R1SpatialSensorsConfig
from dimos.robot.galaxea.r1pro.stereo import HeadStereo
from dimos.sim2.sensors.camera.config import CameraConfig

# What R1SpatialSensors publishes, read from its own defaults.
SPATIAL_HZ: float = R1SpatialSensorsConfig.model_fields["publish_hz"].default
SPATIAL_HEAD_WIDTH: int = R1SpatialSensorsConfig.model_fields["stereo_width"].default
# A copy of the robot's /opt/galaxea/body/stereo.yaml, beside the vendor model.
HEAD_CALIBRATION = CACHE_DIR / "r1pro-hardware/stereo.yaml"
# The wrist D405 color mode on the robot.
WRIST_WIDTH, WRIST_HEIGHT = 1280, 720
WRIST_FPS = 30.0
# The vendor wrist driver sends ROS's default JPEG quality; its frames measured
# about 250 kB each on the robot.
WRIST_JPEG_QUALITY = 95
# The vendor field of view of a head eye, in degrees, where no calibration is at hand.
_HEAD_FOV = (118, 92)


def head_depth_camera(
    rectified_info: CameraInfo | None = None, *, calibration: Path = HEAD_CALIBRATION
) -> CameraConfig:
    """Depth in the rectified left head view; ``rectified_info`` overrides the calibration file."""
    if rectified_info is None and calibration.exists():
        rectified_info = HeadStereo(calibration, SPATIAL_HEAD_WIDTH).info
    if rectified_info is None:
        width = SPATIAL_HEAD_WIDTH
        height = round(HEAD_HEIGHT * width / HEAD_WIDTH)
        fx = width / (2 * math.tan(math.radians(_HEAD_FOV[0] / 2)))
        fy = height / (2 * math.tan(math.radians(_HEAD_FOV[1] / 2)))
        cx, cy = width / 2, height / 2
    else:
        if any(rectified_info.D):
            raise ValueError(
                "Use rectified head-left calibration; Unreal is an ideal pinhole renderer"
            )
        if rectified_info.width <= 0 or rectified_info.height <= 0:
            raise ValueError("Calibration dimensions must be positive")
        # Any size of the same rectified view scales to the published width.
        width = SPATIAL_HEAD_WIDTH
        height = round(rectified_info.height * width / rectified_info.width)
        sx, sy = width / rectified_info.width, height / rectified_info.height
        fx, fy = rectified_info.K[0] * sx, rectified_info.K[4] * sy
        cx, cy = rectified_info.K[2] * sx, rectified_info.K[5] * sy
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
            raise ValueError("Invalid camera intrinsics")
    return CameraConfig(
        "r1pro/camera_head_left_link",
        hz=SPATIAL_HZ,
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        color=False,
    )


# Ideal depth range from the prior D405 profile, not a sensor noise model.
WRIST_MIN_DEPTH_M = 0.07
WRIST_MAX_DEPTH_M = 0.50


def _wrist_camera(side: str, **kind: Any) -> CameraConfig:
    if side not in ("left", "right"):
        raise ValueError("Wrist side must be left or right")
    # Vendor D405 link: preserve its mount pose; treat +Z as the ideal optical
    # axis. Factory optical-center offsets/calibration are not available here.
    return CameraConfig(
        f"r1pro/{side}_D405_link",
        width=WRIST_WIDTH,
        height=WRIST_HEIGHT,
        fx=WRIST_WIDTH / (2 * math.tan(math.radians(87 / 2))),
        fy=WRIST_HEIGHT / (2 * math.tan(math.radians(58 / 2))),
        cx=WRIST_WIDTH / 2,
        cy=WRIST_HEIGHT / 2,
        **kind,
    )


def wrist_depth_camera(side: str) -> CameraConfig:
    """One wrist camera's depth as the spatial sensors publish it."""
    return _wrist_camera(
        side,
        hz=SPATIAL_HZ,
        min_depth_m=WRIST_MIN_DEPTH_M,
        max_depth_m=WRIST_MAX_DEPTH_M,
        color=False,
    )


def wrist_color_camera(side: str) -> CameraConfig:
    """One wrist camera's color as the vendor driver publishes it: 30 Hz, no depth."""
    return _wrist_camera(side, hz=WRIST_FPS, depth=False)


def head_color_camera(side: str, *, calibration: Path = HEAD_CALIBRATION) -> CameraConfig:
    """One head eye as its camera publishes it: full size, at the sensor's rate, color only.

    The focal lengths and principal point are the eye's own from the factory
    calibration when a copy exists, else the vendor field of view.
    """
    if side not in ("left", "right"):
        raise ValueError("Head eye must be left or right")
    if calibration.exists():
        info = head_camera_infos(str(calibration))[("left", "right").index(side)]
        if (info.width, info.height) != (HEAD_WIDTH, HEAD_HEIGHT):
            raise ValueError("Head calibration is not for the camera's full size")
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
    else:
        fx = HEAD_WIDTH / (2 * math.tan(math.radians(_HEAD_FOV[0] / 2)))
        fy = HEAD_HEIGHT / (2 * math.tan(math.radians(_HEAD_FOV[1] / 2)))
        cx, cy = HEAD_WIDTH / 2, HEAD_HEIGHT / 2
    return CameraConfig(
        f"r1pro/camera_head_{side}_link",
        hz=HEAD_FPS,
        width=HEAD_WIDTH,
        height=HEAD_HEIGHT,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        depth=False,
    )
