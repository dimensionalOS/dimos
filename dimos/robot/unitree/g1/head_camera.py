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

"""G1 head-camera constants shared by the perception blueprint and its tools."""

from __future__ import annotations

from pathlib import Path

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The camera module names its frames from `camera_name`, and marker detection
# reads the frame off the image, so these must agree with the TF chain.
HEAD_CAMERA_NAME = "head_camera"
HEAD_CAMERA_OPTICAL_FRAME = f"{HEAD_CAMERA_NAME}_color_optical_frame"
# Fixed camera body link in the G1 URDF; G1HeadCameraTf publishes pelvis->here.
HEAD_CAMERA_MOUNT_FRAME = "d435_link"

HEAD_CAMERA_INFO_YAML = Path(__file__).resolve().parent / "artifacts" / "head_camera_info.yaml"

# Must match the captured intrinsics exactly: marker detection skips any frame
# whose size differs from CameraInfo, silently. tool_dump_camera_info prints
# this unit's real modes if librealsense rejects these.
CAMERA_STREAM_CONFIG = {"width": 848, "height": 480, "fps": 15, "enable_depth": False}


def head_camera_info() -> CameraInfo | None:
    """Intrinsics captured from this robot's own camera, or None if uncaptured.

    No fallback intrinsics on purpose: marker range scales with focal length,
    so a plausible-looking guess would put the pot at the wrong distance and
    the arm would plan to empty space. Without this file marker detection
    emits nothing, which is the failure we want.
    """
    if not HEAD_CAMERA_INFO_YAML.exists():
        logger.warning(
            "G1 head camera intrinsics missing — marker detection will publish "
            "no detections. Capture them on the robot with the camera plugged in: "
            "`.venv/bin/python -m dimos.robot.unitree.g1.tool_dump_camera_info` "
            f"(expected at {HEAD_CAMERA_INFO_YAML})"
        )
        return None
    info = CameraInfo.from_yaml(str(HEAD_CAMERA_INFO_YAML))
    info.frame_id = HEAD_CAMERA_OPTICAL_FRAME
    return info
