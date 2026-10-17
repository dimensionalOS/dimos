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

"""The R1 Pro's two head cameras, colour read straight off V4L2.

The head is a stereo pair of GMSL2 cameras (SENSING SG3S-ISX031C-GMSL2F, a Sony
ISX031 with its own ISP) behind a MAX96724 deserializer on the Jetson. It is
not a ZED. The ISP delivers finished UYVY frames to the Tegra VI, which exposes
them as plain V4L2 capture nodes, so the native ``V4L2Camera`` reads them and
NVJPG encodes them; nvarguscamerasrc does not apply, as it needs raw Bayer.

Only 1920x1536 at 30 fps is real. The driver lists smaller sizes and 60 fps,
but a smaller size is a corrupted crop of the full frame and 60 fps still
delivers 30. Galaxea's stereo calibration (``/opt/galaxea/body/stereo*.yaml``)
is for this size.

The vendor head pane (``hdas``, ``start_signal_camera_head.sh``) must not be
holding the cameras; while it is, these modules log and retry.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.utils.fsync_trigger import trigger_gmsl_cameras
from dimos.hardware.sensors.camera.v4l2.module import V4L2Camera, V4L2CameraConfig
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The VI port numbers are fixed by the device tree, so by-path pins each eye to
# its GMSL link however the nodes are numbered.
_HEAD_V4L2 = "/dev/v4l/by-path/platform-tegra-capture-vi-video-index{index}"
HEAD_LEFT_V4L2 = _HEAD_V4L2.format(index=0)
HEAD_RIGHT_V4L2 = _HEAD_V4L2.format(index=10)

HEAD_WIDTH = 1920
HEAD_HEIGHT = 1536
# The rate the ISX031 runs at, free or hardware-triggered.
HEAD_FPS = 30.0
HEAD_FOURCC = "UYVY"
# Every link on the head's deserializer: a mask of only the two head links stops them streaming.
HEAD_TRIGGER_LINKS = 0x0F

# The vendor URDF's optical frames for each eye.
HEAD_LEFT_FRAME = "camera_head_left_link"
HEAD_RIGHT_FRAME = "camera_head_right_link"

# Galaxea's factory stereo calibration for the 1920x1536 mode, on the robot.
HEAD_STEREO_CALIBRATION = "/opt/galaxea/body/stereo.yaml"


class HeadLeftCameraConfig(V4L2CameraConfig):
    device: str = HEAD_LEFT_V4L2
    width: int = HEAD_WIDTH
    height: int = HEAD_HEIGHT
    fourcc: str = HEAD_FOURCC
    frame_id: str = HEAD_LEFT_FRAME


class HeadRightCameraConfig(HeadLeftCameraConfig):
    device: str = HEAD_RIGHT_V4L2
    frame_id: str = HEAD_RIGHT_FRAME


# Distinct classes only because blueprints can't yet run two instances of one
# module (same reason as the wrist cameras).
class HeadLeftCamera(V4L2Camera):
    config: HeadLeftCameraConfig

    @rpc
    def start(self) -> None:
        trigger_gmsl_cameras(int(HEAD_FPS), HEAD_TRIGGER_LINKS)
        super().start()


class HeadRightCamera(V4L2Camera):
    config: HeadRightCameraConfig

    @rpc
    def start(self) -> None:
        trigger_gmsl_cameras(int(HEAD_FPS), HEAD_TRIGGER_LINKS)
        super().start()


def head_camera_infos(path: str = HEAD_STEREO_CALIBRATION) -> tuple[CameraInfo, CameraInfo]:
    """Each eye's unrectified intrinsics from Galaxea's OpenCV stereo calibration file."""
    import cv2

    storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise FileNotFoundError(path)
    width, height = (
        int(storage.getNode("image_width").real()),
        int(storage.getNode("image_height").real()),
    )

    def eye(side: str, frame_id: str) -> CameraInfo:
        k = storage.getNode(f"K_{side}").mat().ravel().tolist()
        # 14 coefficients; past the first eight (rational polynomial) they are the unused tilt terms.
        d = storage.getNode(f"D_{side}").mat().ravel().tolist()[:8]
        return CameraInfo(
            height=height,
            width=width,
            distortion_model="rational_polynomial",
            D=d,
            K=k,
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0],
            frame_id=frame_id,
        )

    return eye("left", HEAD_LEFT_FRAME), eye("right", HEAD_RIGHT_FRAME)


# How often to look again for a calibration file that is not there yet.
_CALIBRATION_RETRY_S = 5.0


class HeadCameraInfoConfig(ModuleConfig):
    calibration_path: str = HEAD_STEREO_CALIBRATION
    publish_hz: float = 1.0


class HeadCameraInfo(Module):
    """Publish both head eyes' intrinsics from the factory calibration, re-stamped each time."""

    config: HeadCameraInfoConfig

    left_info: Out[CameraInfo]
    right_info: Out[CameraInfo]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="HeadCameraInfo")
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    def _run(self) -> None:
        while True:
            try:
                left, right = head_camera_infos(self.config.calibration_path)
                break
            except FileNotFoundError:
                logger.warning(
                    "no head calibration at %s yet; retrying", self.config.calibration_path
                )
                if self._stop.wait(_CALIBRATION_RETRY_S):
                    return
        while not self._stop.is_set():
            for out, info in ((self.left_info, left), (self.right_info, right)):
                info.ts = time.time()
                out.publish(info)
            self._stop.wait(1.0 / self.config.publish_hz)
