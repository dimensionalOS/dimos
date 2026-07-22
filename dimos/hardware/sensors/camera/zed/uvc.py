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

"""SDK-free ZED stereo camera (plain UVC / V4L2, no pyzed).

Every ZED enumerates as a standard UVC webcam that delivers both eyes packed
side-by-side in one frame (left eye in the left half). This module grabs that
combined frame with OpenCV, splits it down the middle, and publishes the halves
as ``color_image_left`` / ``color_image_right`` — no ZED SDK, no CUDA, no depth.

Defaults target the 60 fps HD720 stereo mode (2 x 1280x720 packed as 2560x720).
The requested mode must be one the camera actually supports (see ZED datasheet:
2.2K@15, 1080p@30, 720p@60, VGA@100); UVC silently falls back otherwise, so the
actual negotiated mode is logged at start.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time

import cv2
import numpy as np
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def find_zed_device() -> int:
    """The lowest /dev/videoN index whose v4l2 device name contains "ZED"."""
    candidates: list[int] = []
    for node in Path("/sys/class/video4linux").glob("video*"):
        try:
            name = (node / "name").read_text()
        except OSError:
            continue
        if "zed" in name.lower():
            candidates.append(int(node.name.removeprefix("video")))
    if not candidates:
        raise RuntimeError("No ZED UVC device found (no /dev/video* names contain 'ZED')")
    return min(candidates)


class ZedUvcCameraConfig(ModuleConfig):
    camera_index: int | None = None  # /dev/videoN; None = auto-detect by v4l2 name
    # Per-eye resolution; the UVC device is asked for a 2*width x height frame.
    width: int = 1280
    height: int = 720
    fps: float = Field(default=60.0, gt=0.0)
    left_frame_id: str = "zed_left_optical_frame"
    right_frame_id: str = "zed_right_optical_frame"


class ZedUvcCamera(Module):
    """Publishes the two ZED eyes as independent 60 fps color streams."""

    config: ZedUvcCameraConfig

    color_image_left: Out[Image]
    color_image_right: Out[Image]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._capture: cv2.VideoCapture | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._index = -1

    @rpc
    def start(self) -> None:
        super().start()
        if self._thread and self._thread.is_alive():
            return

        self._index = (
            self.config.camera_index if self.config.camera_index is not None else find_zed_device()
        )
        capture = cv2.VideoCapture(self._index)
        if not capture.isOpened():
            raise RuntimeError(f"Failed to open ZED UVC device {self._index}")

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width * 2)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)

        actual_w = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        logger.info(
            "ZED UVC device %d negotiated %dx%d @ %.0f fps (requested %dx%d @ %.0f)",
            self._index,
            actual_w,
            actual_h,
            actual_fps,
            self.config.width * 2,
            self.config.height,
            self.config.fps,
        )
        if actual_w != self.config.width * 2 or actual_h != self.config.height:
            logger.warning(
                "ZED UVC mode mismatch — splitting the %dx%d frame down the middle anyway",
                actual_w,
                actual_h,
            )

        self._capture = capture
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self) -> None:
        while self._capture and not self._stop_event.is_set():
            ret, frame = self._capture.read()
            if not ret:
                if self._stop_event.is_set():
                    break
                logger.warning("ZED UVC device %d dropped a frame", self._index)
                continue
            ts = time.time()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            half = frame_rgb.shape[1] // 2
            left = np.ascontiguousarray(frame_rgb[:, :half])
            right = np.ascontiguousarray(frame_rgb[:, half:])
            self.color_image_left.publish(
                Image.from_numpy(
                    left, format=ImageFormat.RGB, frame_id=self.config.left_frame_id, ts=ts
                )
            )
            self.color_image_right.publish(
                Image.from_numpy(
                    right, format=ImageFormat.RGB, frame_id=self.config.right_frame_id, ts=ts
                )
            )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._thread = None
        if self._capture:
            self._capture.release()
            self._capture = None
        super().stop()
