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

"""RTSP camera for M20 quadruped.

The M20 serves camera feeds via go2rtc/RTSP at:
    Front: rtsp://<M20_IP>:8554/video1
    Rear:  rtsp://<M20_IP>:8554/video2

Uses PyAV (FFmpeg) for H.265/HEVC decoding.
"""

import logging
import threading
import time

import av
import numpy as np
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

logger = logging.getLogger(__name__)

# Default M20 camera intrinsics (approximate — should be calibrated per unit)
_DEFAULT_CAMERA_INFO = CameraInfo(
    frame_id="camera_optical",
    height=720,
    width=1280,
    distortion_model="plumb_bob",
    D=[0.0, 0.0, 0.0, 0.0, 0.0],
    K=[900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0],
    R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    P=[900.0, 0.0, 640.0, 0.0, 0.0, 900.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    binning_x=0,
    binning_y=0,
)


class M20RTSPCamera:
    """Reads RTSP H.265 video from the M20 and produces an Observable image stream.

    Usage:
        camera = M20RTSPCamera("10.21.31.103")
        camera.start()
        stream = camera.image_stream()
        stream.subscribe(lambda img: ...)
        camera.stop()
    """

    def __init__(
        self,
        host: str = "10.21.31.103",
        rtsp_port: int = 8554,
        stream_path: str = "video1",
        frame_id: str = "camera_optical",
        target_fps: float = 15.0,
    ):
        self.url = f"rtsp://{host}:{rtsp_port}/{stream_path}"
        self.frame_id = frame_id
        self.target_fps = target_fps

        self._subject: Subject[Image] = Subject()
        self._thread: threading.Thread | None = None
        self._running = False

    @property
    def camera_info(self) -> CameraInfo:
        return _DEFAULT_CAMERA_INFO

    def image_stream(self) -> Observable[Image]:
        return self._subject

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info(f"RTSP camera started: {self.url}")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
            self._thread = None
        logger.info("RTSP camera stopped")

    def _capture_loop(self) -> None:
        min_interval = 1.0 / self.target_fps if self.target_fps > 0 else 0.0

        while self._running:
            try:
                self._stream_from_rtsp(min_interval)
            except av.error.ExitError:
                logger.warning("RTSP stream ended, reconnecting in 2s...")
            except (ConnectionError, OSError, av.error.InvalidDataError) as e:
                logger.warning(f"RTSP connection error: {e}, reconnecting in 2s...")
            except Exception:
                logger.exception("Unexpected RTSP error, reconnecting in 2s...")

            if self._running:
                time.sleep(2.0)

    def _stream_from_rtsp(self, min_interval: float) -> None:
        container = av.open(
            self.url,
            options={
                "rtsp_transport": "tcp",
                "stimeout": "5000000",  # 5s connection timeout
            },
        )

        try:
            stream = container.streams.video[0]
            stream.thread_type = "AUTO"

            last_frame_time = 0.0

            for frame in container.decode(stream):
                if not self._running:
                    break

                now = time.time()
                if now - last_frame_time < min_interval:
                    continue
                last_frame_time = now

                # Convert to RGB numpy array
                rgb_frame = frame.to_ndarray(format="rgb24")

                image = Image.from_numpy(
                    rgb_frame,
                    format=ImageFormat.RGB,
                    frame_id=self.frame_id,
                    ts=now,
                )

                self._subject.on_next(image)
        finally:
            container.close()
