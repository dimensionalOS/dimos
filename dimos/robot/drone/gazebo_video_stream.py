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

"""Gazebo RTP/H264 video stream via GStreamer (e.g. ArduPilot SITL)."""

import subprocess
import threading
import time

import numpy as np
from reactivex import Subject

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GAZEBO_RTP_CAPS = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264"


class GazeboVideoStream:
    def __init__(self, port: int = 5600, width: int = 640, height: int = 360) -> None:
        self.port = port
        self.width = width
        self.height = height
        self._video_subject: Subject[Image] = Subject()
        self._process: subprocess.Popen[bytes] | None = None
        self._stop_event = threading.Event()

    def start(self) -> bool:
        try:
            cmd = [
                "gst-launch-1.0",
                "-q",
                "udpsrc",
                f"port={self.port}",
                "!",
                GAZEBO_RTP_CAPS,
                "!",
                "rtph264depay",
                "!",
                "h264parse",
                "!",
                "avdec_h264",
                "!",
                "videoscale",
                "!",
                f"video/x-raw,width={self.width},height={self.height}",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=RGB",
                "!",
                "filesink",
                "location=/dev/stdout",
                "buffer-mode=2",
            ]
            self._process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
            )
            self._stop_event.clear()
            threading.Thread(target=self._capture_loop, daemon=True).start()
            threading.Thread(target=self._error_monitor, daemon=True).start()
            logger.info("Gazebo video started port %s", self.port)
            return True
        except Exception as e:
            logger.error("Gazebo video failed: %s", e)
            return False

    def _capture_loop(self) -> None:
        channels = 3
        frame_size = self.width * self.height * channels
        while not self._stop_event.is_set():
            try:
                frame_data = b""
                bytes_needed = frame_size
                while bytes_needed > 0 and not self._stop_event.is_set():
                    if self._process is None or self._process.stdout is None:
                        break
                    chunk = self._process.stdout.read(bytes_needed)
                    if not chunk:
                        time.sleep(0.1)
                        break
                    frame_data += chunk
                    bytes_needed -= len(chunk)
                if len(frame_data) == frame_size:
                    frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                        (self.height, self.width, channels)
                    )
                    self._video_subject.on_next(Image.from_numpy(frame, format=ImageFormat.RGB))
            except Exception as e:
                if not self._stop_event.is_set():
                    logger.error("Gazebo capture error: %s", e)
                time.sleep(0.1)

    def _error_monitor(self) -> None:
        while not self._stop_event.is_set() and self._process and self._process.stderr:
            line = self._process.stderr.readline()
            if not line:
                continue
            s = line.decode("utf-8", errors="replace").strip()
            if "ERROR" in s or "WARNING" in s:
                logger.warning("GStreamer: %s", s)

    def stop(self) -> None:
        self._stop_event.set()
        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
        logger.info("Gazebo video stopped")

    def get_stream(self) -> Subject[Image]:
        return self._video_subject
