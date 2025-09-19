#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

# Copyright 2025 Dimensional Inc.

"""Video streaming using GStreamer appsink for proper frame extraction."""

import functools
import subprocess
import threading
import time
import numpy as np
from typing import Optional
from reactivex import Subject

from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class DJIDroneVideoStream:
    """Capture drone video using GStreamer appsink."""

    def __init__(self, port: int = 5600):
        self.port = port
        self._video_subject = Subject()
        self._process = None
        self._running = False

    def start(self) -> bool:
        """Start video capture using gst-launch with appsink."""
        try:
            # Use appsink to get properly formatted frames
            # The ! at the end tells appsink to emit data on stdout in a parseable format
            cmd = [
                "gst-launch-1.0",
                "-q",
                "udpsrc",
                f"port={self.port}",
                "!",
                "application/x-rtp,encoding-name=H264,payload=96",
                "!",
                "rtph264depay",
                "!",
                "h264parse",
                "!",
                "avdec_h264",
                "!",
                "videoscale",
                "!",
                "video/x-raw,width=640,height=360",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=BGR",
                "!",
                "filesink",
                "location=/dev/stdout",
                "buffer-mode=2",  # Unbuffered output
            ]

            logger.info(f"Starting video capture on UDP port {self.port}")
            logger.debug(f"Pipeline: {' '.join(cmd)}")

            self._process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
            )

            self._running = True

            # Start capture thread
            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._capture_thread.start()

            # Start error monitoring
            self._error_thread = threading.Thread(target=self._error_monitor, daemon=True)
            self._error_thread.start()

            logger.info("Video stream started")
            return True

        except Exception as e:
            logger.error(f"Failed to start video stream: {e}")
            return False

    def _capture_loop(self):
        """Read frames with fixed size."""
        # Fixed parameters
        width, height = 640, 360
        channels = 3
        frame_size = width * height * channels

        logger.info(f"Capturing frames: {width}x{height} RGB ({frame_size} bytes per frame)")

        frame_count = 0
        total_bytes = 0

        while self._running:
            try:
                # Read exactly one frame worth of data
                frame_data = b""
                bytes_needed = frame_size

                while bytes_needed > 0 and self._running:
                    chunk = self._process.stdout.read(bytes_needed)
                    if not chunk:
                        logger.warning("No data from GStreamer")
                        time.sleep(0.1)
                        break
                    frame_data += chunk
                    bytes_needed -= len(chunk)

                if len(frame_data) == frame_size:
                    # We have a complete frame
                    total_bytes += frame_size

                    # Convert to numpy array
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = frame.reshape((height, width, channels))

                    # Create Image message (BGR format - matches GStreamer pipeline output)
                    img_msg = Image.from_numpy(frame, format=ImageFormat.BGR)

                    # Publish
                    self._video_subject.on_next(img_msg)

                    frame_count += 1
                    if frame_count == 1:
                        logger.info(f"First frame captured! Shape: {frame.shape}")
                    elif frame_count % 100 == 0:
                        logger.info(
                            f"Captured {frame_count} frames ({total_bytes / 1024 / 1024:.1f} MB)"
                        )

            except Exception as e:
                if self._running:
                    logger.error(f"Error in capture loop: {e}")
                time.sleep(0.1)

    def _error_monitor(self):
        """Monitor GStreamer stderr."""
        while self._running and self._process:
            try:
                line = self._process.stderr.readline()
                if line:
                    msg = line.decode("utf-8").strip()
                    if "ERROR" in msg or "WARNING" in msg:
                        logger.warning(f"GStreamer: {msg}")
                    else:
                        logger.debug(f"GStreamer: {msg}")
            except:
                pass

    def stop(self):
        """Stop video stream."""
        self._running = False

        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except:
                self._process.kill()
            self._process = None

        logger.info("Video stream stopped")

    def get_stream(self):
        """Get the video stream observable."""
        return self._video_subject


class FakeDJIVideoStream(DJIDroneVideoStream):
    """Replay video for testing."""

    def __init__(self, port: int = 5600):
        super().__init__(port)
        from dimos.utils.data import get_data

        # Ensure data is available
        get_data("drone")

    def start(self) -> bool:
        """Start replay of recorded video."""
        self._running = True
        logger.info("Video replay started")
        return True

    @functools.cache
    def get_stream(self):
        """Get the replay stream directly."""
        from dimos.utils.testing import TimedSensorReplay

        logger.info("Creating video replay stream")
        video_store = TimedSensorReplay("drone/video")
        return video_store.stream()

    def stop(self):
        """Stop replay."""
        self._running = False
        logger.info("Video replay stopped")
