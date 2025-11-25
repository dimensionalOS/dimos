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

from abc import ABC, abstractmethod
from pathlib import Path
import subprocess
from typing import Literal

AnnotatorType = Literal["rgb", "normals", "bounding_box_3d", "motion_vectors"]
TransportType = Literal["tcp", "udp"]


class StreamBase(ABC):
    """Base class for simulation streaming."""

    @abstractmethod
    def __init__(  # type: ignore[no-untyped-def]
        self,
        simulator,
        width: int = 1920,
        height: int = 1080,
        fps: int = 60,
        camera_path: str = "/World/camera",
        annotator_type: AnnotatorType = "rgb",
        transport: TransportType = "tcp",
        rtsp_url: str = "rtsp://mediamtx:8554/stream",
        usd_path: str | Path | None = None,
    ) -> None:
        """Initialize the stream.

        Args:
            simulator: Simulator instance
            width: Stream width in pixels
            height: Stream height in pixels
            fps: Frames per second
            camera_path: Camera path in scene
            annotator: Type of annotator to use
            transport: Transport protocol
            rtsp_url: RTSP stream URL
            usd_path: Optional USD file path to load
        """
        self.simulator = simulator
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_path = camera_path
        self.annotator_type = annotator_type
        self.transport = transport
        self.rtsp_url = rtsp_url
        self.proc = None

    @abstractmethod
    def _load_stage(self, usd_path: str | Path):  # type: ignore[no-untyped-def]
        """Load stage from file."""
        pass

    @abstractmethod
    def _setup_camera(self):  # type: ignore[no-untyped-def]
        """Setup and validate camera."""
        pass

    def _setup_ffmpeg(self) -> None:
        """Setup FFmpeg process for streaming."""
        command = [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{self.width}x{self.height}",
            "-r",
            str(self.fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "h264_nvenc",
            "-preset",
            "fast",
            "-f",
            "rtsp",
            "-rtsp_transport",
            self.transport,
            self.rtsp_url,
        ]
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE)  # type: ignore[assignment]

    @abstractmethod
    def _setup_annotator(self):  # type: ignore[no-untyped-def]
        """Setup annotator."""
        pass

    @abstractmethod
    def stream(self):  # type: ignore[no-untyped-def]
        """Start streaming."""
        pass

    @abstractmethod
    def cleanup(self):  # type: ignore[no-untyped-def]
        """Cleanup resources."""
        pass
