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

import datetime
import logging
import time
from typing import TypedDict

import cv2
import numpy as np
from dask.distributed import get_client, get_worker
from distributed.actor import Actor as DistributedActor
from streamz import Stream

logger = logging.getLogger(__name__)


class Frame(TypedDict):
    frame: np.ndarray  # The actual image data from cv2
    timestamp: float  # Unix timestamp when frame was captured
    frame_number: int  # Sequential frame number


def deploy_actor(dask_client, actor_class, *args, **kwargs):
    return dask_client.submit(
        actor_class,
        *args,
        **kwargs,
        actor=True,
    ).result()


class RemoteStream(Stream):
    def __init__(self, actor):
        self.actor = actor
        super().__init__(asynchronous=True)
        self.actor.connect(self)

    def emit_remote(self, msg):
        self.emit(msg)


class LocalStream:
    actor: "Actor"
    stream: Stream

    def __init__(self, actor: "Actor"):
        self.actor = actor
        self.stream = Stream(asynchronous=True)
        self.worker = None
        self.actor_key = None

    def connect(self, stream):
        self.stream.sink(stream.emit_remote)

    def __reduce__(self):
        return (RemoteStream, (self.actor.proxy,))


class Actor:
    stream: LocalStream

    def __init__(self):
        self._stream = LocalStream(self)
        self.worker = get_worker()

    @property
    def key(self):
        for key, actor_instance in self.worker.state.actors.items():
            if actor_instance is self:
                return key

    @property
    def proxy(self):
        return DistributedActor(self.__class__, self.worker.address, self.key, worker=self.worker)

    @property
    def stream(self):
        return self._stream

    def connect(self, stream):
        self.stream.connect(stream.emit_remote)

    def emit(self, msg):
        self._stream.stream.emit(msg)


class TimedFrame(Frame):
    latency: float


class LatencyActor:
    avg_latency: float = 0
    frame_count: int = 0

    def __init__(self, name, verbose=False):
        self.client = get_client()
        self.name = name
        self.verbose = verbose
        self.stream = Stream(asynchronous=True)
        self.stream.map(self._measure_latency).map(self._update_avg_latency).sink(
            lambda frame: print(f"{self.name}: {frame}") if self.verbose else None
        )
        # self.stream.sink(lambda frame: print(f"{self.name}: {frame}") if self.verbose else None)

    def _measure_latency(self, frame: Frame) -> TimedFrame:
        time_diff = (
            datetime.datetime.now() - datetime.datetime.fromtimestamp(frame["timestamp"])
        ).total_seconds() * 1_000

        timed_frame: TimedFrame = {
            "frame": frame["frame"],
            "timestamp": frame["timestamp"],
            "frame_number": frame["frame_number"],
            "latency": time_diff,
        }
        return timed_frame

    def _update_avg_latency(self, timed_frame: TimedFrame) -> TimedFrame:
        time_diff = timed_frame["latency"]

        self.frame_count += 1
        self.avg_latency = (
            self.avg_latency * (self.frame_count - 1) + time_diff
        ) / self.frame_count

        return timed_frame

    async def get_latency(self) -> float:
        return self.avg_latency

    async def receive_frame(self, frame: Frame) -> None:
        # print("LatencyActor received frame", frame)
        self.stream.emit(frame)


class CameraActor:
    stream: Stream = Stream(asynchronous=True)

    def __init__(self, camera_index=None, width=640, height=480):
        """
        Initialize the camera loop.

        Args:
            camera_index: Camera device index (None for auto-select first working camera)
            width: Frame width in pixels
            height: Frame height in pixels
        """
        self.client = get_client()
        self.camera_index = camera_index

        self.width = width
        self.height = height
        self.cap = None
        self.frame_count = 0

    def _initialize_camera(self):
        """Initialize the camera capture."""
        if self.cap is None or not self.cap.isOpened():
            if self.cap:
                self.cap.release()

            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self.camera_index}")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            # Get actual properties (camera might not support exact values)
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            logger.info(f"Camera initialized: {actual_width}x{actual_height}")

    def add_processor(self, processor):
        """Add a processor to receive camera frames."""
        self.stream.sink(processor.receive_frame)

    def add_processors(self, *processors):
        """Add multiple processors to receive camera frames."""
        for processor in processors:
            self.add_processor(processor)

    async def run(self, total_frames=None):
        """
        Run the camera loop to capture and emit frames.

        Args:
            total_frames: Maximum number of frames to capture (None for infinite)
        """

        self._initialize_camera()

        start_time = time.time()

        while True:
            # Capture frame
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to capture frame from camera")
                break

            # Create frame data with timestamp and frame number
            frame_data: Frame = {
                "frame": frame,
                "timestamp": time.time(),
                "frame_number": self.frame_count,
            }

            # Emit the frame
            print("CameraActor emitting frame", self.frame_count)
            await self.stream.emit(frame_data)
            self.frame_count += 1

            # Check if we've reached the frame limit
            if total_frames is not None and self.frame_count >= total_frames:
                break

        total_time = time.time() - start_time
        avg_fps = self.frame_count / total_time if total_time > 0 else 0
        logger.info(
            f"Camera loop completed: {self.frame_count} frames in {total_time:.2f}s (avg {avg_fps:.1f} FPS)"
        )

    def cleanup(self):
        """Clean up camera resources."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            logger.info("Camera released")

    def __del__(self):
        """Destructor to ensure camera is released."""
        self.cleanup()
