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

"""Video provider module for capturing and streaming video frames.

This module provides classes for capturing video from various sources and
exposing them as reactive observables. It handles resource management,
frame rate control, and thread safety.
"""

# Standard library imports
import logging
import os
import time
from abc import ABC, abstractmethod
from threading import Lock
from typing import Optional

# Third-party imports
import cv2
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler

# Local imports
from dimos.utils.threadpool import get_scheduler

# Note: Logging configuration should ideally be in the application initialization,
# not in a module. Keeping it for now but with a more restricted scope.
logger = logging.getLogger(__name__)


# Specific exception classes
class VideoSourceError(Exception):
    """Raised when there's an issue with the video source."""
    pass


class VideoFrameError(Exception):
    """Raised when there's an issue with frame acquisition."""
    pass


class AbstractVideoProvider(ABC):
    """Abstract base class for video providers managing video capture resources."""

    def __init__(self,
                 dev_name: str = "NA",
                 pool_scheduler: Optional[ThreadPoolScheduler] = None) -> None:
        """Initializes the video provider with a device name.

        Args:
            dev_name: The name of the device. Defaults to "NA".
            pool_scheduler: The scheduler to use for thread pool operations.
                If None, the global scheduler from get_scheduler() will be used.
        """
        self.dev_name = dev_name
        self.pool_scheduler = (pool_scheduler
                               if pool_scheduler else get_scheduler())
        self.disposables = CompositeDisposable()

    @abstractmethod
    def capture_video_as_observable(self, fps: int = 30) -> Observable:
        """Create an observable from video capture.
        
        Args:
            fps: Frames per second to emit. Defaults to 30fps.
                
        Returns:
            Observable: An observable emitting frames at the specified rate.
            
        Raises:
            VideoSourceError: If the video source cannot be opened.
            VideoFrameError: If frames cannot be read properly.
        """
        pass

    def dispose_all(self) -> None:
        """Disposes of all active subscriptions managed by this provider."""
        if self.disposables:
            self.disposables.dispose()
        else:
            logger.info("No disposables to dispose.")

    def __del__(self) -> None:
        """Destructor to ensure resources are cleaned up if not explicitly disposed."""
        self.dispose_all()


class VideoProvider(AbstractVideoProvider):
    """Video provider implementation for capturing video as an observable."""

    def __init__(self,
                 dev_name: str,
                 video_source: str = f"{os.getcwd()}/assets/video-f30-480p.mp4",
                 pool_scheduler: Optional[ThreadPoolScheduler] = None) -> None:
        """Initializes the video provider with a device name and video source.

        Args:
            dev_name: The name of the device.
            video_source: The path to the video source. Defaults to a sample video.
            pool_scheduler: The scheduler to use for thread pool operations.
                If None, the global scheduler from get_scheduler() will be used.
        """
        super().__init__(dev_name, pool_scheduler)
        self.video_source = video_source
        self.cap = None
        self.lock = Lock()

    def _initialize_capture(self) -> None:
        """Initializes the video capture object if not already initialized.
        
        Raises:
            VideoSourceError: If the video source cannot be opened.
        """
        if self.cap is None or not self.cap.isOpened():
            # Release previous capture if it exists but is closed
            if self.cap:
                self.cap.release()
                logger.info("Released previous capture")

            # Attempt to open new capture
            self.cap = cv2.VideoCapture(self.video_source)
            if self.cap is None or not self.cap.isOpened():
                error_msg = f"Failed to open video source: {self.video_source}"
                logger.error(error_msg)
                raise VideoSourceError(error_msg)

            logger.info(f"Opened new capture: {self.video_source}")

    def capture_video_as_observable(self,
                                    realtime: bool = True,
                                    fps: int = 30) -> Observable:
        """Creates an observable from video capture.

        Creates an observable that emits frames at specified FPS or the video's 
        native FPS, with proper resource management and error handling.

        Args:
            realtime: If True, use the video's native FPS. Defaults to True.
            fps: Frames per second to emit. Defaults to 30fps. Only used if 
                realtime is False or the video's native FPS is not available.

        Returns:
            Observable: An observable emitting frames at the configured rate.
            
        Raises:
            VideoSourceError: If the video source cannot be opened.
            VideoFrameError: If frames cannot be read properly.
        """

        def emit_frames(observer, scheduler):
            try:
                self._initialize_capture()

                # Determine the FPS to use based on configuration and availability
                local_fps: float = fps
                if realtime:
                    native_fps: float = self.cap.get(cv2.CAP_PROP_FPS)
                    if native_fps > 0:
                        local_fps = native_fps
                    else:
                        logger.warning(
                            "Native FPS not available, defaulting to specified FPS"
                        )

                frame_interval: float = 1.0 / local_fps
                frame_time: float = time.monotonic()

                while self.cap.isOpened():
                    # Thread-safe access to video capture
                    with self.lock:
                        ret, frame = self.cap.read()

                    if not ret:
                        # Loop video when we reach the end
                        logger.warning(
                            "End of video reached, restarting playback")
                        with self.lock:
                            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue

                    # Control frame rate to match target FPS
                    now: float = time.monotonic()
                    next_frame_time: float = frame_time + frame_interval
                    sleep_time: float = next_frame_time - now

                    if sleep_time > 0:
                        time.sleep(sleep_time)

                    observer.on_next(frame)
                    frame_time = next_frame_time

            except VideoSourceError as e:
                logger.error(f"Video source error: {e}")
                observer.on_error(e)
            except Exception as e:
                logger.error(f"Unexpected error during frame emission: {e}")
                observer.on_error(
                    VideoFrameError(f"Frame acquisition failed: {e}"))
            finally:
                # Clean up resources regardless of success or failure
                with self.lock:
                    if self.cap and self.cap.isOpened():
                        self.cap.release()
                        logger.info("Capture released")
                observer.on_completed()

        return rx.create(emit_frames).pipe(
            ops.subscribe_on(self.pool_scheduler),
            ops.observe_on(self.pool_scheduler),
            ops.share(),  # Share the stream among multiple subscribers
        )

    def dispose_all(self) -> None:
        """Disposes of all resources including video capture."""
        with self.lock:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                logger.info("Capture released in dispose_all")
        super().dispose_all()

    def __del__(self) -> None:
        """Destructor to ensure resources are cleaned up if not explicitly disposed."""
        self.dispose_all()
