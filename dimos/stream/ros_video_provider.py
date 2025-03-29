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

"""ROS-based video provider module.

This module provides a video frame provider that receives frames from ROS (Robot Operating System)
and makes them available as an Observable stream.
"""

from reactivex import Subject, Observable
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler
import logging
import time
from typing import Any, Optional
import numpy as np

from dimos.utils.threadpool import get_scheduler
from dimos.stream.video_provider import AbstractVideoProvider

logging.basicConfig(level=logging.INFO)


class ROSVideoProvider(AbstractVideoProvider):
    """Video provider that uses a Subject to broadcast frames pushed by ROS.
    
    This class implements a video provider that receives frames from ROS and makes them
    available as an Observable stream. It uses ReactiveX's Subject to broadcast frames.
    
    Attributes:
        logger: Logger instance for this provider.
        _subject: ReactiveX Subject that broadcasts frames.
        _last_frame_time: Timestamp of the last received frame.
    """

    def __init__(self,
                 dev_name: str = "ros_video",
                 pool_scheduler: Optional[ThreadPoolScheduler] = None):
        """Initialize the ROS video provider.
        
        Args:
            dev_name: A string identifying this provider.
            pool_scheduler: Optional ThreadPoolScheduler for multithreading.
        """
        super().__init__(dev_name, pool_scheduler)
        self.logger = logging.getLogger(dev_name)
        self._subject = Subject()
        self._last_frame_time = None
        self.logger.info("ROSVideoProvider initialized")

    def push_data(self, frame: np.ndarray) -> None:
        """Push a new frame into the provider.
        
        Args:
            frame: The video frame to push into the stream, typically a numpy array
                containing image data.
            
        Raises:
            Exception: If there's an error pushing the frame.
        """
        try:
            current_time = time.time()
            if self._last_frame_time:
                frame_interval = current_time - self._last_frame_time
                self.logger.debug(
                    f"Frame interval: {frame_interval:.3f}s ({1/frame_interval:.1f} FPS)"
                )
            self._last_frame_time = current_time

            self.logger.debug(f"Pushing frame type: {type(frame)}")
            self._subject.on_next(frame)
            self.logger.debug("Frame pushed")
        except Exception as e:
            self.logger.error(f"Push error: {e}")
            raise

    def capture_video_as_observable(self, fps: int = 30) -> Observable:
        """Return an observable of video frames.
        
        Args:
            fps: Frames per second rate limit (default: 30; ignored for now).
            
        Returns:
            Observable: An observable stream of video frames (numpy.ndarray objects),
                with each emission containing a single video frame. The frames are
                multicast to all subscribers.
                
        Note:
            The fps parameter is currently not enforced. See implementation note below.
        """
        self.logger.info(f"Creating observable with {fps} FPS rate limiting")
        # TODO: Implement rate limiting using ops.throttle_with_timeout() or
        # ops.sample() to restrict emissions to one frame per (1/fps) seconds.
        # Example: ops.sample(1.0/fps)
        return self._subject.pipe(
            # Ensure subscription work happens on the thread pool
            ops.subscribe_on(self.pool_scheduler),
            # Ensure observer callbacks execute on the thread pool
            ops.observe_on(self.pool_scheduler),
            # Make the stream hot/multicast so multiple subscribers get the same frames
            ops.share(),
        )
