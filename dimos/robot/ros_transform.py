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
import rclpy
from rclpy.node import Node
from typing import Optional
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros
from dimos.utils.logging_config import setup_logger
from reactivex import Observable, create
from reactivex.disposable import Disposable
import reactivex as rx
from reactivex import operators as ops, from_callable
from reactivex import Observable
from dimos.utils.threadpool import get_scheduler


logger = setup_logger("dimos.robot.ros_transform")

__all__ = ["ROSTransform"]


class ROSTransformAbility:
    """Base class for handling ROS transforms between coordinate frames"""

    def get_transform_stream(
        self,
        child_frame: str,
        parent_frame: str = "base_link",
        timeout: float = 1.0,
        rate_hz: float = 1.0  # Default to 1 Hz
    ) -> Observable:
        """
        Creates an Observable stream of transforms between coordinate frames.
        
        This can function in two modes:
        1. If rate_hz is provided (default is 1 Hz), returns a timer-based stream
        2. If rate_hz is None, returns a factory function for on-demand transforms

        Args:
            child_frame: Child/target coordinate frame
            parent_frame: Parent/source coordinate frame
            timeout: How long to wait for the transform to become available (seconds)
            rate_hz: Rate at which to emit transform values (default: 1 Hz)

        Returns:
            Observable: A stream of TransformStamped messages
        """
        from reactivex import interval, defer, create
        from reactivex import operators as ops
        import time
        import threading
        
        logger.info(f"Creating transform stream from {parent_frame} to {child_frame} at {rate_hz} Hz")
        
        # If rate_hz is None, create an on-demand transform observable
        if rate_hz is None:
            def lookup_on_subscribe(observer, scheduler):
                transform = self.get_transform(parent_frame, child_frame, timeout)
                if transform is not None:
                    observer.on_next(transform)
                return lambda: None  # Empty disposable
                
            return create(lookup_on_subscribe)
        
        # For timer-based streaming, we'll use a more robust approach
        def emit_transforms(observer, scheduler):
            # Create and start a daemon thread to periodically emit transforms
            stop_event = threading.Event()
            
            def transform_thread():
                period = 1.0 / rate_hz  # Period in seconds
                
                while not stop_event.is_set():
                    try:
                        # Get the transform
                        transform = self.get_transform(parent_frame, child_frame, timeout)
                        
                        # Only emit if transform was found
                        if transform is not None:
                            observer.on_next(transform)
                        else:
                            logger.debug(f"No transform found from {parent_frame} to {child_frame}")
                            
                        # Sleep for the remainder of the period
                        time.sleep(period)
                    except Exception as e:
                        logger.error(f"Error in transform thread: {e}")
                        # Don't pass the error to the observer - just log it and continue
                        time.sleep(period)  # Sleep to avoid tight loop
            
            # Start the thread
            thread = threading.Thread(target=transform_thread, daemon=True)
            thread.start()
            
            # Return a disposable that stops the thread
            def dispose():
                logger.info(f"Disposing transform stream from {parent_frame} to {child_frame}")
                stop_event.set()
                
            return dispose
            
        return create(emit_transforms).pipe(ops.share())

    def get_transform(
        self, child_frame: str, parent_frame: str = "base_link", timeout: float = 1.0
    ) -> Optional[TransformStamped]:
        """
        Read transform data between two coordinate frames

        Args:
            child_frame: Child/target coordinate frame
            parent_frame: Parent/source coordinate frame
            timeout: How long to wait for the transform to become available (seconds)

        Returns:
            TransformStamped: The transform data or None if not available
        """
        try:
            # Look up transform
            transform = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout),
            )
            return transform
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            logger.error(f"Transform lookup failed: {e}")
            return None


class ROSTransformRXAbility:
    """Mixin class for handling ROS transforms between coordinate frames"""

    @property
    def tf_buffer(self) -> Buffer:
        if not hasattr(self, "_tf_buffer"):
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
            logger.info(f"Transform listener initialized")

        return self._tf_buffer

    def transform(
        self, child_frame: str, frequency=10, parent_frame="map", timeout=1.0
    ) -> Observable:

        def get_transform():
            return self.get_transform(child_frame, parent_frame, timeout)

        return rx.interval(1 / frequency, scheduler=get_scheduler()).pipe(
            ops.flat_map(lambda _: rx.from_callable(get_transform)),
            ops.replay(buffer_size=1),
            ops.ref_count(),
        )

    def get_transform(
        self, child_frame: str, parent_frame: str = "map", timeout: float = 1.0
    ) -> Optional[TransformStamped]:
        try:
            # Look up transform
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout),
            )
            return transform
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            logger.error(f"Transform lookup failed: {e}")
            return None
