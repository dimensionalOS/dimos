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
import time
from typing import Optional
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer
import tf2_ros
from dimos.utils.logging_config import setup_logger
from reactivex import operators as ops, Observable
import reactivex as rx
from dimos.utils.threadpool import get_scheduler


logger = setup_logger("dimos.robot.ros_transform")

__all__ = ["ROSTransformAbility"]


class ROSTransformAbility:
    """Mixin class for handling ROS transforms between coordinate frames"""

    @property
    def tf_buffer(self) -> Buffer:
        if not hasattr(self, "_tf_buffer"):
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
            logger.info("Transform listener initialized")

        return self._tf_buffer

    def transform(
        self, child_frame: str, frequency=10, parent_frame="map", timeout=1.0
    ) -> Observable:
        # sometimes it takes time for the transform to become available
        # so we ignore errors
        def get_transform():
            try:
                return self.get_transform(child_frame, parent_frame, timeout)
            except:
                time.sleep(0.1)
                return None

        return rx.interval(1 / frequency, scheduler=get_scheduler()).pipe(
            ops.flat_map_latest(lambda _: rx.from_callable(get_transform)),
            ops.filter(lambda x: x is not None),
            ops.replay(buffer_size=1),
            ops.ref_count(),
        )

    def get_transform(
        self, child_frame: str, parent_frame: str = "map", timeout: float = 1.0
    ) -> Optional[TransformStamped]:
        try:
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
