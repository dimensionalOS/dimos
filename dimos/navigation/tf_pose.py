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

"""Resolve odometry into base-frame poses using the static mount tf."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()


def base_height_above_ground(lidar_height: float, base_to_sensor: Transform) -> float:
    """Height of the base frame origin above the ground while standing.

    The lidar sits ``lidar_height`` above the ground and
    ``base_to_sensor.translation.z`` above the base origin.
    """
    return lidar_height - base_to_sensor.translation.z


class OdomBasePose:
    """Turn odometry messages into the base-frame pose they imply.

    The odometry message itself carries the live parent -> child edge, so only the
    static leg from the message's child frame to the base frame is looked up on tf,
    once, then reused for every following message.
    """

    # While the leg is missing, retry the lookup at most this often. The buffer
    # warns on every miss, so per-message retries would flood the log.
    RETRY_PERIOD_S = 1.0

    def __init__(self, tf: TFLookup, base_frame: str) -> None:
        self._tf = tf
        self.base_frame = base_frame
        self._legs: dict[str, Transform] = {}
        self._waiting = False
        self._next_lookup = 0.0

    def resolve(self, msg: Odometry) -> PoseStamped | None:
        """The base pose for one message. None until tf has the mount leg."""
        if msg.child_frame_id == self.base_frame:
            return msg.to_pose_stamped()
        leg = self.sensor_to_base(msg.child_frame_id)
        if leg is None:
            return None
        odom = Transform.from_pose(msg.child_frame_id, msg.to_pose_stamped())
        return (odom + leg).to_pose(ts=msg.ts)

    def sensor_to_base(self, sensor_frame: str) -> Transform | None:
        """The cached static sensor -> base leg. Logs once per outage, not per message."""
        if sensor_frame == self.base_frame:
            return Transform.identity()
        leg = self._legs.get(sensor_frame)
        if leg is None:
            if self._waiting and time.monotonic() < self._next_lookup:
                return None
            leg = self._tf.get(sensor_frame, self.base_frame)
            if leg is None:
                self._next_lookup = time.monotonic() + self.RETRY_PERIOD_S
                if not self._waiting:
                    self._waiting = True
                    logger.warning(
                        "No %s -> %s transform on tf yet, dropping odometry until it arrives.",
                        sensor_frame,
                        self.base_frame,
                    )
                return None
            if self._waiting:
                self._waiting = False
                logger.info("Got the %s -> %s transform, resuming.", sensor_frame, self.base_frame)
            self._legs[sensor_frame] = leg
        return leg
