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

"""The robot's base pose off tf: live (`TfPose`) or resolved from odometry (`OdomBasePose`)."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()


def base_height_above_ground(lidar_height: float, base_to_sensor: Transform) -> float:
    """Height of the base frame origin above the ground while standing."""
    return lidar_height - base_to_sensor.translation.z


class OdomBasePose:
    """Turn odometry messages into the base-frame pose they imply."""

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


class TfPose:
    """The latest ``world -> base`` pose off tf; None while missing or once its stamp stops advancing."""

    # While the edge is missing, retry the lookup at most this often: the
    # buffer warns on every miss, and a tick loop would flood the log.
    RETRY_PERIOD_S = 1.0

    def __init__(
        self,
        tf: TFLookup,
        base_frame: str,
        max_age_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._tf = tf
        self.base_frame = base_frame
        self.max_age_s = max_age_s
        self._clock = clock
        self._next_lookup = 0.0
        # The stamp last seen and when it was first seen, on OUR clock: the
        # edge's stamp is the robot's clock, which need not be ours, and what
        # the deadman asks is how long since tf last moved.
        self._seen: tuple[float, float] | None = None

    def get(self, world_frame: str) -> PoseStamped | None:
        now = self._clock()
        if now < self._next_lookup:
            return None
        tf = self._tf.get(world_frame, self.base_frame)
        if tf is None:
            self._next_lookup = now + self.RETRY_PERIOD_S
            self._seen = None
            return None
        if self._seen is None or self._seen[0] != tf.ts:
            self._seen = (tf.ts, now)
        if now - self._seen[1] > self.max_age_s:
            return None
        return tf.to_pose(ts=tf.ts)
