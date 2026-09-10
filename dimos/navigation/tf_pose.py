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

"""The robot's base pose off tf."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.protocol.tf.tf import TFLookup


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
