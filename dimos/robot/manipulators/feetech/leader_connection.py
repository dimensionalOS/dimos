# Copyright 2025-2026 Dimensional Inc.
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

"""Feetech leader arms that a follow switch can pause."""

from __future__ import annotations

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.manipulators.feetech.connection import (
    GUARDED_JOINT_FRAME_ID,
    FeetechConnection,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FeetechLeaderConnection(FeetechConnection):
    """Leader arms whose targets can be paused, e.g. from a Joy-Con button.

    While ``follow`` is off, nothing is sent and the follower holds still, so
    the leaders can be moved freely.
    """

    # Off pauses the leaders; on (the start state) sends their targets again.
    follow: In[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._following = True
        self._follow_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.follow.subscribe(self._on_follow)))

    def publish_reading(self, readings: dict[str, dict[str, float]]) -> None:
        with self._follow_lock:
            if self._following:
                super().publish_reading(readings)

    def _on_follow(self, msg: Bool) -> None:
        with self._follow_lock:
            if msg.data == self._following:
                return
            self._following = msg.data
            if not msg.data:
                # A jog with no positions and zero speed tells the trajectory
                # task to hold every joint now, not 0.4 s after the last target.
                joints = [
                    f"{self.config.source}/{motor.joint}"
                    for bus in self.robot.buses
                    for motor in bus.motors
                ]
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=GUARDED_JOINT_FRAME_ID,
                        name=joints,
                        velocity=[0.0] * len(joints),
                    )
                )
        logger.info("%s %s", type(self).__name__, "resumed" if msg.data else "paused")
