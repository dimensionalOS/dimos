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

"""OpenArm Mini connection settings: two serial ports for the two leader arms."""

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
    FeetechConnectionConfig,
)
from dimos.robot.manipulators.openarm.config import (
    OPENARM_MINI_ARM_LEADERS,
    OPENARM_MINI_LEADERS,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OpenArmMiniConnectionConfig(FeetechConnectionConfig):
    """Serial ports of the two OpenArm Mini leader arms.

    Attributes:
        left_serial_port: The left leader's serial port, e.g. "/dev/ttyACM0";
            empty leaves that leader unread.
        right_serial_port: The right leader's serial port; empty leaves it unread.
    """

    left_serial_port: str = ""
    right_serial_port: str = ""


class OpenArmMiniConnection(FeetechConnection):
    """The two OpenArm Mini leader arms; the serial ports given select which are read.

    While ``follow`` is off, nothing is sent and the follower holds still, so
    the leaders can be moved freely.
    """

    config: OpenArmMiniConnectionConfig
    robot = OPENARM_MINI_LEADERS
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

    def serial_ports(self) -> dict[str, str]:
        return {"left": self.config.left_serial_port, "right": self.config.right_serial_port}

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
        logger.info("OpenArm Mini leaders %s", "resumed" if msg.data else "paused")


class OpenArmMiniArmConnection(OpenArmMiniConnection):
    """The two leader arms without their trigger servos; the jaws are driven elsewhere.

    Each side's calibration file must list only the seven arm motors.
    """

    robot = OPENARM_MINI_ARM_LEADERS
