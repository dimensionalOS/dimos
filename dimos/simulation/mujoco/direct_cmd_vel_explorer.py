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

import math
import threading
from typing import TYPE_CHECKING

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from collections.abc import Callable


class DirectCmdVelExplorer:
    def __init__(
        self,
        linear_speed: float = 0.8,
        rotation_speed: float = 1.5,
        publish_rate: float = 10.0,
    ) -> None:
        self.linear_speed = linear_speed
        self.rotation_speed = rotation_speed
        self._dt = 1.0 / publish_rate
        self._cmd_vel: LCMTransport[Twist] | None = None
        self._odom: LCMTransport[PoseStamped] | None = None
        self._pose: PoseStamped | None = None
        self._new_pose = threading.Event()
        self._unsub: Callable[[], None] | None = None

    def start(self) -> None:
        self._cmd_vel = LCMTransport("/cmd_vel", Twist)
        self._odom = LCMTransport("/odom", PoseStamped)
        self._pose = None
        self._unsub = self._odom.subscribe(self._on_odom)

    def stop(self) -> None:
        if self._unsub:
            self._unsub()
        if self._cmd_vel:
            self._cmd_vel.stop()
        if self._odom:
            self._odom.stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        self._pose = msg
        self._new_pose.set()

    def _wait_for_pose(self) -> PoseStamped:
        self._new_pose.clear()
        self._new_pose.wait(timeout=5.0)
        assert self._pose is not None, "No odom received"
        return self._pose

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _stop(self) -> None:
        assert self._cmd_vel is not None
        self._cmd_vel.broadcast(None, Twist(linear=Vector3(), angular=Vector3()))

    def _drive_to(self, target_x: float, target_y: float) -> None:
        """Pursuit controller: steer toward the target while driving forward."""
        while True:
            pose = self._wait_for_pose()
            dx = target_x - pose.x
            dy = target_y - pose.y
            distance = math.hypot(dx, dy)
            if distance < 0.3:
                break
            target_heading = math.atan2(dy, dx)
            heading_error = self._normalize_angle(target_heading - pose.yaw)
            # Only drive forward when roughly facing the target.
            if abs(heading_error) > 0.3:
                linear = 0.0
            else:
                linear = self.linear_speed
            angular = max(-self.rotation_speed, min(self.rotation_speed, heading_error * 2.0))
            assert self._cmd_vel is not None
            self._cmd_vel.broadcast(
                None,
                Twist(linear=Vector3(linear, 0, 0), angular=Vector3(0, 0, angular)),
            )
        self._stop()

    def follow_points(self, waypoints: list[tuple[float, float]]) -> None:
        self._wait_for_pose()
        for tx, ty in waypoints:
            self._drive_to(tx, ty)
