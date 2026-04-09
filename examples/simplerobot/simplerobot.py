#!/usr/bin/env python3
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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""
Simple virtual robot demonstrating a dimos Module with In/Out ports.

Subscribes to Twist commands and publishes PoseStamped.
"""

import math
import time

import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def apply_twist(pose: Pose, twist: Twist, dt: float) -> Pose:
    """Apply a velocity command to a pose (unicycle model)."""
    yaw = pose.yaw + twist.angular.z * dt
    return Pose(
        position=(
            pose.x + twist.linear.x * math.cos(yaw) * dt,
            pose.y + twist.linear.x * math.sin(yaw) * dt,
            pose.z,
        ),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


class SimpleRobotConfig(ModuleConfig):
    frame_id: str = "world"
    update_rate: float = 30.0
    cmd_timeout: float = 0.5


class SimpleRobot(Module):
    """A 2D robot that integrates velocity commands into pose."""

    config: SimpleRobotConfig
    cmd_vel: In[Twist]
    pose: Out[PoseStamped]
    _pose = Pose()
    _vel = Twist()
    _vel_time = 0.0

    @rpc
    def start(self) -> None:
        self._disposables.add(self.cmd_vel.observable().subscribe(self._on_twist))
        self._disposables.add(
            rx.interval(1.0 / self.config.update_rate).subscribe(lambda _: self._update())
        )
        self._disposables.add(
            rx.interval(1.0).subscribe(lambda _: print(f"\033[34m{self._pose}\033[0m"))
        )

    def _on_twist(self, twist: Twist) -> None:
        self._vel = twist
        self._vel_time = time.time()
        print(f"\033[32m{twist}\033[0m")

    def _update(self) -> None:
        now = time.time()
        dt = 1.0 / self.config.update_rate

        vel = self._vel if now - self._vel_time < self.config.cmd_timeout else Twist()

        self._pose = apply_twist(self._pose, vel, dt)

        self.pose.publish(
            PoseStamped(
                ts=now,
                frame_id=self.config.frame_id,
                position=self._pose.position,
                orientation=self._pose.orientation,
            )
        )


if __name__ == "__main__":
    import argparse

    from dimos.core.transport import LCMTransport

    parser = argparse.ArgumentParser(description="Simple virtual robot")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--selftest", action="store_true", help="Run demo movements")
    args = parser.parse_args()

    robot = SimpleRobot()
    robot.pose.transport = LCMTransport("/odom", PoseStamped)
    robot.cmd_vel.transport = LCMTransport("/cmd_vel", Twist)
    robot.start()

    if not args.headless:
        from vis import start_visualization

        start_visualization(robot)

    print("Robot running.")
    print("  Publishing: /odom (PoseStamped)")
    print("  Subscribing: /cmd_vel (Twist)")
    print("  Run 'lcmspy' in another terminal to see LCM messages")
    print("  Check /examples/language-interop for sending commands from LUA, C++, TS etc.")
    print("  Ctrl+C to exit")

    try:
        if args.selftest:
            time.sleep(1)
            print("Forward...")
            for _ in range(8):
                robot._on_twist(Twist(linear=(1.0, 0, 0)))
                time.sleep(0.25)
            print("Turn...")
            for _ in range(12):
                robot._on_twist(Twist(linear=(0.5, 0, 0), angular=(0, 0, 0.5)))
                time.sleep(0.25)
            print("Stop")
            robot._on_twist(Twist())
            time.sleep(1)
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        robot.stop()
