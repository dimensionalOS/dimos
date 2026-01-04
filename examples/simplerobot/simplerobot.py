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

"""
Simple virtual robot module.

Demonstrates a dimos Module that:
- Subscribes to Twist commands (velocity)
- Publishes PoseStamped (position/orientation)
"""

from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Twist, Vector3


@dataclass
class SimpleRobotConfig(ModuleConfig):
    frame_id: str = "world"
    update_rate: float = 30.0  # Hz
    cmd_timeout: float = 0.5  # seconds - stop if no command received


class SimpleRobot(Module[SimpleRobotConfig]):
    """A 2D robot that integrates velocity commands into pose."""

    # Input: velocity commands (linear.x = forward, angular.z = turn)
    cmd_vel: In[Twist]

    # Output: current pose
    pose: Out[PoseStamped]

    default_config = SimpleRobotConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = Vector3()
        self.angular_vel = Vector3()
        self._last_cmd_time = 0.0
        self._running = False
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()
        self._disposables.add(self.cmd_vel.observable().subscribe(self._on_twist))

    def _on_twist(self, twist: Twist) -> None:
        self.linear_vel = twist.linear
        self.angular_vel = twist.angular
        self._last_cmd_time = time.time()

    def _update(self) -> None:
        last = time.time()
        last_print = 0.0
        while self._running:
            now = time.time()
            dt = now - last
            last = now

            # Check command timeout
            if now - self._last_cmd_time > self.config.cmd_timeout:
                self.linear_vel = Vector3()
                self.angular_vel = Vector3()

            # Integrate velocity (unicycle model)
            self.x += self.linear_vel.x * math.cos(self.theta) * dt
            self.y += self.linear_vel.x * math.sin(self.theta) * dt
            self.theta += self.angular_vel.z * dt
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Publish pose
            self.pose.publish(
                PoseStamped(
                    ts=now,
                    frame_id=self.config.frame_id,
                    position=(self.x, self.y, 0.0),
                    orientation=(0, 0, math.sin(self.theta / 2), math.cos(self.theta / 2)),
                )
            )

            # Print status every second
            if now - last_print >= 1.0:
                print(
                    f"\033[36mPose: x={self.x:.2f} y={self.y:.2f} θ={math.degrees(self.theta):.1f}°\033[0m",
                    flush=True,
                )
                if self.linear_vel.x or self.angular_vel.z:
                    print(
                        f"\033[32mTwist: v={self.linear_vel.x:.2f} ω={math.degrees(self.angular_vel.z):.1f}°/s\033[0m",
                        flush=True,
                    )
                last_print = now

            time.sleep(1.0 / self.config.update_rate)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
        super().stop()


if __name__ == "__main__":
    import argparse

    from dimos.core import LCMTransport

    parser = argparse.ArgumentParser(description="Simple virtual robot")
    parser.add_argument("--headless", action="store_true", help="No visualization")
    parser.add_argument("--selftest", action="store_true", help="Run demo movements")
    args = parser.parse_args()

    # Create robot and set up LCM transports
    robot = SimpleRobot()
    robot.pose.transport = LCMTransport("/pose", PoseStamped)
    robot.cmd_vel.transport = LCMTransport("/cmd_vel", Twist)
    robot.start()

    # Start visualization
    if not args.headless:
        from vis import start_visualization

        start_visualization(robot)

    print("Robot running. Send Twist to /cmd_vel, receive PoseStamped on /pose")

    try:
        if args.selftest:
            import reactivex as rx
            from reactivex import operators as ops

            def send_twist(twist: Twist, duration: float) -> None:
                """Send twist at 4Hz for duration seconds."""
                rx.interval(0.25).pipe(
                    ops.take(int(duration * 4)),
                ).subscribe(lambda _: robot._on_twist(twist))
                time.sleep(duration)

            time.sleep(1)
            print(">> Forward")
            send_twist(Twist(linear=(1.0, 0, 0), angular=(0, 0, 0)), 2.0)
            print(">> Turn")
            send_twist(Twist(linear=(0.5, 0, 0), angular=(0, 0, 0.5)), 3.0)
            print(">> Stop")
            robot._on_twist(Twist())
            time.sleep(1)
            print("Self-test complete.")
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        robot.stop()
