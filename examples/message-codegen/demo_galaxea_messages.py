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

"""Run Galaxea wheel feedback through generated output streams without ROS or hardware."""

from contextlib import ExitStack
import math
import threading
from typing import Any

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.time import to_nanoseconds
from dimos.robot.galaxea.r1pro.connection import R1ProConnection


def main() -> None:
    robot = R1ProConnection()
    outputs: dict[str, list[Any]] = {name: [] for name in ("odom", "odometry", "tf")}
    ready = threading.Event()
    lock = threading.Lock()
    with ExitStack() as stack:
        stack.callback(robot.stop)

        def receive(name: str, message: Any) -> None:
            with lock:
                outputs[name].append(message)
                if all(len(values) == 3 for values in outputs.values()):
                    ready.set()

        for name in outputs:
            unsubscribe = getattr(robot, name).subscribe(
                lambda message, name=name: receive(name, message)
            )
            stack.callback(unsubscribe)
        for index in range(4):
            robot._on_chassis_speed(
                TwistStamped(
                    header=Header(
                        stamp=Time(sec=1700000000, nanosec=123456789 + index * 100000000)
                    ),
                    twist=Twist(linear=Vector3(x=1.0), angular=Vector3(z=0.2)),
                ),
                None,
            )
        assert ready.wait(5), "Generated Galaxea outputs did not reach subscribers"
        for index, (pose, odometry, tf) in enumerate(zip(*outputs.values(), strict=True), start=1):
            stamp = 1700000000123456789 + index * 100000000
            assert to_nanoseconds(pose.header.stamp) == stamp
            assert odometry.pose.pose == pose.pose
            assert all(to_nanoseconds(edge.header.stamp) == stamp for edge in tf.transforms)
            assert math.isclose(pose.pose.orientation.z, math.sin(index * 0.01))
            for message in (pose, odometry, tf):
                assert type(message).decode(message.encode()) == message
            print(
                f"tick {index}: x={pose.pose.position.x:.6f}, y={pose.pose.position.y:.6f}, stamp={stamp}"
            )
            print(
                "  TF: "
                + " -> ".join(
                    [tf.transforms[0].header.frame_id]
                    + [edge.child_frame_id for edge in tf.transforms]
                )
            )
    print("PASS: Galaxea generated pose, odometry, and TF streams preserve exact source time")


if __name__ == "__main__":
    main()
