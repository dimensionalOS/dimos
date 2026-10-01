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

"""Show generated CDR click forwarding and teleop/navigation arbitration."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, PointStamped, Twist, Vector3
from dimos_generated.std_msgs.msg import Header

from dimos.navigation.movement_manager.movement_manager import MovementManager


def main() -> None:
    manager = MovementManager(tele_cooldown_sec=10)
    goals: list[PointStamped] = []
    commands: list[Twist] = []
    unsubs = [
        manager.goal.subscribe(lambda msg: goals.append(PointStamped.decode(msg.encode()))),
        manager.cmd_vel.subscribe(lambda msg: commands.append(Twist.decode(msg.encode()))),
    ]
    try:
        clicked = PointStamped(
            header=Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789)),
            point=Point(x=3, y=4),
        )
        manager._on_click(PointStamped.decode(clicked.encode()))
        assert goals == [clicked]
        print("Click → CDR goal: (3, 4), map, 1700000000123456789 ns")
        manager._on_nav(Twist(linear=Vector3(x=0.2)))
        manager._on_teleop(Twist(linear=Vector3(x=0.5)))
        manager._on_nav(Twist(linear=Vector3(x=0.9)))
        assert [msg.linear.x for msg in commands] == [0.2, 0.5]
        print("Navigation: 0.2 m/s; teleop: 0.5 m/s; competing navigation suppressed")
        manager.config.tele_cooldown_sec = 0
        manager._on_nav(Twist(linear=Vector3(x=0.1)))
        assert commands[-1].linear.x == 0.1
        print("Cooldown elapsed → navigation resumes at 0.1 m/s")
        print("PASS: generated click and velocity messages cross CDR output boundaries")
    finally:
        for unsub in unsubs:
            unsub()
        manager.stop()


if __name__ == "__main__":
    main()
