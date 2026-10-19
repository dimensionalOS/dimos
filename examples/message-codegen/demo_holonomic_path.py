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

"""Run the threaded holonomic path follower with a synthetic CDR robot."""

import math
import time

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTCConfig, _HolonomicPathFollower
from dimos.navigation.dannav.local_planner.module import DanLocalPlannerConfig, _ReplanGate


def main() -> None:
    core = _HolonomicPathFollower(
        DanHolonomicTCConfig(speed_m_s=1.2, control_frequency=60.0, goal_tolerance=0.08)
    )
    command = Twist()
    stopped: list[str] = []

    def receive(value: Twist) -> None:
        nonlocal command
        command = Twist.decode(value.encode())

    subscription = core.cmd_vel.subscribe(receive)
    stop_subscription = core.stopped_navigating.subscribe(stopped.append)
    x = y = yaw = 0.0
    dt = 1 / 60

    def odometry(tick: int) -> PoseStamped:
        value = PoseStamped(
            header=Header(
                frame_id="map",
                stamp=time_from_nanoseconds(1700000000000000000 + round(tick * dt * 1e9)),
            ),
            pose=Pose(
                position=Point(x=x, y=y),
                orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        )
        return PoseStamped.decode(value.encode())

    path = Path(
        header=Header(frame_id="map"),
        poses=[
            PoseStamped(pose=Pose(position=Point(x=px), orientation=Quaternion(w=1.0)))
            for px in (0.1, 1.0)
        ],
    )
    try:
        gate = _ReplanGate(
            DanLocalPlannerConfig(lock_replan=0.5, resample_spacing_m=0.1, smoothing_window=3)
        )
        gate.on_odom(odometry(0))
        committed = gate.on_planner_path(Path.decode(path.encode()))
        assert committed is not None
        assert gate.on_planner_path(Path.decode(path.encode())) is None
        print(
            f"Local planner committed {len(committed.poses)} resampled poses; duplicate replan held"
        )
        core.handle_odom(odometry(0))
        core.start_planning(Path.decode(committed.encode()))
        for tick in range(300):
            if stopped:
                break
            time.sleep(dt * 1.1)
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * command.linear.x - s * command.linear.y) * dt
            y += (s * command.linear.x + c * command.linear.y) * dt
            yaw += command.angular.z * dt
            core.handle_odom(odometry(tick + 1))
            if tick % 30 == 0:
                print(
                    f"position=({x:.3f}, {y:.3f}) decoded speed={math.hypot(command.linear.x, command.linear.y):.3f}m/s"
                )
        assert "arrived" in stopped, stopped
        assert math.hypot(1 - x, y) < 0.15
        print(
            f"Path follower arrived at ({x:.3f}, {y:.3f}); final command={command.linear.x:.3f}m/s"
        )
    finally:
        core.close()
        subscription.dispose()
        stop_subscription.dispose()


if __name__ == "__main__":
    main()
