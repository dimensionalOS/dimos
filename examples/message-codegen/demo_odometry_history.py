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

"""Follow a CDR path and inspect generated odometry history without hardware."""

import asyncio
from pathlib import Path as FilePath

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from dimos_generated.nav_msgs.msg import Odometry, Path
from dimos_generated.std_msgs.msg import Bool, Header

from dimos.mapping.odometry_hist import OdometryHist, path_at_true_height
from dimos.memory.vis.space.elements import Polyline
from dimos.memory.vis.space.space import Space
from dimos.navigation.basic_path_follower.module import BasicPathFollower


async def main() -> None:
    follower = BasicPathFollower()
    history = OdometryHist(min_publish_interval_seconds=0)
    commands: list[Twist] = []
    arrivals: list[Bool] = []
    paths: list[Path] = []
    follower.nav_cmd_vel.subscribe(lambda msg: commands.append(Twist.decode(msg.encode())))
    follower.goal_reached.subscribe(lambda msg: arrivals.append(Bool.decode(msg.encode())))
    history.odom_hist.subscribe(lambda msg: paths.append(Path.decode(msg.encode())))
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    reference = Path(
        header=header,
        poses=[
            PoseStamped(header=header, pose=Pose(position=Point(x=x), orientation=Quaternion(w=1)))
            for x in [0, 1, 2]
        ],
    )
    try:
        follower._on_path(Path.decode(reference.encode()))
        waypoints = follower._waypoints
        assert waypoints is not None
        x = 0.0
        for tick in range(100):
            header.stamp.nanosec = 123456789 + tick
            pose = PoseStamped(
                header=header, pose=Pose(position=Point(x=x, z=0.25), orientation=Quaternion(w=1))
            )
            sample = Odometry(header=header)
            sample.pose.pose = pose.pose
            await history.handle_odometry(Odometry.decode(sample.encode()))
            follower._step(PoseStamped.decode(pose.encode()), waypoints)
            if arrivals:
                break
            x += commands[-1].linear.x * 0.1
        assert arrivals and arrivals[-1].data and commands[-1] == Twist()
        assert x >= 1.7 and x <= 2
        recorded = paths[-1]
        assert recorded.poses[0].header.stamp.nanosec == 123456789
        assert recorded.header.stamp == header.stamp
        strips = path_at_true_height(recorded).strips.as_arrow_array().to_pylist()
        assert all(point[2] == 0.25 for point in strips[0])
        output = FilePath("build/message-codegen/demo/evidence/odometry-history.svg")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            Space().add(Polyline(reference)).add(Polyline(recorded, color="#ff0000")).to_svg()
        )
        print(
            f"Arrived at x={x:.3f}; {len(recorded.poses)} generated history poses; final Twist is zero"
        )
        print(
            f"Source stamps: 1700000000123456789..{1700000000000000000 + recorded.header.stamp.nanosec} ns"
        )
        print(f"SVG reference/history: {output}; Rerun geometry retains z=0.25 m")
    finally:
        follower.stop()
        history.stop()
    print("PASS: generated Path → basic follower → Twist/Bool; Odometry → exact stamped history")


if __name__ == "__main__":
    asyncio.run(main())
