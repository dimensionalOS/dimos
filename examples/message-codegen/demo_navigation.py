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

"""Drive the real navigation module to arrival through generated LCM/Zenoh streams."""

from contextlib import ExitStack
import math
from pathlib import Path as FilePath
import socket
from threading import Event, Lock
from typing import Any
import uuid

from demo_pubsub import free_port
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, Twist
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path
from dimos_generated.std_msgs.msg import Bool, Header
import numpy as np

from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.memory.vis.space.elements import Polyline
from dimos.memory.vis.space.space import Space
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.testing.waiting import wait_until


def demonstrate(backend: str) -> None:
    with ExitStack() as stack:
        pools = [ZenohSessionPool(), ZenohSessionPool()]
        for pool in pools:
            stack.callback(pool.close_all)
        endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
        url = f"udpm://239.255.76.67:{free_port(socket.SOCK_DGRAM)}?ttl=0"
        prefix = f"n_{uuid.uuid4().hex[:6]}"

        def transport(name: str, message_type: type, peer: int) -> Any:
            topic = f"{prefix}/{name}"
            if backend == "lcm":
                result: Any = LCMTransport(topic, message_type, url=url)
            else:
                result = ZenohTransport(
                    topic,
                    message_type,
                    session_pool=pools[peer],
                    scouting=False,
                    multicast=False,
                    gossip=False,
                    listen=[endpoint] if peer == 0 else [],
                    connect=[] if peer == 0 else [endpoint],
                )
            result.start()
            stack.callback(result.stop)
            return result

        module = ReplanningAStarPlanner()
        module._planner._local_planner._control_frequency = 100
        for name in (
            "odom",
            "odometry",
            "global_costmap",
            "goal_request",
            "clicked_point",
            "target",
            "stop_movement",
            "goal_reached",
            "navigation_state",
            "nav_cmd_vel",
            "path",
            "navigation_costmap",
        ):
            port = getattr(module, name)
            port.transport = transport(name, port.type, 0)
        odometry = transport("odometry", Odometry, 1)
        maps = transport("global_costmap", OccupancyGrid, 1)
        goals = transport("clicked_point", PointStamped, 1)
        velocities = transport("nav_cmd_vel", Twist, 1)
        arrivals = transport("goal_reached", Bool, 1)
        paths = transport("path", Path, 1)
        header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
        grid = OccupancyGrid(
            header=header,
            info=MapMetaData(
                width=24, height=24, resolution=0.25, origin=Pose(orientation=Quaternion(w=1))
            ),
            data=np.zeros(24 * 24, dtype=np.int8),
        )
        position = [1.0, 1.0, 0.0]
        lock = Lock()
        arrived = Event()
        commands: list[Twist] = []
        received_paths: list[Path] = []

        def feedback() -> None:
            message = Odometry(header=header, child_frame_id="base_link")
            message.pose.pose = Pose(
                position=Point(x=position[0], y=position[1]),
                orientation=Quaternion(z=math.sin(position[2] / 2), w=math.cos(position[2] / 2)),
            )
            odometry.publish(message)

        def command(message: Twist) -> None:
            with lock:
                commands.append(message)
                position[0] += message.linear.x * math.cos(position[2]) * 0.1
                position[1] += message.linear.x * math.sin(position[2]) * 0.1
                position[2] += message.angular.z * 0.1
                feedback()

        stack.callback(velocities.subscribe(command))
        stack.callback(arrivals.subscribe(lambda message: arrived.set() if message.data else None))
        stack.callback(paths.subscribe(received_paths.append))
        module.start()
        stack.callback(module.stop)

        def initialized() -> bool:
            maps.publish(grid)
            with lock:
                feedback()
            return (
                module._planner._current_odom is not None
                and module._planner._navigation_map._binary is not None
            )

        wait_until(initialized, timeout=5)
        goals.publish(PointStamped(header=header, point=Point(x=3, y=1)))
        assert arrived.wait(10), f"{backend}: navigation module did not report arrival"
        with lock:
            final = tuple(position)
        assert abs(final[0] - 3) < 0.25 and abs(final[1] - 1) < 0.1
        assert any(command.linear.x > 0 for command in commands)
        path = next(path for path in received_paths if len(path.poses))
        assert path.header == header and all(pose.header == header for pose in path.poses)
        wait_until(lambda: bool(commands) and commands[-1] == Twist(), timeout=5)
        output = FilePath(f"build/message-codegen/demo/evidence/navigation-{backend}.svg")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(Space().base_map(grid).add(Polyline(path)).to_svg())
        print(
            f"{backend}: arrived=({final[0]:.3f},{final[1]:.3f}), commands={len(commands)}, path={len(path.poses)} poses"
        )
        print(f"SVG: {output}; exact source timestamp=1700000000123456789 ns")
    print(f"PASS: {backend} generated odometry/map/clicked goal → navigation → Twist/Path/Bool")


if __name__ == "__main__":
    for backend in ("lcm", "zenoh"):
        demonstrate(backend)
