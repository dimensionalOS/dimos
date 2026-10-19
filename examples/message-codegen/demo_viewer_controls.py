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

"""Exercise viewer click, velocity, and stop events without opening a GUI."""

import asyncio
import json
from threading import Event

from dimos_generated.geometry_msgs.msg import PointStamped, Twist
import websockets.asyncio.client as ws_client

from dimos.core.global_config import GlobalConfig
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def main() -> None:
    server = RerunWebSocketServer(
        g=GlobalConfig(rerun_host="127.0.0.1", rerun_websocket_server_port=0)
    )
    points: list[PointStamped] = []
    velocities: list[Twist] = []
    stopped = Event()

    def receive_velocity(message: Twist) -> None:
        velocities.append(Twist.decode(message.encode()))
        if len(velocities) == 2:
            stopped.set()

    subscriptions = [
        server.clicked_point.subscribe(
            lambda message: points.append(PointStamped.decode(message.encode()))
        ),
        server.tele_cmd_vel.subscribe(receive_velocity),
    ]

    async def send_events() -> None:
        async with ws_client.connect(f"ws://127.0.0.1:{server.bound_port}/ws") as ws:
            for event in [
                {
                    "type": "click",
                    "x": 1.5,
                    "y": 2.5,
                    "z": None,
                    "entity_path": "map",
                    "timestamp_ms": 1700000000123,
                },
                {"type": "twist", "linear_x": 0.5, "angular_z": 0.8},
                {"type": "stop"},
            ]:
                await ws.send(json.dumps(event))

    try:
        server.start()
        asyncio.run(send_events())
        assert stopped.wait(3), "Viewer stop command was not delivered"
        assert len(points) == 1
        point = points[0]
        assert (point.point.x, point.point.y, point.point.z) == (1.5, 2.5, 0)
        assert point.header.frame_id == "map"
        assert (point.header.stamp.sec, point.header.stamp.nanosec) == (1700000000, 123000000)
        assert velocities[0].linear.x == 0.5 and velocities[0].angular.z == 0.8
        assert velocities[1] == Twist()
        print("Viewer click → CDR PointStamped: map (1.5, 2.5, 0), stamp 1700000000.123000000")
        print("Viewer velocity → CDR Twist: forward 0.5 m/s, yaw 0.8 rad/s")
        print("Viewer stop → CDR Twist: all six components zero")
    finally:
        for unsubscribe in subscriptions:
            unsubscribe()
        server.stop()


if __name__ == "__main__":
    main()
