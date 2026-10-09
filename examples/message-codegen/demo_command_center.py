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

"""Print command-center state and commands from generated messages, without a GUI."""

import asyncio
import json

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def main() -> None:
    module = WebsocketVisModule()
    goals: list[PoseStamped] = []
    unsubscribe = module.goal_request.subscribe(
        lambda value: goals.append(cdr_decode(value.encode(), PoseStamped))
    )
    try:
        module._create_server()
        pose = PoseStamped(
            pose=Pose(
                position=Point(x=2, y=3, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        module._on_robot_pose(cdr_decode(cdr_encode(pose), PoseStamped))
        path = Path(
            poses=[
                pose,
                PoseStamped(
                    pose=Pose(
                        position=Point(x=5, y=3, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                ),
            ],
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        module._on_path(cdr_decode(cdr_encode(path), Path))
        grid = OccupancyGrid(
            info=MapMetaData(
                width=2,
                height=2,
                resolution=1,
                origin=Pose(
                    orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                    position=Point(x=0.0, y=0.0, z=0.0),
                ),
                map_load_time=Time(sec=0, nanosec=0),
            ),
            data=np.array([100, 0, 0, -1], dtype=np.int8),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        module._on_global_costmap(cdr_decode(cdr_encode(grid), OccupancyGrid))
        assert module.sio is not None
        asyncio.run(module.sio.handlers["/"]["click"]("demo", [5, 3]))
        assert goals[0].pose.position == Point(x=5, y=3, z=0.0)
        assert goals[0].header.frame_id == "world"
        print("Command-center state from CDR pose, path, and costmap:")
        print(json.dumps(module.vis_state, indent=2))
        print("Command-center click → generated CDR goal: world (5, 3, 0)")
        print("Handlers exercised directly; no browser or network server was opened.")
    finally:
        unsubscribe()
        module.stop()


if __name__ == "__main__":
    main()
