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

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path

from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def main() -> None:
    module = WebsocketVisModule()
    goals: list[PoseStamped] = []
    unsubscribe = module.goal_request.subscribe(
        lambda value: goals.append(PoseStamped.decode(value.encode()))
    )
    try:
        module._create_server()
        pose = PoseStamped(pose=Pose(position=Point(x=2, y=3), orientation=Quaternion(w=1)))
        module._on_robot_pose(PoseStamped.decode(pose.encode()))
        path = Path(poses=[pose, PoseStamped(pose=Pose(position=Point(x=5, y=3)))])
        module._on_path(Path.decode(path.encode()))
        grid = OccupancyGrid(
            info=MapMetaData(
                width=2, height=2, resolution=1, origin=Pose(orientation=Quaternion(w=1))
            ),
            data=[100, 0, 0, -1],
        )
        module._on_global_costmap(OccupancyGrid.decode(grid.encode()))
        assert module.sio is not None
        asyncio.run(module.sio.handlers["/"]["click"]("demo", [5, 3]))
        assert goals[0].pose.position == Point(x=5, y=3)
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
