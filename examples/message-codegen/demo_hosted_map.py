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

"""Save the hosted operator's PNG and odometry payload from generated CDR inputs."""

import base64
import json
from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.msgs.geometry import quaternion_from_euler
from dimos.teleop.hosted.map_compress import MapCompressModule


def main() -> None:
    module = MapCompressModule()
    payloads = []
    unsubscribe = module.map_out.subscribe(lambda data: payloads.append(json.loads(data)))
    try:
        cells = np.zeros((64, 64), dtype=np.int8)
        cells[:8] = -1
        cells[20:44, 20:44] = 50
        cells[28:36, 28:36] = 100
        grid = OccupancyGrid(
            info=MapMetaData(
                width=64,
                height=64,
                resolution=0.1,
                map_load_time=Time(sec=0, nanosec=0),
                origin=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            data=np.asarray(cells.ravel(), dtype=np.int8),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        module._on_costmap(cdr_decode(cdr_encode(grid), OccupancyGrid))
        pose = PoseStamped(
            pose=Pose(
                position=Point(x=3.2, y=3.2, z=0.0), orientation=quaternion_from_euler(0, 0, 1.0)
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        module._on_odom(cdr_decode(cdr_encode(pose), PoseStamped))
        assert [value["type"] for value in payloads] == ["map", "odom"]
        assert payloads[1]["yaw"] == 1.0
        destination = Path("build/message-codegen/demo/evidence/hosted-map.png")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(base64.b64decode(payloads[0]["png_b64"]))
        print(f"CDR occupancy → hosted PNG: {destination}")
        print("64 x 64: transparent unknown strip, cyan obstacle square, white lethal center")
        print(f"CDR pose → operator marker: {payloads[1]}")
        print("Local callbacks only; no hosted connection or hardware required.")
    finally:
        unsubscribe()
        module.stop()


if __name__ == "__main__":
    main()
