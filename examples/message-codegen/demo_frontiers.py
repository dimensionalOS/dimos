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

"""Inspect frontier selection on a generated CDR occupancy map."""

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.memory.vis.space.space import Space
from dimos.msgs.occupancy import grid_to_world
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)


def main() -> None:
    cells = np.full((20, 20), -1, dtype=np.int8)
    cells[:, :10] = 0
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    grid = OccupancyGrid(
        header=header,
        info=MapMetaData(
            width=20, height=20, resolution=0.5, origin=Pose(orientation=Quaternion(w=1))
        ),
        data=cells.ravel(),
    )
    explorer = WavefrontFrontierExplorer()
    try:
        received = OccupancyGrid.decode(grid.encode())
        frontiers = explorer.detect_frontiers(grid_to_world(received, (5, 10)), received)
        assert len(frontiers) == 1
        goal = PoseStamped(
            header=received.header, pose=Pose(position=frontiers[0], orientation=Quaternion(w=1))
        )
        decoded = PoseStamped.decode(goal.encode())
        assert decoded.header == header
        assert decoded.pose.position.x == 5 and decoded.pose.position.y == 4.75
        output = Path("build/message-codegen/demo/evidence/frontiers.svg")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(Space().base_map(received).add(decoded, color="#ff0000").to_svg())
        print(
            f"Frontier centroid: ({decoded.pose.position.x:.2f}, {decoded.pose.position.y:.2f}) m"
        )
        print(f"SVG: {output}; stamp=1700000000123456789 ns")
        print("PASS: generated map → frontier centroid → CDR goal with exact source header")
    finally:
        explorer.stop()


if __name__ == "__main__":
    main()
