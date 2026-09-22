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

"""Render generated CDR patrol goals selected by all three routers."""

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.memory.vis.space.space import Space
from dimos.msgs.occupancy import grid_to_world
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.utilities import point_to_pose_stamped


def main() -> None:
    np.random.seed(42)
    cells = np.zeros((24, 24), dtype=np.int8)
    cells[[0, -1], :] = 100
    cells[:, [0, -1]] = 100
    grid = OccupancyGrid(
        header=Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789)),
        info=MapMetaData(
            width=24, height=24, resolution=0.5, origin=Pose(orientation=Quaternion(w=1))
        ),
        data=cells.ravel(),
    )
    grid = OccupancyGrid.decode(grid.encode())
    for name in ("random", "coverage", "frontier"):
        router = create_patrol_router(name, 0.5)
        router.handle_occupancy_grid(grid)
        router.handle_odom(point_to_pose_stamped(grid_to_world(grid, (12, 12)), grid.header))
        space = Space().base_map(grid)
        for _ in range(5):
            goal = router.next_goal()
            assert goal is not None
            goal = PoseStamped.decode(goal.encode())
            assert goal.header == grid.header
            space.add(goal, color="#ff0000")
            router.handle_odom(goal)
            print(f"{name}: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) m")
        output = Path(f"build/message-codegen/demo/evidence/patrol-{name}.svg")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(space.to_svg())
        print(f"SVG: {output}")
    print("PASS: three patrol routers select CDR goals with exact map headers")


if __name__ == "__main__":
    main()
