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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Plan on a generated CDR map in Python and C++, then render the resampled path."""

from pathlib import Path as FilePath

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.occupancy.path_mask import make_path_mask
from dimos.mapping.occupancy.path_resampling import simple_resample_path
from dimos.memory.vis.space.elements import Polyline
from dimos.memory.vis.space.space import Space
from dimos.msgs.occupancy import grid_to_world
from dimos.navigation.replanning_a_star.min_cost_astar import _USE_CPP, min_cost_astar


def main() -> None:
    if not _USE_CPP:
        raise RuntimeError("Build the native A* extension before running this demo")
    cells = np.zeros((40, 40), dtype=np.int8)
    cells[20, :30] = 100
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    grid = OccupancyGrid(
        header=header,
        info=MapMetaData(
            width=40, height=40, resolution=0.25, origin=Pose(orientation=Quaternion(w=1))
        ),
        data=cells.ravel(),
    )
    received = OccupancyGrid.decode(grid.encode())
    start = grid_to_world(received, (5, 5))
    goal = grid_to_world(received, (35, 35))
    output_dir = FilePath("build/message-codegen/demo/evidence")
    output_dir.mkdir(parents=True, exist_ok=True)
    for use_cpp in (False, True):
        name = "cpp" if use_cpp else "python"
        path = min_cost_astar(received, goal, start, use_cpp=use_cpp)
        assert path is not None
        resampled = simple_resample_path(path, Pose(orientation=Quaternion(w=1)), 0.1)
        decoded = Path.decode(resampled.encode())
        assert decoded.header == header
        assert all(pose.header == header for pose in decoded.poses)
        mask = make_path_mask(received, decoded, 0.1)
        assert mask.any() and not np.any(mask & (cells == 100))
        output = output_dir / f"grid-path-{name}.svg"
        output.write_text(Space().base_map(received).add(Polyline(decoded)).to_svg())
        print(
            f"{name}: {len(path.poses)} planned points → {len(decoded.poses)} CDR poses; SVG={output}"
        )
    print("PASS: both planners route around the wall; source nanoseconds survive resampling/CDR")


if __name__ == "__main__":
    main()
