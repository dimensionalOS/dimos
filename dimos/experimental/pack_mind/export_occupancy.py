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

"""Export the PACK MIND maze as a .npy occupancy grid so MuJoCo can build the room.

    uv run python -m dimos.experimental.pack_mind.export_occupancy --out pack_mind_maze.npy
    # then drive a Go2 through the photoreal 3D version of our maze:
    dimos --simulation run unitree-go2 --mujoco-room-from-occupancy pack_mind_maze.npy

`OccupancyGrid.from_path` loads .npy via np.load; `generate_mujoco_scene`
(dimos/mapping/occupancy/extrude_occupancy.py) extrudes occupied cells into 3D walls.
Our grid uses FREE=0 / OCCUPIED=100 / UNKNOWN=-1 (standard nav_msgs convention).
"""

from __future__ import annotations

import argparse

import numpy as np

from dimos.experimental.pack_mind.world import make_maze_world
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues


def main() -> None:
    p = argparse.ArgumentParser(description="Export PACK MIND maze to .npy for MuJoCo")
    p.add_argument("--out", default="pack_mind_maze.npy")
    args = p.parse_args()

    grid = make_maze_world().grid
    np.save(args.out, grid)
    walls = int(np.count_nonzero(grid == CostValues.OCCUPIED))
    free = int(np.count_nonzero(grid == CostValues.FREE))
    print(f"saved {grid.shape} occupancy -> {args.out}  (walls={walls}, free={free})")
    print("next:  dimos --simulation run unitree-go2 --mujoco-room-from-occupancy " + args.out)


if __name__ == "__main__":
    main()
