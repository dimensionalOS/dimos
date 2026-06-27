# Copyright 2025-2026 Dimensional Inc.
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

"""Visualise a scanned environment in MuJoCo.

Loads the occupancy grid exported by CostMapper and opens an interactive
MuJoCo viewer showing the reconstructed room geometry (walls extruded from
occupied cells).  Uses only plain `mujoco` — no mujoco_playground needed.

Usage::

    python -m dimos.navigation.camera_nav.run_sim
    python -m dimos.navigation.camera_nav.run_sim --costmap ~/Downloads/map_costmap.npy
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer

from dimos.mapping.occupancy.extrude_occupancy import generate_mujoco_scene
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--costmap",
        default="~/Downloads/map_costmap.npy",
        help="Occupancy grid exported by CostMapper (.npy)",
    )
    args = parser.parse_args()

    costmap_path = Path(args.costmap).expanduser()
    if not costmap_path.exists():
        raise FileNotFoundError(
            f"Costmap not found: {costmap_path}\n"
            "Run '--zed-only' or '--trial' first to scan and export the map."
        )

    grid = OccupancyGrid.from_path(costmap_path)
    logger.info("Loaded costmap %s  shape=%s", costmap_path, grid.grid.shape)

    xml = generate_mujoco_scene(grid)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    logger.info("Opening MuJoCo viewer — drag to orbit, scroll to zoom, Esc to quit")
    mujoco.viewer.launch(model, data)
