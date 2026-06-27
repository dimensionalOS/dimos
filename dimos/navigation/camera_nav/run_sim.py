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

"""Run camera-nav pipeline inside a MuJoCo sim built from our scanned costmap.

The MuJoCo scene geometry is reconstructed from the occupancy grid exported by
CostMapper at shutdown (default ~/Downloads/map_costmap.npy).  A G1 robot is
placed in that environment and drives itself using the same depth→map→nav stack
that runs on the real robot.

Usage::

    # Run in the environment scanned most recently
    python -m dimos.navigation.camera_nav.run_sim

    # Specify a different costmap
    python -m dimos.navigation.camera_nav.run_sim --costmap ~/Downloads/map_costmap.npy
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--costmap",
        default="~/Downloads/map_costmap.npy",
        help="Path to occupancy grid exported by CostMapper (.npy)",
    )
    parser.add_argument("--cpu", action="store_true", help="Force CPU depth inference")
    args = parser.parse_args()

    costmap_path = str(Path(args.costmap).expanduser())
    if not os.path.exists(costmap_path):
        raise FileNotFoundError(
            f"Costmap not found: {costmap_path}\n"
            "Run '--zed-only' or '--trial' first to build and export the map."
        )

    # Set env before importing GlobalConfig so it picks up the path.
    os.environ["MUJOCO_ROOM_FROM_OCCUPANCY"] = costmap_path

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.mapping.costmapper import CostMapper
    from dimos.navigation.camera_nav.viz import cloud_points, costmap_viz, pinhole_setup
    from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
        WavefrontFrontierExplorer,
    )
    from dimos.navigation.movement_manager.movement_manager import MovementManager
    from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
    from dimos.perception.depth.accumulator import DepthAccumulatorModule
    from dimos.perception.depth.monocular_depth_module import MonocularDepthModule
    from dimos.protocol.pubsub.impl.lcmpubsub import LCM
    from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_nav import (
        unitree_g1_primitive_no_nav,
    )
    from dimos.robot.unitree.g1.mujoco_sim import G1SimConnection
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    device = "cpu" if args.cpu else None  # None = auto-detect CUDA/MPS

    sim = autoconnect(
        unitree_g1_primitive_no_nav,
        G1SimConnection.blueprint(),
        MonocularDepthModule.blueprint(device=device),
        DepthAccumulatorModule.blueprint(),
        CostMapper.blueprint(algo="height_cost"),
        WavefrontFrontierExplorer.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
        RerunBridgeModule.blueprint(
            pubsubs=[LCM()],
            rerun_open="web",
            visual_override={
                "world/global_map": cloud_points,
                "world/frame_cloud": cloud_points,
                "world/global_costmap": costmap_viz,
                "world/camera_info": pinhole_setup,
            },
        ),
    )

    ModuleCoordinator.build(sim).loop()
