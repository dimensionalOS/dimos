#!/usr/bin/env python3
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

"""Drop-in pimsim variant of unitree_g1_nav_sim.

Identical nav stack + rerun setup as ``unitree_g1_nav_sim``, but replaces
the Unity TCP simulator with the in-process Babylon viewer / kinematic
sim from ``dimos.experimental.pimsim``. The viewer doesn't synthesize
camera or lidar frames, so vision-dependent plumbing is dropped here.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from mujoco_playground._src import mjx_env

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.experimental.pimsim.module import BabylonSceneViewerModule
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.unitree.g1.config import G1, G1_LOCAL_PLANNER_PRECOMPUTED_PATHS, G1_PELVIS_HEIGHT
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.simulation.scenes import get_dimos_office
from dimos.visualization.vis_module import vis_module


def _resolve_g1_mjcf() -> Path:
    mjx_env.ensure_menagerie_exists()
    return Path(mjx_env.MENAGERIE_PATH) / "unitree_g1" / "scene.xml"


_OFFICE = get_dimos_office()


nav_config: dict[str, Any] = dict(
    planner="simple",
    vehicle_height=G1.height_clearance,
    max_speed=2.0,
    terrain_analysis={
        "ground_height_threshold": 0.05,
        "min_relative_z": -1.5,
    },
    terrain_map_ext={
        "decay_time": 120,
    },
    local_planner={
        "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
        "min_relative_z": -1.5,
        "freeze_ang": 180.0,
        "obstacle_height_threshold": 0.02,
        "publish_free_paths": True,
    },
    path_follower={
        "max_acceleration": 2.0,
        "max_yaw_rate": 60.0,
    },
)

unitree_g1_nav_sim_pimsim = (
    autoconnect(
        BabylonSceneViewerModule.blueprint(
            mjcf_path=str(_resolve_g1_mjcf()),
            scene=_OFFICE,
            vehicle_height=G1_PELVIS_HEIGHT,
            init_x=-2.0,
            init_z=_OFFICE.floor_z,
            lock_z=True,
            enable_sim=True,
        ),
        create_nav_stack(**nav_config),
        MovementManager.blueprint(),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=nav_stack_rerun_config(
                {
                    "static": {
                        "world/tf/robot": g1_static_robot,
                    },
                },
                vis_throttle=0.1,
            ),
        ),
    )
    .remappings(
        [
            # Route the synthetic lidar into the viewer's point-cloud overlay
            # so you can see what mj_ray is actually hitting.
            (BabylonSceneViewerModule, "pointcloud_overlay", "registered_scan"),
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
)
