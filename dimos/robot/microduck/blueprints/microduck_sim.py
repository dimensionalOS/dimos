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

"""Microduck room-simulation blueprint.

A pretrained Microduck (pollen-robotics's ~25 cm biped) walking in a small
MuJoCo room, with the standard dimOS navigation stack on top of its RL gait:

    raycast lidar -> VoxelGridMapper -> CostMapper -> ReplanningAStarPlanner
        -> MovementManager -> cmd_vel -> walking policy -> MuJoCo

Usage:
    dimos run microduck-sim
    dimos shell            # then e.g. app.WavefrontFrontierExplorer.explore()

Model assets and the walking policy are downloaded from the public
pollen-robotics repos into ~/.cache/dimos/microduck on first start.
"""

from __future__ import annotations

from pathlib import Path
import platform

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.microduck.config import MICRODUCK
from dimos.robot.microduck.sim_module import LIDAR_CAMERA_SPECS, MicroduckSimModule

_SCENE_XML = Path(__file__).resolve().parents[1] / "assets" / "room_scene.xml"

# Ground-truth object positions; must match assets/room_scene.xml.
MICRODUCK_ROOM_OBJECTS: dict[str, tuple[float, float]] = {
    "red_box": (1.5, 0.8),
    "blue_box": (-1.5, -0.8),
}

microduck_sim = (
    autoconnect(
        MicroduckSimModule.blueprint(
            scene_xml=_SCENE_XML,
            headless=True,
            spawn_xy=(0.0, 0.0),
            # Head camera stream (for viewers / future perception). Rendered
            # cameras need mujoco.Renderer, whose GL context cannot be
            # created off the main thread on macOS - the sim thread would
            # deadlock the worker - so the stream is Linux-only.
            width=320,
            height=240,
            fps=5,
            enable_color=platform.system() != "Darwin",
            enable_depth=False,
            # Raycast lidar -> world-frame pointcloud for mapping. World
            # geometry is group 0; the robot (groups 2/3) is invisible to it.
            enable_pointcloud=True,
            pointcloud_fps=3.0,
            enable_mujoco_lidar=True,
            mujoco_lidar_camera_names=[name for name, _ in LIDAR_CAMERA_SPECS],
            mujoco_lidar_geom_groups=[0],
            # Dense enough that the room's small (6-14 cm) objects reliably
            # land in the voxel map, not just the walls.
            mujoco_lidar_raycast_width=96,
            mujoco_lidar_raycast_height=48,
            mujoco_lidar_min_range=0.05,
            mujoco_lidar_max_range=6.0,
            mujoco_lidar_max_height=0.6,
            mujoco_lidar_voxel_size=0.03,
            mujoco_lidar_robot_exclusion_radius=0.2,
        ),
        # CPU voxel grid: the room is tiny, and this also runs on machines
        # without CUDA (macOS).
        VoxelGridMapper.blueprint(emit_every=1, voxel_size=0.03, device="CPU:0"),
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=0.03,
                can_pass_under=MICRODUCK.height_clearance + 0.05,
                can_climb=0.03,
            ),
            initial_safe_radius_meters=MICRODUCK.width_clearance + 0.15,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=MICRODUCK.width_clearance,
            robot_rotation_diameter=MICRODUCK.rotation_diameter,
        ),
        MovementManager.blueprint(),
        WavefrontFrontierExplorer.blueprint(
            min_frontier_perimeter=0.15,
            safe_distance=0.6,
            lookahead_distance=2.0,
            max_explored_distance=6.0,
            goal_timeout=30.0,
        ),
    )
    .remappings([(VoxelGridMapper, "lidar", "pointcloud")])
    # nerf_speed halves the planner's 0.55 m/s default; the sim module's
    # command gain maps the rest into the gait's trained velocity range.
    .global_config(robot_model="microduck", nerf_speed=0.5)
)
