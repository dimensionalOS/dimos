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

"""Go2 navigation, composed onto any source of lidar, odometry and the odom tf edge."""

from __future__ import annotations

from collections.abc import Callable
from types import ModuleType
from typing import TYPE_CHECKING

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.viz import nav_static, nav_visual_override
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.robot.unitree.go2.nav_3d_config import ray_tracing_config, voxel_size, wall_clearance_m

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

    from dimos.visualization.rerun.bridge import VisualOverride

# Raise above 0 to draw what the planner searched over: surface, nodes and cost-colored
# edges. Drives both its publishing and the rerun overrides.
planner_viz_hz = 2.0
BODY_DILATE_M = -0.03

_mls_planner = MLSPlannerNative.blueprint(
    world_frame="odom",
    voxel_size=voxel_size,
    robot_height=0.4,
    surface_closing_radius=0.4,
    wall_clearance_m=0.05,
    wall_buffer_m=0.2,
    wall_buffer_weight=20.0,
    step_threshold_m=0.16,
    step_penalty_weight=4.0,
    viz_publish_hz=planner_viz_hz,
).remappings(
    [
        (MLSPlannerNative, "global_map", "global_map_unused"),
        (MLSPlannerNative, "path", "planner_path"),
    ]
)

# MLS stays global. Its path is remapped to planner_path, the carrot source for the local
# planner over the raycaster's local map.
_go2_nav = autoconnect(
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
    _mls_planner,
    LocalPlannerNative.blueprint(body_dilate_m=BODY_DILATE_M),
    TrajectoryFollowerNative.blueprint(),
    MovementManager.blueprint(),
)


def go2_nav_static() -> dict[str, Callable[[ModuleType], list[Archetype]]]:
    """Bridge static entities for navigation: the body box and clearance cylinder on base_link."""
    return nav_static(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT, wall_clearance_m)


def go2_nav_overrides() -> dict[str, VisualOverride]:
    """Bridge overrides for the navigation maps, paths, goal and planner debug entities."""
    return {
        **nav_visual_override(planner_viz_hz, voxel_size, wall_clearance_m),
        **motion_visual_override(body_dilate_m=BODY_DILATE_M),
    }
