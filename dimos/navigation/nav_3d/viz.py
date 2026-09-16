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


"""Rendering shared by the nav_3d bridge config and the replay tools: maps, path, goal, robot."""

from __future__ import annotations

from collections.abc import Callable
from functools import partial
import math
from types import ModuleType
from typing import TYPE_CHECKING

import numpy as np

from dimos.mapping.ray_tracing.viz import voxel_map_points
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

    from dimos.msgs.geometry_msgs.PointStamped import PointStamped
    from dimos.msgs.nav_msgs.Path import Path
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.visualization.rerun.bridge import VisualOverride

PATH_Z_LIFT = 0.05

PATH_COLOR = (0, 255, 0)
GOAL_COLOR = (255, 0, 0)
BODY_COLOR = (0, 255, 127)
CLEARANCE_COLOR = (255, 120, 120, 80)


def render_voxel_map(msg: PointCloud2, voxel_size: float) -> Archetype:
    return voxel_map_points(msg.points_f32(), voxel_size)


def path_strip(
    waypoints: NDArray[np.float32] | None, color: tuple[int, int, int] = PATH_COLOR
) -> Archetype:
    import rerun as rr

    if waypoints is None or len(waypoints) == 0:
        return rr.LineStrips3D([])
    points = np.asarray(waypoints, dtype=np.float32).copy()
    points[:, 2] += PATH_Z_LIFT
    return rr.LineStrips3D([points], colors=[color], radii=0.05)


def render_path(msg: Path) -> Archetype | None:
    """The planned route. An empty path means no route, so the last one stays drawn."""
    if len(msg.poses) == 0:
        return None
    return path_strip(np.array([[p.x, p.y, p.z] for p in msg.poses], dtype=np.float32))


def goal_point(xyz: tuple[float, float, float]) -> Archetype:
    import rerun as rr

    return rr.Points3D([xyz], colors=[GOAL_COLOR], radii=0.1)


def render_goal(msg: PointStamped) -> Archetype | None:
    """The active goal. NaN is the movement manager's placeholder for none."""
    if any(math.isnan(v) for v in (msg.x, msg.y, msg.z)):
        return None
    return goal_point((msg.x, msg.y, msg.z))


def robot_body_box(length: float, width: float, height: float) -> Archetype:
    import rerun as rr

    return rr.Boxes3D(half_sizes=[length / 2, width / 2, height / 2], colors=[BODY_COLOR])


def robot_clearance(height: float, wall_clearance_m: float) -> Archetype:
    """The planner's wall clearance as a cylinder around the body."""
    import rerun as rr

    return rr.Cylinders3D(
        lengths=[height],
        radii=[wall_clearance_m],
        colors=[CLEARANCE_COLOR],
        fill_mode="solid",
    )


def _body_on_base_link(
    rr: ModuleType, length: float, width: float, height: float
) -> list[Archetype]:
    return [robot_body_box(length, width, height), rr.Transform3D(parent_frame="tf#/base_link")]


def _clearance_on_body(rr: ModuleType, height: float, wall_clearance_m: float) -> list[Archetype]:
    return [robot_clearance(height, wall_clearance_m)]


def nav_static(
    length: float, width: float, height: float, wall_clearance_m: float
) -> dict[str, Callable[[ModuleType], list[Archetype]]]:
    """Bridge static entities: the body box and clearance cylinder riding on base_link.

    Module-level partials, since blueprint config is pickled out to the workers.
    """
    return {
        "world/robot_body": partial(_body_on_base_link, length=length, width=width, height=height),
        "world/robot_body/clearance": partial(
            _clearance_on_body, height=height, wall_clearance_m=wall_clearance_m
        ),
    }


def nav_visual_override(
    viz_publish_hz: float,
    voxel_size: float,
    wall_clearance_m: float,
    clearance_clamp_m: float = 1.0,
) -> dict[str, VisualOverride]:
    """Bridge overrides for the maps, path, goal and the planner's debug entities.

    Pass the same values given to ``MLSPlannerNative.blueprint(...)``.
    """
    voxels = partial(render_voxel_map, voxel_size=voxel_size)
    return {
        "world/global_map": voxels,
        "world/full_map": voxels,
        "world/local_map": voxels,
        "world/path": render_path,
        "world/goal": render_goal,
        **planner_visual_override(viz_publish_hz, voxel_size, wall_clearance_m, clearance_clamp_m),
    }
