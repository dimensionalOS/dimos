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

"""Composable navigation stack.

``create_nav_stack()`` returns a Blueprint with terrain analysis, local
planner, path follower, global planner (FAR or SimplePlanner), and PGO.
``nav_stack_rerun_config()`` returns Rerun visualization defaults.
"""

from __future__ import annotations

import math
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.navigation.nav_stack.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.nav_stack.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.nav_stack.modules.path_follower.path_follower import PathFollower
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.modules.simple_planner.simple_planner import SimplePlanner
from dimos.navigation.nav_stack.modules.tare_planner.tare_planner import TarePlanner
from dimos.navigation.nav_stack.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.nav_stack.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def create_nav_stack(
    *,
    use_tare: bool = False,
    use_terrain_map_ext: bool = True,
    planner: str = "far",
    vehicle_height: float | None = None,
    max_speed: float | None = None,
    terrain_voxel_size: float = 0.2,
    replan_rate: float = 0.5,
    record: bool = False,
    terrain_analysis: dict[str, Any] | None = None,
    terrain_map_ext: dict[str, Any] | None = None,
    local_planner: dict[str, Any] | None = None,
    path_follower: dict[str, Any] | None = None,
    far_planner: dict[str, Any] | None = None,
    simple_planner: dict[str, Any] | None = None,
    pgo: dict[str, Any] | None = None,
    tare_planner: dict[str, Any] | None = None,
    nav_record: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose a nav stack Blueprint.

    Per-module config dicts (``terrain_analysis``, ``local_planner``, etc.)
    override defaults. ``vehicle_height`` and ``max_speed`` propagate to
    the relevant modules automatically.
    """
    far_planner_config = {**(far_planner or {})}

    # Propagate vehicle_height to far_planner config
    if vehicle_height is not None:
        far_planner_config.setdefault("vehicle_height", vehicle_height)
    modules: list[Blueprint] = [
        TerrainAnalysis.blueprint(
            **{
                "scan_voxel_size": 0.05,
                "terrain_voxel_size": terrain_voxel_size,
                "terrain_voxel_half_width": 10,
                "obstacle_height_threshold": 0.1,
                "ground_height_threshold": 0.1,
                "min_relative_z": -1.5,
                "max_relative_z": 0.3,
                "use_sorting": True,
                "quantile_z": 0.25,
                "decay_time": 1.0,
                "no_decay_distance": 1.5,
                "clearing_distance": 8.0,
                "clear_dynamic_obstacles": True,
                "no_data_obstacle": False,
                "no_data_block_skip_count": 0,
                "min_block_point_count": 10,
                "voxel_point_update_threshold": 100,
                "voxel_time_update_threshold": 2.0,
                "min_dynamic_obstacle_distance": 0.14,
                "abs_dynamic_obstacle_relative_z_threshold": 0.2,
                "min_dynamic_obstacle_vfov": -55.0,
                "max_dynamic_obstacle_vfov": 10.0,
                "min_dynamic_obstacle_point_count": 1,
                "min_out_of_fov_point_count": 20,
                "consider_drop": False,
                "limit_ground_lift": False,
                "max_ground_lift": 0.15,
                "distance_ratio_z": 0.2,
                "vehicle_height": 1.5 if vehicle_height is None else vehicle_height,
                **(terrain_analysis or {}),
            }
        ),
        LocalPlanner.blueprint(
            **{
                "autonomy_mode": True,
                "use_terrain_analysis": True,
                "max_speed": 1.0 if max_speed is None else max_speed,
                "autonomy_speed": 1.0 if max_speed is None else max_speed,
                "obstacle_height_threshold": 0.1,
                "max_relative_z": 0.3,
                "min_relative_z": -0.4,
                "two_way_drive": False,
                **(local_planner or {}),
            }
        ),
        PathFollower.blueprint(
            **{
                "autonomy_mode": True,
                "max_speed": 1.0 if max_speed is None else max_speed,
                "autonomy_speed": 1.0 if max_speed is None else max_speed,
                "max_acceleration": 1.0,
                "slow_down_distance_threshold": 1.0,
                "omni_dir_goal_threshold": 1.0,
                "two_way_drive": False,
                **(path_follower or {}),
            }
        ),
        PGO.blueprint(**(pgo or {})),
    ]
    if planner == "simple":
        sp_config: dict[str, Any] = {"replan_rate": replan_rate}
        if vehicle_height is not None:
            sp_config["ground_offset_below_robot"] = vehicle_height
        sp_config.update(simple_planner or {})
        modules.append(SimplePlanner.blueprint(**sp_config))
    elif planner == "far":
        modules.append(FarPlanner.blueprint(**far_planner_config))
    else:
        raise Exception(f'invalid planner: {planner}')
    
    if use_terrain_map_ext:
        modules.append(
            TerrainMapExt.blueprint(
                **{
                    "voxel_size": terrain_voxel_size,
                    "decay_time": 30.0,
                    "publish_rate": replan_rate,
                    "max_range": 40.0,
                    **(terrain_map_ext or {}),
                }
            )
        )
    if use_tare:
        modules.append(TarePlanner.blueprint(**(tare_planner or {})))
    record_remappings: list[tuple[type[ModuleBase], str, str | type[ModuleBase] | type[Spec]]] = []
    if record:
        # Lazy: breaks on G1 onboard (linux-aarch64 TLS allocation failure)
        from dimos.navigation.nav_stack.modules.nav_record.nav_record import NavRecord

        modules.append(NavRecord.blueprint(**(nav_record or {})))
        record_remappings.append((NavRecord, "global_map", "global_map_pgo"))

    remappings: list[tuple[type[ModuleBase], str, str | type[ModuleBase] | type[Spec]]] = [
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        (TerrainAnalysis, "odometry", "corrected_odometry"),
        (PGO, "global_map", "global_map_pgo"),
        *record_remappings,
    ]
    if planner == "far":
        remappings.append((FarPlanner, "odometry", "corrected_odometry"))
        remappings.append((FarPlanner, "registered_scan", "_none"))

    return autoconnect(*modules).remappings(remappings)


def nav_stack_rerun_config(
    user_config: dict[str, Any] | None = None,
    *,
    agentic_debug: bool = False,
) -> dict[str, Any]:
    """Return a rerun config dict with nav stack visualization defaults.

    Caller entries win; this fills in missing keys. ``agentic_debug``
    lifts nav elements above the scene for top-down visibility.
    """
    resolved = dict(user_config or {})
    resolved.setdefault("blueprint", _default_rerun_blueprint)
    resolved.setdefault("pubsubs", [LCM()])
    resolved.setdefault("visual_override", {})
    resolved.setdefault("static", {})
    visual_override = dict(resolved["visual_override"])
    visual_override.setdefault("world/sensor_scan", _sensor_scan_colors)
    visual_override.setdefault("world/terrain_map", _terrain_map_colors)
    visual_override.setdefault("world/terrain_map_ext", _terrain_map_colors)
    visual_override.setdefault("world/global_map", _global_map_colors)
    visual_override.setdefault("world/global_map_pgo", _global_map_colors)
    visual_override.setdefault("world/global_map_fastlio", _global_map_colors)
    visual_override.setdefault("world/registered_scan", _registered_scan_colors)
    visual_override.setdefault("world/explored_areas", _explored_areas_colors)
    visual_override.setdefault("world/preloaded_map", _preloaded_map_colors)
    visual_override.setdefault("world/trajectory", _trajectory_colors)
    visual_override.setdefault("world/path", _path_colors)
    if agentic_debug:
        visual_override.setdefault("world/way_point", _waypoint_colors_debug)
        visual_override.setdefault("world/goal", _goal_colors_debug)
        visual_override.setdefault("world/goal_path", _goal_path_colors_debug)
        visual_override.setdefault("world/nav_boundary", _nav_boundary_colors_debug)
        visual_override.setdefault("world/contour_polygons", _contour_polygons_colors_debug)
        visual_override.setdefault("world/graph_nodes", _graph_nodes_colors_debug)
        visual_override.setdefault("world/graph_edges", _graph_edges_colors_debug)
    else:
        visual_override.setdefault("world/way_point", _waypoint_colors)
        visual_override.setdefault("world/goal", _goal_colors)
        visual_override.setdefault("world/goal_path", _goal_path_colors)
        visual_override.setdefault("world/nav_boundary", _nav_boundary_colors)
        visual_override.setdefault("world/contour_polygons", _contour_polygons_colors)
        visual_override.setdefault("world/graph_nodes", _hide)
        visual_override.setdefault("world/graph_edges", _hide)
    visual_override.setdefault("world/obstacle_cloud", _obstacle_cloud_colors)
    visual_override.setdefault("world/costmap_cloud", _costmap_cloud_colors)
    visual_override.setdefault("world/free_paths", _free_paths_colors)
    resolved["visual_override"] = visual_override
    static_entries = dict(resolved["static"])
    static_entries.setdefault("world/floor", _static_floor)
    resolved["static"] = static_entries
    return resolved


# Small lifts prevent z-fighting with the terrain/floor plane.
_VIS_LIFT = 0.3  # default lift for nav markers (goals, paths, boundaries)
_VIS_LIFT_TRAJECTORY = 0.05  # just above the floor
_VIS_LIFT_COSTMAP = 0.2  # high enough to avoid terrain/obstacle clouds

# lifts nav elements high above the scene so they're
# visible from a top-down camera even when terrain occludes them.
_AGENTIC_DEBUG_LIFT = 3.0
_AGENTIC_DEBUG_PATH_LIFT = _AGENTIC_DEBUG_LIFT + 0.4  # path slightly above goal markers
_AGENTIC_DEBUG_BOUNDARY_LIFT = _AGENTIC_DEBUG_LIFT - 1.0  # boundary below markers


def _default_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


def _sensor_scan_colors(cloud: Any) -> Any:
    return None


def _global_map_colors(cloud: Any) -> Any:
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    # Low z  = deep blue  (30, 80, 200)
    # High z = vivid green (60, 220, 100)
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (30 + z_norm * 30).astype(np.uint8)
    colors[:, 1] = (80 + z_norm * 140).astype(np.uint8)
    colors[:, 2] = (200 - z_norm * 100).astype(np.uint8)

    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.03)


def _registered_scan_colors(cloud: Any) -> Any:
    """Live lidar — bright white-ish points, larger than the accumulated map
    so the current sweep stands out against ``global_map`` underneath."""
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    colors = np.full((len(points), 3), [255, 240, 180], dtype=np.uint8)
    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.05)


def _terrain_map_colors(cloud: Any) -> Any:
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    # Low z  = pale lavender (200, 160, 240)
    # High z = vivid magenta (255, 40, 180)
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (200 + z_norm * 55).astype(np.uint8)
    colors[:, 1] = (160 - z_norm * 120).astype(np.uint8)
    colors[:, 2] = (240 - z_norm * 60).astype(np.uint8)

    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.08)


def _costmap_cloud_colors(cloud: Any) -> Any:
    import numpy as np
    import rerun as rr

    points, embedded = cloud.as_numpy()
    if len(points) == 0:
        return None
    lifted = points[:, :3].copy()
    lifted[:, 2] += _VIS_LIFT_COSTMAP
    if embedded is not None:
        colors = (np.clip(embedded, 0.0, 1.0) * 255).astype(np.uint8)
    else:
        colors = np.full((len(points), 3), [255, 40, 40], dtype=np.uint8)
    return rr.Points3D(positions=lifted, colors=colors, radii=0.12)


def _obstacle_cloud_colors(cloud: Any) -> Any:
    import rerun as rr

    arch = cloud.to_rerun(colormap="plasma", size=0.06)
    return [
        ("world/obstacle_cloud", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/obstacle_cloud", arch),
    ]


def _explored_areas_colors(cloud: Any) -> Any:
    return cloud.to_rerun(colormap="magma", size=0.05)


def _preloaded_map_colors(cloud: Any) -> Any:
    return cloud.to_rerun(colormap="greys", size=0.04)


def _trajectory_colors(cloud: Any) -> Any:
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) < 2:
        return None
    pts = [[float(p[0]), float(p[1]), float(p[2]) + _VIS_LIFT_TRAJECTORY] for p in points]
    return [
        ("world/trajectory/line", rr.LineStrips3D([pts], colors=[(0, 200, 255)], radii=0.03)),
        ("world/trajectory/nodes", rr.Points3D(pts, colors=[(0, 150, 255)], radii=0.05)),
    ]


def _path_colors(path_msg: Any) -> Any:
    import rerun as rr

    if not path_msg.poses:
        return None

    points = [[p.x, p.y, p.z + _VIS_LIFT] for p in path_msg.poses]
    return [
        ("world/nav_path", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/nav_path", rr.LineStrips3D([points], colors=[(0, 255, 128)], radii=0.05)),
    ]


def _nav_boundary_colors(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_VIS_LIFT, color=(0, 220, 255, 200), radii=0.05)


def _contour_polygons_colors(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_VIS_LIFT, color=(220, 30, 30, 255), radii=0.08)


def _hide(_msg: Any) -> Any:
    return None


def _goal_path_colors(path_msg: Any) -> Any:
    import rerun as rr

    poses = path_msg.poses or []
    if len(poses) < 2:
        # Cancellation sentinel from SimplePlanner — one (or zero) pose at the
        # robot.  Explicitly clear edges and replace nodes with a single grey
        # marker so the viewer reads "goal cleared, holding here" instead of
        # showing the stale path from before the cancel.
        cancel_point = (
            [[poses[0].x, poses[0].y, poses[0].z + _VIS_LIFT]] if poses else []
        )
        return [
            ("world/goal_path/edges", rr.LineStrips3D([])),
            ("world/goal_path/nodes", rr.Points3D(cancel_point, colors=[(160, 160, 160)], radii=0.18)),
        ]

    points = [[p.x, p.y, p.z + _VIS_LIFT] for p in poses]
    return [
        # Edges: orange line connecting all waypoints
        ("world/goal_path/edges", rr.LineStrips3D([points], colors=[(255, 140, 0)], radii=0.06)),
        # Nodes: yellow spheres at each graph node in the path
        ("world/goal_path/nodes", rr.Points3D(points, colors=[(255, 255, 0)], radii=0.15)),
    ]


def _waypoint_colors(msg: Any) -> Any:
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _VIS_LIFT]],
        colors=[(255, 140, 0)],
        radii=0.22,
    )


def _goal_colors(msg: Any) -> Any:
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _VIS_LIFT]],
        colors=[(180, 60, 220)],
        radii=0.3,
    )


def _free_paths_colors(cloud: Any) -> Any:
    import rerun as rr

    return [
        ("world/free_paths", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/free_paths", cloud.to_rerun(colormap="cool", size=0.02)),
    ]


def _static_floor(rr: Any) -> list[Any]:
    half_size = 50.0
    z_below_ground = -0.2
    floor_color_rgba = [40, 40, 40, 120]  # dark grey, semi-transparent
    return [
        rr.Mesh3D(
            vertex_positions=[
                [-half_size, -half_size, z_below_ground],
                [half_size, -half_size, z_below_ground],
                [half_size, half_size, z_below_ground],
                [-half_size, half_size, z_below_ground],
            ],
            triangle_indices=[[0, 1, 2], [0, 2, 3]],
            vertex_colors=[floor_color_rgba] * 4,
        )
    ]


def _waypoint_colors_debug(msg: Any) -> Any:
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _AGENTIC_DEBUG_LIFT]],
        colors=[(255, 140, 0)],
        radii=0.22,
    )


def _goal_colors_debug(msg: Any) -> Any:
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _AGENTIC_DEBUG_LIFT]],
        colors=[(180, 60, 220)],
        radii=0.3,
    )


def _goal_path_colors_debug(path_msg: Any) -> Any:
    import rerun as rr

    poses = path_msg.poses or []
    if len(poses) < 2:
        cancel_point = (
            [[poses[0].x, poses[0].y, poses[0].z + _AGENTIC_DEBUG_PATH_LIFT]] if poses else []
        )
        return [
            ("world/goal_path/edges", rr.LineStrips3D([])),
            ("world/goal_path/nodes", rr.Points3D(cancel_point, colors=[(160, 160, 160)], radii=0.18)),
        ]

    points = [[p.x, p.y, p.z + _AGENTIC_DEBUG_PATH_LIFT] for p in poses]
    return [
        ("world/goal_path/edges", rr.LineStrips3D([points], colors=[(255, 140, 0)], radii=0.06)),
        ("world/goal_path/nodes", rr.Points3D(points, colors=[(255, 255, 0)], radii=0.15)),
    ]


def _nav_boundary_colors_debug(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_AGENTIC_DEBUG_BOUNDARY_LIFT, color=(0, 220, 255, 200), radii=0.05)


def _contour_polygons_colors_debug(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_AGENTIC_DEBUG_BOUNDARY_LIFT, color=(220, 30, 30, 255), radii=0.08)


def _graph_nodes_colors_debug(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_AGENTIC_DEBUG_BOUNDARY_LIFT)


def _graph_edges_colors_debug(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_AGENTIC_DEBUG_BOUNDARY_LIFT)
