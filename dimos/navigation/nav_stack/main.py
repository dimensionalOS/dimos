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

"""SmartNav composable navigation stack.

`create_nav_stack(**kwargs)` returns an autoconnected Blueprint containing the core
SmartNav modules (terrain analysis, local planner, path follower, FAR planner,
PGO, click-to-goal, cmd-vel mux), with optional TARE exploration.

`nav_stack_rerun_config(user_config)` returns a Rerun config dict with the
SmartNav defaults filled in via setdefault — pass it to `RerunBridgeModule`
or `vis_module` separately.

Defaults match the onboard (real hardware) configuration. Override any
module's config via per-module kwarg dicts (e.g.
`terrain_analysis={"obstacle_height_threshold": 0.1}`).
"""

from __future__ import annotations

import math
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.navigation.nav_stack.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.nav_stack.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.nav_stack.modules.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.modules.nav_record.nav_record import NavRecord
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
    use_simple_planner: bool = False,
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
    movement_manager: dict[str, Any] | None = None,
    tare_planner: dict[str, Any] | None = None,
    nav_record: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose a SmartNav autoconnect Blueprint with the given options.

    Core external streams (always present regardless of toggles):

        registered_scan: In[PointCloud2]   — world-frame lidar scan
        odometry:        In[Odometry]      — raw SLAM odometry
        clicked_point:   In[PointStamped]  — click-to-goal from UI
        joy_cmd:         In[Twist]         — optional joystick override
        tele_cmd_vel:    In[Twist]         — optional teleop command

        cmd_vel:            Out[Twist]        — final velocity command (MovementManager)
        corrected_odometry: Out[Odometry]     — PGO loop-closure-corrected pose
        global_map:         Out[PointCloud2]  — PGO accumulated keyframe map
        terrain_map:        Out[PointCloud2]  — TerrainAnalysis ground/obstacle grid
        path:               Out[Path]         — LocalPlanner's chosen local path
        goal_path:          Out[Path]         — FAR planner's global path
        way_point:          Out[PointStamped] — current waypoint target
        goal:               Out[PointStamped] — current navigation goal
        stop_movement:      Out[Bool]         — stop signal from MovementManager

    Args:
        use_tare: Add the TARE frontier-based exploration planner. Auto-remaps
            MovementManager's `way_point` output so TARE has exclusive control
            of LocalPlanner's waypoint input.
        use_terrain_map_ext: Add TerrainMapExt — the persistent extended terrain
            accumulator used for visualization and wider-range planning.
        vehicle_height: Ignore terrain points above this height (m). Threaded
            into TerrainAnalysis's `vehicle_height` config. Defaults to 1.2m.
        max_speed: Cap peak velocity (m/s) on both LocalPlanner (planning) and
            PathFollower (execution). Sets `max_speed` and `autonomy_speed` on
            both modules. Per-module overrides still win.
        terrain_analysis, terrain_map_ext, local_planner, path_follower,
        far_planner, pgo, movement_manager, tare_planner:
        Per-module config override dicts. Merged on top
        of the SmartNav defaults.
        record: Add NavRecord module to record all nav streams to SQLite.
        nav_record: Config override dict for NavRecord (e.g. ``{"db_path": "..."}``).

    Returns:
        An autoconnected Blueprint with the selected modules wired together.
    """
    terrain_analysis_config = {**(terrain_analysis or {})}
    local_planner_config = {**(local_planner or {})}
    terrain_analysis_threshold = terrain_analysis_config.get("obstacle_height_threshold", 0.1)
    local_planner_threshold = local_planner_config.get("obstacle_height_threshold", 0.1)
    if terrain_analysis_threshold < local_planner_threshold:
        logger.warning(
            "terrain_analysis obstacle_height_threshold (%.3f) < "
            "local_planner obstacle_height_threshold (%.3f). "
            "Terrain analysis will pass through points that local_planner "
            "treats as hard obstacles, causing phantom obstacle blocking.",
            terrain_analysis_threshold,
            local_planner_threshold,
        )

    modules: list[Blueprint] = [
        TerrainAnalysis.blueprint(
            **{
                # Input filtering
                "scan_voxel_size": 0.05,
                # Voxel grid
                "terrain_voxel_size": terrain_voxel_size,
                "terrain_voxel_half_width": 10,
                # Obstacle/ground classification
                "obstacle_height_threshold": 0.1,
                "ground_height_threshold": 0.1,
                "min_relative_z": -1.5,
                "max_relative_z": 0.3,
                "use_sorting": True,
                "quantile_z": 0.25,
                # Decay and clearing
                "decay_time": 1.0,
                "no_decay_distance": 1.5,
                "clearing_distance": 8.0,
                "clear_dynamic_obstacles": True,
                "no_data_obstacle": False,
                "no_data_block_skip_count": 0,
                "min_block_point_count": 10,
                # Voxel culling
                "voxel_point_update_threshold": 100,
                "voxel_time_update_threshold": 2.0,
                # Dynamic obstacle filtering
                "min_dynamic_obstacle_distance": 0.14,
                "abs_dynamic_obstacle_relative_z_threshold": 0.2,
                "min_dynamic_obstacle_vfov": -55.0,
                "max_dynamic_obstacle_vfov": 10.0,
                "min_dynamic_obstacle_point_count": 1,
                "min_out_of_fov_point_count": 20,
                # Ground lift limits
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
        *(
            [
                SimplePlanner.blueprint(
                    **{
                        "replan_rate": replan_rate,
                        **(
                            {"ground_offset_below_robot": vehicle_height}
                            if vehicle_height is not None
                            else {}
                        ),
                        **(simple_planner or {}),
                    }
                )
            ]
            if use_simple_planner
            else [FarPlanner.blueprint(**(far_planner or {}))]
        ),
        PGO.blueprint(**(pgo or {})),
        MovementManager.blueprint(**(movement_manager or {})),
    ]
    if use_terrain_map_ext:
        modules.append(
            TerrainMapExt.blueprint(
                **{
                    # Note: stale obstacles may appear and take a bit to clear if this voxel_size is different than the terrain_voxel_size
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
    if record:
        modules.append(NavRecord.blueprint(**(nav_record or {})))

    remappings: list[tuple[type[ModuleBase], str, str | type[ModuleBase] | type[Spec]]] = [
        # PathFollower cmd_vel → MovementManager nav input (avoid collision with mux output)
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        # NativeModule planners still receive corrected odometry via the
        # stream (C++ binaries subscribe to LCM topics directly).
        # Python modules (SimplePlanner, MovementManager) query the TF tree
        # instead (map→body via the PGO map→odom + FastLio2 odom→body chain).
        *([] if use_simple_planner else [(FarPlanner, "odometry", "corrected_odometry")]),
        (TerrainAnalysis, "odometry", "corrected_odometry"),
        # Planner owns way_point — disconnect MovementManager's click relay.
        (MovementManager, "way_point", "_mgr_way_point_unused"),
        (PGO, "global_map", "global_map_pgo"),
        *([(NavRecord, "global_map", "global_map_pgo")] if record else []),
    ]

    return autoconnect(*modules).remappings(remappings)


# ─── Rerun visual overrides (robot-agnostic) ─────────────────────────────────


def nav_stack_rerun_config(
    user_config: dict[str, Any] | None = None,
    *,
    agentic_debug: bool = False,
) -> dict[str, Any]:
    """Return a rerun config dict with SmartNav defaults filled in via setdefault.

    The caller's entries win — this just ensures missing keys (blueprint,
    pubsubs, visual_override entries, static entries) are
    populated with the SmartNav defaults.

    Args:
        agentic_debug: When True, elevate nav paths, goals, and waypoints high
            above the scene so they're visible even when occluded by
            terrain/obstacles.  Useful for debugging planner behavior from a
            top-down view.
    """
    resolved = dict(user_config or {})
    resolved.setdefault("blueprint", _default_rerun_blueprint)
    resolved.setdefault("pubsubs", [LCM()])
    resolved.setdefault("visual_override", {})
    resolved.setdefault("static", {})
    visual_override = dict(resolved["visual_override"])
    visual_override.setdefault("world/sensor_scan", _sensor_scan_override)
    visual_override.setdefault("world/terrain_map", _terrain_map_override)
    # terrain_map_ext is the persistent accumulator of the same terrain data,
    # so reuse the warm (yellow → red) gradient for visual consistency.
    visual_override.setdefault("world/terrain_map_ext", _terrain_map_override)
    visual_override.setdefault("world/global_map", _global_map_override)
    # Common remapped names: PGO renames to global_map_pgo (in nav_stack itself),
    # FastLio2's global_map is typically remapped to global_map_fastlio to avoid
    # the collision. Register both so the cool palette applies either way.
    visual_override.setdefault("world/global_map_pgo", _global_map_override)
    visual_override.setdefault("world/global_map_fastlio", _global_map_override)
    # registered_scan is the live lidar scan consumed by nav_stack; share the
    # global_map's blue → green gradient so SLAM-space data reads as one family.
    visual_override.setdefault("world/registered_scan", _global_map_override)
    visual_override.setdefault("world/explored_areas", _explored_areas_override)
    visual_override.setdefault("world/preloaded_map", _preloaded_map_override)
    visual_override.setdefault("world/trajectory", _trajectory_override)
    visual_override.setdefault("world/path", _path_override)
    if agentic_debug:
        visual_override.setdefault("world/way_point", _waypoint_override_debug)
        visual_override.setdefault("world/goal", _goal_override_debug)
        visual_override.setdefault("world/goal_path", _goal_path_override_debug)
        visual_override.setdefault("world/nav_boundary", _nav_boundary_override_debug)
    else:
        visual_override.setdefault("world/way_point", _waypoint_override)
        visual_override.setdefault("world/goal", _goal_override)
        visual_override.setdefault("world/goal_path", _goal_path_override)
        visual_override.setdefault("world/nav_boundary", _nav_boundary_override)
    visual_override.setdefault("world/obstacle_cloud", _obstacle_cloud_override)
    visual_override.setdefault("world/costmap_cloud", _costmap_cloud_override)
    visual_override.setdefault("world/free_paths", _free_paths_override)
    resolved["visual_override"] = visual_override
    static_entries = dict(resolved["static"])
    static_entries.setdefault("world/floor", _static_floor)
    resolved["static"] = static_entries
    return resolved


# Z-offsets for Rerun visualization (metres above the data's actual z).
# Small lifts prevent z-fighting with the terrain/floor plane.
_VIS_LIFT = 0.3  # default lift for nav markers (goals, paths, boundaries)
_VIS_LIFT_TRAJECTORY = 0.05  # trajectory breadcrumbs sit just above the floor
_VIS_LIFT_COSTMAP = 0.2  # lift costmap cells clear of terrain/obstacle clouds

# Agentic debug mode lifts nav elements high above the scene so they're
# visible from a top-down camera even when terrain occludes them.
_AGENTIC_DEBUG_LIFT = 3.0
_AGENTIC_DEBUG_PATH_LIFT = _AGENTIC_DEBUG_LIFT + 0.4  # path slightly above goal markers
_AGENTIC_DEBUG_BOUNDARY_LIFT = _AGENTIC_DEBUG_LIFT - 1.0  # boundary below markers


def _default_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


def _sensor_scan_override(cloud: Any) -> Any:
    """Hide sensor_scan — it clutters the view with raw lidar returns."""
    return None


def _global_map_override(cloud: Any) -> Any:
    """Render accumulated global map with a blue→green gradient by z-height.

    Cool half (deep blue → green, passing through teal) keeps the room map
    visually separated from the terrain_map's warm half (yellow → red).
    Shared across `world/global_map`, `world/global_map_pgo`, and
    `world/global_map_fastlio` so every SLAM/PGO map uses the same palette.
    """
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


def _terrain_map_override(cloud: Any) -> Any:
    """Render terrain_map with a lavender → magenta gradient by z-height.

    The terrain_analysis C++ module sets point intensity to the height
    difference above the planar voxel ground. Low z → ground (pale lavender),
    high z → obstacle (vivid magenta). Magenta/purple slice keeps terrain
    visually distinct from both global_map (blue → green) and any
    yellow/red elements.
    """
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


def _costmap_cloud_override(cloud: Any) -> Any:
    """Render SimplePlanner's costmap_cloud — the blocked grid cells
    (with inflation) that A* actually treats as obstacles. Big red
    boxes so they pop against the terrain clouds.

    Lifted above ground so the costmap cells aren't buried under the
    terrain/obstacle clouds that share the same z-range.
    """
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None
    lifted = points[:, :3].copy()
    lifted[:, 2] += _VIS_LIFT_COSTMAP
    colors = np.full((len(points), 3), [255, 40, 40], dtype=np.uint8)
    return rr.Points3D(positions=lifted, colors=colors, radii=0.12)


def _obstacle_cloud_override(cloud: Any) -> Any:
    """Render LocalPlanner's obstacle_cloud — the vehicle-frame crop the C++
    planner actually tests candidate paths against. Colored by intensity
    (= height above ground) with a `plasma` colormap so it's visually
    distinct from terrain_map. Attached to the sensor TF frame since the
    points are already in vehicle frame.
    """
    import rerun as rr

    arch = cloud.to_rerun(colormap="plasma", size=0.06)
    return [
        ("world/obstacle_cloud", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/obstacle_cloud", arch),
    ]


def _explored_areas_override(cloud: Any) -> Any:
    """Render PreloadedMapTracker's explored_areas — cumulative seen points."""
    return cloud.to_rerun(colormap="magma", size=0.05)


def _preloaded_map_override(cloud: Any) -> Any:
    """Render PreloadedMapTracker's static pre-loaded reference map."""
    return cloud.to_rerun(colormap="greys", size=0.04)


def _trajectory_override(cloud: Any) -> Any:
    """Render robot trajectory breadcrumb as a connected line strip."""
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) < 2:
        return None
    pts = [[float(p[0]), float(p[1]), float(p[2]) + _VIS_LIFT_TRAJECTORY] for p in points]
    return [
        ("world/trajectory/line", rr.LineStrips3D([pts], colors=[(0, 200, 255)], radii=0.03)),
        ("world/trajectory/nodes", rr.Points3D(pts, colors=[(0, 150, 255)], radii=0.05)),
    ]


def _path_override(path_msg: Any) -> Any:
    """Render path in vehicle frame by attaching to the sensor TF."""
    import rerun as rr

    if not path_msg.poses:
        return None

    points = [[p.x, p.y, p.z + _VIS_LIFT] for p in path_msg.poses]
    return [
        ("world/nav_path", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/nav_path", rr.LineStrips3D([points], colors=[(0, 255, 128)], radii=0.05)),
    ]


def _nav_boundary_override(msg: Any) -> Any:
    """Render navigation boundary as cyan edges."""
    return msg.to_rerun(z_offset=_VIS_LIFT, color=(0, 220, 255, 200), radii=0.05)


def _goal_path_override(path_msg: Any) -> Any:
    """Render FAR planner's planned path: orange line + yellow node markers."""
    import rerun as rr

    if not path_msg.poses or len(path_msg.poses) < 2:
        return None

    points = [[p.x, p.y, p.z + _VIS_LIFT] for p in path_msg.poses]
    return [
        # Edges: orange line connecting all waypoints
        ("world/goal_path/edges", rr.LineStrips3D([points], colors=[(255, 140, 0)], radii=0.06)),
        # Nodes: yellow spheres at each graph node in the path
        ("world/goal_path/nodes", rr.Points3D(points, colors=[(255, 255, 0)], radii=0.15)),
    ]


def _waypoint_override(msg: Any) -> Any:
    """Render the current waypoint goal as a visible marker.

    Orange + slightly smaller than the goal sphere so the final goal
    stays the larger, dominant marker.
    """
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _VIS_LIFT]],
        colors=[(255, 140, 0)],
        radii=0.22,
    )


def _goal_override(msg: Any) -> Any:
    """Render the current navigation goal as a purple sphere."""
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _VIS_LIFT]],
        colors=[(180, 60, 220)],
        radii=0.3,
    )


def _free_paths_override(cloud: Any) -> Any:
    """Render LocalPlanner free (collision-free) candidate paths in vehicle frame."""
    import rerun as rr

    return [
        ("world/free_paths", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/free_paths", cloud.to_rerun(colormap="cool", size=0.02)),
    ]


def _static_floor(rr: Any) -> list[Any]:
    """Static ground plane as a solid textured quad.

    Dropped 0.2 m below z=0 so lidar points that land right at ground height
    (low-obstacle edges, ground classifications) stay visible instead of
    getting z-fought / occluded by the floor quad.
    """

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


# ─── Debug overrides (elevated paths for top-down debugging) ─────────────────


def _waypoint_override_debug(msg: Any) -> Any:
    """Agentic debug: waypoint elevated above the scene."""
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _AGENTIC_DEBUG_LIFT]],
        colors=[(255, 140, 0)],
        radii=0.22,
    )


def _goal_override_debug(msg: Any) -> Any:
    """Agentic debug: goal elevated above the scene."""
    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _AGENTIC_DEBUG_LIFT]],
        colors=[(180, 60, 220)],
        radii=0.3,
    )


def _goal_path_override_debug(path_msg: Any) -> Any:
    """Agentic debug: goal path elevated above the scene."""
    import rerun as rr

    if not path_msg.poses or len(path_msg.poses) < 2:
        return None

    points = [[p.x, p.y, p.z + _AGENTIC_DEBUG_PATH_LIFT] for p in path_msg.poses]
    return [
        ("world/goal_path/edges", rr.LineStrips3D([points], colors=[(255, 140, 0)], radii=0.06)),
        ("world/goal_path/nodes", rr.Points3D(points, colors=[(255, 255, 0)], radii=0.15)),
    ]


def _nav_boundary_override_debug(msg: Any) -> Any:
    """Agentic debug: nav boundary elevated above the scene."""
    return msg.to_rerun(z_offset=_AGENTIC_DEBUG_BOUNDARY_LIFT, color=(0, 220, 255, 200), radii=0.05)
