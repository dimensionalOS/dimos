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

`smart_nav(**kwargs)` returns an autoconnected Blueprint containing the core
SmartNav modules (terrain analysis, local planner, path follower, FAR planner,
PGO, click-to-goal, cmd-vel mux), with optional TARE exploration.

`smart_nav_rerun_config(user_config)` returns a Rerun config dict with the
SmartNav defaults filled in via setdefault — pass it to `RerunBridgeModule`
or `vis_module` separately.

Defaults match the onboard (real hardware) configuration. Override any
module's config via per-module kwarg dicts (e.g.
`terrain_analysis={"obstacle_height_threshold": 0.1}`).
"""

from __future__ import annotations

import logging
from typing import Any, Literal

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.spec.utils import Spec

logger = logging.getLogger(__name__)
from dimos.navigation.smart_nav.modules.arise_slam.arise_slam import AriseSlam
from dimos.navigation.smart_nav.modules.better_fastlio2.better_fastlio2 import BetterFastLio2
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.movement_manager.movement_manager import MovementManager
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smart_nav.modules.simple_planner.simple_planner import SimplePlanner
from dimos.navigation.smart_nav.modules.tare_planner.tare_planner import TarePlanner
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smart_nav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM

def _get_pgo_class() -> type:
    """Lazy-import PGO to avoid requiring gtsam Python when using other SLAM backends."""
    from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
    return PGO


SlamChoice = Literal["pgo", "better_fastlio2", "arise_slam"]
"""SLAM backend selection for smart_nav.

- ``"pgo"``: PGO (Pose Graph Optimization) — lightweight loop-closure
  correction on top of upstream odometry. Default.
- ``"better_fastlio2"``: Enhanced FAST-LIO2 with iESKF, iKdtree, Scan
  Context loop closure, GTSAM, and dynamic object removal. Requires raw
  lidar + IMU inputs (use with a sim adapter for Unity).
- ``"arise_slam"``: ARISE-SLAM — LOAM-based LiDAR-inertial SLAM with
  Ceres ICP + GTSAM IMU preintegration, curvature-based feature
  extraction, rolling grid local map, and degeneracy detection.
  Requires raw lidar + IMU inputs.
"""


def smart_nav(
    *,
    use_tare: bool = False,
    use_terrain_map_ext: bool = True,
    use_simple_planner: bool = False,
    slam: SlamChoice = "pgo",
    vehicle_height: float | None = None,
    body_frame: str | None = None,
    terrain_analysis: dict[str, Any] | None = None,
    terrain_map_ext: dict[str, Any] | None = None,
    local_planner: dict[str, Any] | None = None,
    path_follower: dict[str, Any] | None = None,
    far_planner: dict[str, Any] | None = None,
    simple_planner: dict[str, Any] | None = None,
    pgo: dict[str, Any] | None = None,
    better_fastlio2: dict[str, Any] | None = None,
    arise_slam: dict[str, Any] | None = None,
    movement_manager: dict[str, Any] | None = None,
    tare_planner: dict[str, Any] | None = None,
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
        slam: SLAM backend selection. ``"pgo"`` (default) uses PGO for
            loop-closure correction. ``"better_fastlio2"`` uses enhanced
            FAST-LIO2 with iESKF, iKdtree, Scan Context + GTSAM, and
            dynamic object removal. The ``better_fastlio2`` backend requires
            raw lidar + IMU inputs (use a sim adapter for Unity).
        vehicle_height: Ignore terrain points above this height (m). Threaded
            into TerrainAnalysis's `vehicle_height` config. Defaults to 1.2m.
        body_frame: TF child frame for the robot body. Defaults to ``"body"``
            (REP-105). Set to ``"sensor"`` for the Unity sim bridge, which
            publishes TF with ``child_frame_id="sensor"``. Threaded into
            SimplePlanner and MovementManager configs.
        terrain_analysis, terrain_map_ext, local_planner, path_follower,
        far_planner, pgo, better_fastlio2, arise_slam, movement_manager,
        tare_planner: Per-module config override dicts. Merged on top
        of the SmartNav defaults.

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
                "terrain_voxel_size": 0.2,
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
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
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
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
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
                        **(
                            {"ground_offset_below_robot": vehicle_height}
                            if vehicle_height is not None
                            else {}
                        ),
                        **({"body_frame": body_frame} if body_frame is not None else {}),
                        **(simple_planner or {}),
                    }
                )
            ]
            if use_simple_planner
            else [FarPlanner.blueprint(**(far_planner or {}))]
        ),
        *(
            [BetterFastLio2.blueprint(**(better_fastlio2 or {}))]
            if slam == "better_fastlio2"
            else [AriseSlam.blueprint(**(arise_slam or {}))]
            if slam == "arise_slam"
            else [_get_pgo_class().blueprint(**(pgo or {}))]
        ),
        MovementManager.blueprint(
            **{
                **({"body_frame": body_frame} if body_frame is not None else {}),
                **(movement_manager or {}),
            }
        ),
    ]
    if use_terrain_map_ext:
        modules.append(
            TerrainMapExt.blueprint(
                **{
                    "voxel_size": 0.1,
                    # Walls are static — keep them around long enough that
                    # a global planner (SimplePlanner / FarPlanner) doesn't
                    # see freshly-empty cells behind the robot and route
                    # paths straight through the walls it can't currently
                    # see. 8 s was way too aggressive for that.
                    # "decay_time": 300.0,
                    "decay_time": 30.0,
                    "publish_rate": 2.0,
                    "max_range": 40.0,
                    **(terrain_map_ext or {}),
                }
            )
        )
    if use_tare:
        modules.append(TarePlanner.blueprint(**(tare_planner or {})))

    remappings: list[tuple[type[ModuleBase], str, str | type[ModuleBase] | type[Spec]]] = [
        # PathFollower cmd_vel → MovementManager nav input (avoid collision with mux output)
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        # Planner owns way_point — disconnect MovementManager's click relay.
        (MovementManager, "way_point", "_mgr_way_point_unused"),
    ]

    if slam == "better_fastlio2":
        # BetterFastLio2 publishes odometry directly; remap downstream
        # modules to use it as corrected_odometry. Its registered_scan
        # output serves as the world-frame cloud for terrain analysis.
        remappings.extend([
            (BetterFastLio2, "odometry", "corrected_odometry"),
            *([] if use_simple_planner else [(FarPlanner, "odometry", "corrected_odometry")]),
            (TerrainAnalysis, "odometry", "corrected_odometry"),
            # BetterFastLio2's global_map is the accumulated keyframe map
            (BetterFastLio2, "global_map", "global_map_slam"),
        ])
    elif slam == "arise_slam":
        # AriseSlam publishes odometry directly (IMU-fused); remap
        # downstream modules to use it as corrected_odometry.
        remappings.extend([
            (AriseSlam, "odometry", "corrected_odometry"),
            *([] if use_simple_planner else [(FarPlanner, "odometry", "corrected_odometry")]),
            (TerrainAnalysis, "odometry", "corrected_odometry"),
        ])
    else:
        # PGO remappings: corrected_odometry for global planners
        PGO = _get_pgo_class()
        remappings.extend([
            *([] if use_simple_planner else [(FarPlanner, "odometry", "corrected_odometry")]),
            (TerrainAnalysis, "odometry", "corrected_odometry"),
            (PGO, "global_map", "global_map_pgo"),
        ])

    return autoconnect(*modules).remappings(remappings)


# ─── Rerun visual overrides (robot-agnostic) ─────────────────────────────────


def smart_nav_rerun_config(
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
    visual_override.setdefault("world/terrain_map_ext", _terrain_map_ext_override)
    visual_override.setdefault("world/global_map", _global_map_override)
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
    """Render accumulated global map — small grey/blue points for map context."""
    return cloud.to_rerun(colormap="cool", size=0.03)


def _terrain_map_override(cloud: Any) -> Any:
    """Render terrain_map: big green dots = traversable, red = obstacle.

    The terrain_analysis C++ module sets point intensity to the height
    difference above the planar voxel ground. Low intensity → ground,
    high intensity → obstacle.
    """
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    # Color by z-height: low = green (ground), high = red (obstacle)
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (z_norm * 255).astype(np.uint8)  # R
    colors[:, 1] = ((1 - z_norm) * 200 + 55).astype(np.uint8)  # G
    colors[:, 2] = 30

    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.08)


def _costmap_cloud_override(cloud: Any) -> Any:
    """Render SimplePlanner's costmap_cloud — the blocked grid cells
    (with inflation) that A* actually treats as obstacles. Big red
    boxes so they pop against the terrain clouds.
    """
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None
    colors = np.full((len(points), 3), [255, 40, 40], dtype=np.uint8)
    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.12)


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


def _terrain_map_ext_override(cloud: Any) -> Any:
    """Render extended terrain map — persistent accumulated cloud."""
    return cloud.to_rerun(colormap="viridis", size=0.06)


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
    """Render the current waypoint goal as a visible marker."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _VIS_LIFT]],
        colors=[(255, 50, 50)],
        radii=0.4,
    )


def _goal_override(msg: Any) -> Any:
    """Render the current navigation goal as a large purple sphere."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _VIS_LIFT]],
        colors=[(180, 60, 220)],
        radii=0.6,
    )


def _free_paths_override(cloud: Any) -> Any:
    """Render LocalPlanner free (collision-free) candidate paths in vehicle frame."""
    import rerun as rr

    return [
        ("world/free_paths", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/free_paths", cloud.to_rerun(colormap="cool", size=0.02)),
    ]


def _static_floor(rr: Any) -> list[Any]:
    """Static ground plane at z=0 as a solid textured quad."""

    s = 50.0  # half-size
    return [
        rr.Mesh3D(
            vertex_positions=[[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]],
            triangle_indices=[[0, 1, 2], [0, 2, 3]],
            vertex_colors=[[40, 40, 40, 120]] * 4,
        )
    ]


# ─── Debug overrides (elevated paths for top-down debugging) ─────────────────


def _waypoint_override_debug(msg: Any) -> Any:
    """Agentic debug: waypoint elevated above the scene."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _AGENTIC_DEBUG_LIFT]],
        colors=[(255, 50, 50)],
        radii=0.4,
    )


def _goal_override_debug(msg: Any) -> Any:
    """Agentic debug: goal elevated above the scene."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + _AGENTIC_DEBUG_LIFT]],
        colors=[(180, 60, 220)],
        radii=0.6,
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
