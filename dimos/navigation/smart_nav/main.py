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
PGO, click-to-goal, cmd-vel mux), with optional TARE exploration and
GlobalMapUpdater accumulator.

`smart_nav_rerun_config(user_config)` returns a Rerun config dict with the
SmartNav defaults filled in via setdefault — pass it to `RerunBridgeModule`
or `vis_module` separately.

Defaults match the onboard (real hardware) configuration. Override any
module's config via per-module kwarg dicts (e.g.
`terrain_analysis={"obstacle_height_threshold": 0.1}`).
"""

from __future__ import annotations

import logging
from typing import Any

from dimos.core.blueprints import Blueprint, autoconnect

logger = logging.getLogger(__name__)
from dimos.navigation.cmd_vel_mux import CmdVelMux
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smart_nav.modules.global_map_updater.global_map_updater import (
    GlobalMapUpdater,
)
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.navigation.smart_nav.modules.simple_planner.simple_planner import SimplePlanner
from dimos.navigation.smart_nav.modules.tare_planner.tare_planner import TarePlanner
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smart_nav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM


def smart_nav(
    *,
    use_tare: bool = False,
    use_global_map_updater: bool = False,
    use_terrain_map_ext: bool = True,
    use_simple_planner: bool = False,
    vehicle_height: float | None = None,
    terrain_analysis: dict[str, Any] | None = None,
    terrain_map_ext: dict[str, Any] | None = None,
    local_planner: dict[str, Any] | None = None,
    path_follower: dict[str, Any] | None = None,
    far_planner: dict[str, Any] | None = None,
    simple_planner: dict[str, Any] | None = None,
    pgo: dict[str, Any] | None = None,
    click_to_goal: dict[str, Any] | None = None,
    cmd_vel_mux: dict[str, Any] | None = None,
    tare_planner: dict[str, Any] | None = None,
    global_map_updater: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose a SmartNav autoconnect Blueprint with the given options.

    Core external streams (always present regardless of toggles):

        registered_scan: In[PointCloud2]   — world-frame lidar scan
        odometry:        In[Odometry]      — raw SLAM odometry
        clicked_point:   In[PointStamped]  — click-to-goal from UI
        joy_cmd:         In[Twist]         — optional joystick override
        tele_cmd_vel:    In[Twist]         — optional teleop command
        agent_cmd_vel:   In[TwistDuration] — optional skill/agent command

        cmd_vel:            Out[Twist]        — final velocity command (CmdVelMux)
        corrected_odometry: Out[Odometry]     — PGO loop-closure-corrected pose
        global_map:         Out[PointCloud2]  — PGO accumulated keyframe map
        terrain_map:        Out[PointCloud2]  — TerrainAnalysis ground/obstacle grid
        path:               Out[Path]         — LocalPlanner's chosen local path
        goal_path:          Out[Path]         — FAR planner's global path
        way_point:          Out[PointStamped] — current waypoint target
        goal:               Out[PointStamped] — current navigation goal
        stop_movement:      Out[Bool]         — stop signal from CmdVelMux

    Args:
        use_tare: Add the TARE frontier-based exploration planner. Auto-remaps
            ClickToGoal's `way_point` output so TARE has exclusive control of
            LocalPlanner's waypoint input.
        use_global_map_updater: Add the bounded-memory voxel accumulator
            (GlobalMapUpdater) on top of registered_scan.
        use_terrain_map_ext: Add TerrainMapExt — the persistent extended terrain
            accumulator used for visualization and wider-range planning.
        vehicle_height: Ignore terrain points above this height (m). Threaded
            into TerrainAnalysis's `vehicle_height` config. Defaults to 1.2m.
        terrain_analysis, terrain_map_ext, local_planner, path_follower,
        far_planner, pgo, click_to_goal, cmd_vel_mux, tare_planner,
        global_map_updater: Per-module config override dicts. Merged on top
        of the SmartNav defaults.

    Returns:
        An autoconnected Blueprint with the selected modules wired together.
    """
    terrain_analysis_config = {**(terrain_analysis or {})}
    local_planner_config = {**(local_planner or {})}
    terrain_analysis_threshold = terrain_analysis_config.get("obstacle_height_threshold", 0.2)
    local_planner_threshold = local_planner_config.get("obstacle_height_threshold", 0.2)
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
                "scan_voxel_size": 0.15,
                # Voxel grid
                "terrain_voxel_size": 1.0,
                "terrain_voxel_half_width": 10,
                # Obstacle/ground classification
                "obstacle_height_threshold": 0.2,
                "ground_height_threshold": 0.1,
                "min_relative_z": -1.5,
                "max_relative_z": 1.5,
                "use_sorting": True,
                "quantile_z": 0.25,
                # Decay and clearing
                "decay_time": 2.0,
                "no_decay_distance": 1.5,
                "clearing_distance": 8.0,
                "clear_dynamic_obstacles": True,
                "no_data_obstacle": False,
                "no_data_block_skip_count": 0,
                "min_block_point_count": 10,
                # Voxel culling
                "voxel_point_update_threshold": 30,
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
                "vehicle_height": 1.2 if vehicle_height is None else vehicle_height,
                **(terrain_analysis or {}),
            }
        ),
        LocalPlanner.blueprint(
            **{
                "autonomy_mode": True,
                "use_terrain_analysis": True,
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "obstacle_height_threshold": 0.2,
                "max_relative_z": 1.5,
                "min_relative_z": -1.5,
                **(local_planner or {}),
            }
        ),
        PathFollower.blueprint(
            **{
                "autonomy_mode": True,
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "max_acceleration": 2.0,
                "slow_down_distance_threshold": 0.2,
                "omni_dir_goal_threshold": 0.5,
                **(path_follower or {}),
            }
        ),
        *(
            [SimplePlanner.blueprint(**(simple_planner or {}))]
            if use_simple_planner
            else [FarPlanner.blueprint(**{"sensor_range": 15.0, **(far_planner or {})})]
        ),
        PGO.blueprint(**(pgo or {})),
        ClickToGoal.blueprint(**(click_to_goal or {})),
        CmdVelMux.blueprint(**(cmd_vel_mux or {})),
    ]
    if use_terrain_map_ext:
        modules.append(
            TerrainMapExt.blueprint(
                **{
                    "voxel_size": 0.3,
                    # Walls are static — keep them around long enough that
                    # a global planner (SimplePlanner / FarPlanner) doesn't
                    # see freshly-empty cells behind the robot and route
                    # paths straight through the walls it can't currently
                    # see. 8 s was way too aggressive for that.
                    "decay_time": 300.0,
                    "publish_rate": 2.0,
                    "max_range": 40.0,
                    **(terrain_map_ext or {}),
                }
            )
        )
    if use_tare:
        modules.append(TarePlanner.blueprint(**(tare_planner or {})))
    if use_global_map_updater:
        modules.append(GlobalMapUpdater.blueprint(**(global_map_updater or {})))

    remappings = [
        # PathFollower cmd_vel → CmdVelMux nav input (avoid collision with mux output)
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        # Global-scale planners use PGO-corrected odometry (per CMU ICRA 2022):
        # loop-closure adjustments go to high-level planners; local modules
        # care only about the local environment and work in the odom frame.
        (
            SimplePlanner if use_simple_planner else FarPlanner,
            "odometry",
            "corrected_odometry",
        ),
        (ClickToGoal, "odometry", "corrected_odometry"),
        (TerrainAnalysis, "odometry", "corrected_odometry"),
        # FAR (or TARE) owns way_point — disconnect ClickToGoal's output.
        (ClickToGoal, "way_point", "_click_way_point_unused"),
        (PGO, "global_map", "global_map_pgo"),
    ]

    return autoconnect(*modules).remappings(remappings)


# ─── Rerun visual overrides (robot-agnostic) ─────────────────────────────────


def smart_nav_rerun_config(
    user_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a rerun config dict with SmartNav defaults filled in via setdefault.

    The caller's entries win — this just ensures missing keys (blueprint,
    pubsubs, visual_override entries, static entries) are
    populated with the SmartNav defaults.
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
    visual_override.setdefault("world/way_point", _waypoint_override)
    visual_override.setdefault("world/goal", _goal_override)
    visual_override.setdefault("world/goal_path", _goal_path_override)
    visual_override.setdefault("world/obstacle_cloud", _obstacle_cloud_override)
    visual_override.setdefault("world/costmap_cloud", _costmap_cloud_override)
    resolved["visual_override"] = visual_override
    static_entries = dict(resolved["static"])
    static_entries.setdefault("world/floor", _static_floor)
    resolved["static"] = static_entries
    return resolved


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
    pts = [[float(p[0]), float(p[1]), float(p[2]) + 0.05] for p in points]
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

    points = [[p.x, p.y, p.z + 0.3] for p in path_msg.poses]
    return [
        ("world/nav_path", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/nav_path", rr.LineStrips3D([points], colors=[(0, 255, 128)], radii=0.05)),
    ]


def _goal_path_override(path_msg: Any) -> Any:
    """Render FAR planner's planned path: orange line + yellow node markers."""
    import rerun as rr

    if not path_msg.poses or len(path_msg.poses) < 2:
        return None

    points = [[p.x, p.y, p.z] for p in path_msg.poses]
    return [
        # Edges: orange line connecting all waypoints
        ("world/goal_path/edges", rr.LineStrips3D([points], colors=[(255, 140, 0)], radii=0.04)),
        # Nodes: yellow spheres at each graph node in the path
        ("world/goal_path/nodes", rr.Points3D(points, colors=[(255, 255, 0)], radii=0.12)),
    ]


def _waypoint_override(msg: Any) -> Any:
    """Render the current waypoint goal as a visible marker."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + 0.5]],
        colors=[(255, 50, 50)],
        radii=0.3,
    )


def _goal_override(msg: Any) -> Any:
    """Render the current navigation goal as a large purple sphere."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + 0.5]],
        colors=[(180, 60, 220)],
        radii=0.6,
    )


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
