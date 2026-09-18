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

"""Habitat blueprints, layered so a failure can be bisected by dropping a level.

- ``habitat-teleop``: sim, streams, teleop.
- ``habitat-raycaster``: + :class:`RayTracingVoxelMap` on a sensor-frame scan.
- ``habitat-nav``: + MLS planner and follower; click the surface to set a goal.
- ``habitat-voxel``: :class:`VoxelGridMapper` on a world-frame scan. An alternative,
  not a layer: the two mappers want the scan in different frames.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.simulation.habitat.connection import HabitatConnection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

# One stream, two port names: remap rather than rename.
SCAN_TOPIC = "habitat_scan"

# Teleop and the follower publish straight onto cmd_vel. No MovementManager: it
# cancels the goal on the zero twist the viewer sends on every key release.
CMD_VEL_TOPIC = "cmd_vel"
# The viewer's clicked point is already the planner's goal.
GOAL_TOPIC = "goal"

# VoxelGridMapper wants world-frame clouds; RayTracingVoxelMap registers via tf
# and raytraces from the sensor origin, so it needs the sensor frame.
SCAN_FRAME_REGISTERED = "world"
SCAN_FRAME_SENSOR = "camera_optical"

WORLD_FRAME = "world"
voxel_size = 0.05
# Must be > 0: the planner's surface_map is what you click to set a goal.
planner_viz_hz = 2.0
ROBOT_HEIGHT = 0.5

# Hidden, not dropped: still tickable in the viewer.
HIDDEN = ("world/nodes", "world/depth_image")


def _small_points(cloud: Any) -> Any:
    """Flat dots; mode is explicit so this does not track to_rerun's default."""
    return cloud.to_rerun(mode="points", ui_radius=1.0)


def _render_path(msg: Any) -> Any:
    """Skip empty paths so a failed plan keeps the last good one drawn."""
    return None if len(msg.poses) == 0 else msg


def _view() -> Any:
    """3D view anchored on the world frame, camera and depth beside it."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
                overrides={p: rrb.EntityBehavior(visible=False) for p in HIDDEN},
            ),
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image"),
                rrb.Spatial2DView(origin="world/depth_image"),
            ),
            column_shares=[3, 1],
        ),
    )


def _rerun_config(extra: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "blueprint": _view,
        "tf_axes": 0.3,
        "visual_override": {
            f"world/{SCAN_TOPIC}": _small_points,
            "world/global_map": _small_points,
            "world/local_map": _small_points,
            **(extra or {}),
        },
    }


# The native is zenoh-only, so every layer pins the transport.
habitat_teleop = autoconnect(
    HabitatConnection.blueprint(publish_scan=False),
    vis_module(global_config.viewer, rerun_config=_rerun_config()).remappings(
        [(RerunWebSocketServer, "tele_cmd_vel", CMD_VEL_TOPIC)]
    ),
).global_config(transport="zenoh")


habitat_voxel = autoconnect(
    HabitatConnection.blueprint(publish_scan=True, scan_frame=SCAN_FRAME_REGISTERED).remappings(
        [(HabitatConnection, "registered_scan", SCAN_TOPIC)]
    ),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, frame_id=WORLD_FRAME).remappings(
        [(VoxelGridMapper, "lidar", SCAN_TOPIC)]
    ),
    vis_module(global_config.viewer, rerun_config=_rerun_config()).remappings(
        [(RerunWebSocketServer, "tele_cmd_vel", CMD_VEL_TOPIC)]
    ),
).global_config(transport="zenoh")


_ray_tracing_config = RayTracingVoxelMapConfig(
    voxel_size=voxel_size,
    world_frame=WORLD_FRAME,
    emit_every=1,
    global_emit_every=50,
    min_health=-1,
    max_health=5,
    support_min=4,
)

# Planner runs on local_map + region_bounds only, as on the robot.
_mls_planner = MLSPlannerNative.blueprint(
    **MLSPlannerNativeConfig(
        world_frame=WORLD_FRAME,
        voxel_size=voxel_size,
        robot_height=ROBOT_HEIGHT,
        start_z_offset_m=0.0,  # base_link sits on the navmesh
        surface_closing_radius=0.3,
        wall_clearance_m=0.1,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=4.0,
        viz_publish_hz=planner_viz_hz,
    ).model_dump(exclude_unset=True)
).remappings([(MLSPlannerNative, "global_map", "global_map_unused")])


# Newest duplicate wins: re-declared with the sensor-frame scan.
habitat_raycaster = autoconnect(
    habitat_teleop,
    HabitatConnection.blueprint(publish_scan=True, scan_frame=SCAN_FRAME_SENSOR).remappings(
        [(HabitatConnection, "registered_scan", SCAN_TOPIC)]
    ),
    RayTracingVoxelMap.blueprint(**_ray_tracing_config.model_dump(exclude_unset=True)).remappings(
        [(RayTracingVoxelMap, "lidar", SCAN_TOPIC)]
    ),
).global_config(transport="zenoh")


# Click the surface to set a goal; a held key overrides the follower.
habitat_nav = autoconnect(
    habitat_raycaster,
    _mls_planner,
    # Defaults to odom; the sim's tf root is world.
    BasicPathFollower.blueprint(
        world_frame=WORLD_FRAME, speed=0.5, heading_gain=1.5, max_angular=1.5
    ).remappings([(BasicPathFollower, "nav_cmd_vel", CMD_VEL_TOPIC)]),
    vis_module(
        global_config.viewer,
        rerun_config=_rerun_config(
            {
                "world/path": _render_path,
                **planner_visual_override(
                    planner_viz_hz, voxel_size=voxel_size, wall_clearance_m=0.1
                ),
            }
        ),
    ).remappings(
        [
            (RerunWebSocketServer, "tele_cmd_vel", CMD_VEL_TOPIC),
            (RerunWebSocketServer, "clicked_point", GOAL_TOPIC),
        ]
    ),
).global_config(transport="zenoh")
