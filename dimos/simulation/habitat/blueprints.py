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

- ``habitat-teleop`` — sim plus streams and teleop, no mapping.
- ``habitat-raycaster`` — adds :class:`RayTracingVoxelMap` on a sensor-frame scan.
- ``habitat-nav`` — adds the MLS planner and follower; click the surface to set a goal.
- ``habitat-voxel`` — the :class:`VoxelGridMapper` alternative, on a pre-registered
  scan. Not a layer: the two mappers want the scan in different frames.
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

# The connection's scan port and VoxelGridMapper's input are the same stream under
# two names, so remap rather than rename either port.
SCAN_TOPIC = "habitat_scan"

# Viewer teleop and the path follower both publish straight onto the sim's
# cmd_vel. No MovementManager: its arbitration is real-robot safety behaviour that
# fights the viewer here -- releasing a movement key publishes a zero twist, which
# latches _teleop_active, cancels the active goal with a NaN and then blocks
# nav_cmd_vel for the cooldown. Manual driving worked; navigation never moved.
CMD_VEL_TOPIC = "cmd_vel"
# The viewer's clicked point IS the planner's goal, so it needs no translation.
GOAL_TOPIC = "goal"

# VoxelGridMapper assumes world-frame clouds. RayTracingVoxelMap needs the sensor
# frame instead: it raytraces from the sensor origin and registers the cloud itself
# via the tf lookup world_frame -> cloud frame_id.
SCAN_FRAME_REGISTERED = "world"
SCAN_FRAME_SENSOR = "camera_optical"

WORLD_FRAME = "world"
voxel_size = 0.05
# The planner's surface_map is what you click to set a goal, so this cannot be 0:
# with viz off there is nothing in the 3D view to click and no goal is ever sent.
planner_viz_hz = 2.0

# Habitat's agent sits on the navmesh, so base_link is already at floor level and
# needs no start_z offset -- unlike a real robot, whose base_link rides above it.
ROBOT_HEIGHT = 0.5


# Hidden rather than dropped: still in the entity tree, tickable in the viewer.
HIDDEN = ("world/nodes",)


def _small_points(cloud: Any) -> Any:
    """Flat dots at half the default radius; a dense RGB-D scan is a lot of them.

    ``mode`` is explicit so this does not depend on to_rerun's default.
    """
    return cloud.to_rerun(mode="points", ui_radius=1.0)


def _camera_info_to_pinhole(camera_info: Any) -> Any:
    """Log the pinhole onto the colour image's entity, not camera_info's own.

    Entities are named after topics, so the two land on sibling paths and a
    Pinhole only projects its own entity and its children -- without this the
    frustum draws but stays empty and the image is placed by whatever transform
    its entity inherits. No ``optical_frame``: the image's frame_id already
    anchors it and a second parent is rejected.
    """
    return camera_info.to_rerun(image_topic="world/color_image")


def _render_path(msg: Any) -> Any:
    """Drop empty paths: the planner emits one when it finds no route, and logging
    it would blank the line rather than leave the last good path up."""
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
            "world/camera_info": _camera_info_to_pinhole,
            f"world/{SCAN_TOPIC}": _small_points,
            "world/global_map": _small_points,
            "world/local_map": _small_points,
            **(extra or {}),
        },
    }


habitat_teleop = autoconnect(
    HabitatConnection.blueprint(publish_scan=False),
    vis_module(global_config.viewer, rerun_config=_rerun_config()).remappings(
        [(RerunWebSocketServer, "tele_cmd_vel", CMD_VEL_TOPIC)]
    ),
)


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
)


_ray_tracing_config = RayTracingVoxelMapConfig(
    voxel_size=voxel_size,
    world_frame=WORLD_FRAME,
    emit_every=1,
    global_emit_every=50,
    min_health=-1,
    max_health=5,
    support_min=4,
)

# global_map is remapped off so the planner runs on the incremental local_map +
# region_bounds pair, as it does on the robot.
_mls_planner = MLSPlannerNative.blueprint(
    **MLSPlannerNativeConfig(
        world_frame=WORLD_FRAME,
        voxel_size=voxel_size,
        robot_height=ROBOT_HEIGHT,
        start_z_offset_m=0.0,
        surface_closing_radius=0.3,
        wall_clearance_m=0.1,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=4.0,
        viz_publish_hz=planner_viz_hz,
    ).model_dump(exclude_unset=True)
).remappings([(MLSPlannerNative, "global_map", "global_map_unused")])


# Re-declares HabitatConnection with the sensor-frame scan the raycaster needs;
# autoconnect keeps the newest duplicate, so this wins over habitat_teleop's.
habitat_raycaster = autoconnect(
    habitat_teleop,
    HabitatConnection.blueprint(publish_scan=True, scan_frame=SCAN_FRAME_SENSOR).remappings(
        [(HabitatConnection, "registered_scan", SCAN_TOPIC)]
    ),
    RayTracingVoxelMap.blueprint(**_ray_tracing_config.model_dump(exclude_unset=True)).remappings(
        [(RayTracingVoxelMap, "lidar", SCAN_TOPIC)]
    ),
)


# A goal is a click on the planner's surface_map in the 3D view. The follower's
# nav_cmd_vel lands on the same cmd_vel the viewer's keys drive, so a keypress just
# overrides the follower for as long as it is held.
habitat_nav = autoconnect(
    habitat_raycaster,
    _mls_planner,
    # world_frame defaults to "odom", which nothing here publishes: the sim's tf
    # root is `world`. Left wrong, the follower never resolves the robot pose and
    # silently emits no cmd_vel.
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
)
