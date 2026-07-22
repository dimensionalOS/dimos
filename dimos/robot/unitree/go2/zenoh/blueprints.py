#!/usr/bin/env python3
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

"""Go2 blueprints for a robot running the go2web zenoh bridge.

The same stack as ``unitree_go2_nav_3d``, minus the two modules the robot now runs
itself: no WebRTC ``GO2Connection`` and no local ``PointLio`` — odom, lidar and video
arrive over zenoh already (see :class:`GO2Zenoh`), which is why the graphs run on the
zenoh transport.

Three layers, each a superset of the one above it, so a failure can be bisected by
dropping down a level:

- ``go2-zenoh-basic`` — streams plus teleop. Confirms the bridge, the tf tree and the
  camera without any mapping.
- ``go2-zenoh-raycaster`` — adds :class:`RayTracingVoxelMap`, so the voxel maps can be
  eyeballed before a planner consumes them.
- ``go2-zenoh-nav`` — the full stack: planner, goal relay and path follower.
"""

import math
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.odom_body_frame import OdomBodyFrame
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08
# Raise above 0 to see what the planner searched over: its traversable surface, the graph
# nodes on it and the edges between them, coloured green->red by cost. Drives both the
# planner's publishing and the rerun overrides. 2.0 is a good working rate.
planner_viz_hz = 0.0

# The lidar mount rotation on this rig (fixed-axis rpy, degrees), feeding both the static
# tf GO2Zenoh publishes and the rotation that levels its odometry — they must agree or nav
# steers off-heading. 60 deg of tilt, carried on roll because the lidar sits yawed 90 deg
# on its bracket; verified against Point-LIO's own attitude on the standing robot.
MID360_MOUNT_RPY_DEG = (-60.0, 0.0, -90.0)


def _mount_rotation() -> list[float]:
    """base_link <- lidar rotation, so nav reads odometry in the level body frame.

    base_link -> front_camera carries no rotation, so the composed base_link -> mid360_link
    rotation is just the mount rpy above.
    """
    rpy = Vector3(*(math.radians(d) for d in MID360_MOUNT_RPY_DEG))
    return list(Quaternion.from_euler(rpy).to_tuple())


def _camera_info_to_pinhole(camera_info: Any) -> Any:
    """Log the pinhole onto the video's entity instead of camera_info's own.

    The rerun bridge names entities after topics, so camera_info and video land on
    sibling paths — and a Pinhole only projects the entity it lives on and that entity's
    children, which is why the frustum draws but stays empty. No ``optical_frame`` here:
    the video carries frame_id ``camera_optical``, so the bridge already anchors that
    entity to the tf frame, and a second parent would be rejected.
    """
    return camera_info.to_rerun(image_topic="world/video")


def _rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world, as the WebRTC go2 blueprint has.

    The 2D view sits on ``world/video`` rather than ``world/color_image`` — over zenoh the
    camera arrives as an H.264 ``CompressedVideo`` on the ``video`` port, decoded in the
    viewer, and that is also where the pinhole is logged.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/video", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def _convert_map(grid: PointCloud2) -> Any:
    return grid.to_rerun(voxel_size=0.025)


def _convert_scan(grid: PointCloud2) -> Any:
    return grid.to_rerun(color=[255, 0, 0], voxel_size=0.05)


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


# What the bridge itself sends, present in all three blueprints.
_BRIDGE_OVERRIDE = {
    "world/camera_info": _camera_info_to_pinhole,
    "world/pointlio_map": _convert_map,
    "world/lidar": _convert_scan,
}
# The clouds are what costs: the world maps run up to 300k points x 16 B per message at
# ~2 Hz, the per-scan lidar ~10 Hz. Deliberately no entry for world/video — the throttle
# drops whole messages, and dropping H.264 samples breaks decoding until the next
# keyframe, so the camera has to be slowed at the source instead. (0 here would mean
# unlimited, not off.)
_BRIDGE_MAX_HZ: dict[str, float] = {
    #    "world/lidar": 2.0,
    #    "world/pointlio_map": 0.5,
}

_MAP_OVERRIDE = {"world/global_map": _render_global_map}
_MAP_MAX_HZ = {
    # Already rate-limited at the source by global_emit_every, roughly every 5s.
    "world/global_map": 0,
    "world/local_map": 0.5,
}

_NAV_OVERRIDE = {
    "world/path": _render_path,
    **planner_visual_override(planner_viz_hz),
}


def _rerun_config(
    visual_override: dict[str, Any] | None = None, max_hz: dict[str, Any] | None = None
) -> dict[str, Any]:
    """The bridge's own view, plus whatever the layer above it adds."""
    return {
        "blueprint": _rerun_blueprint,
        "visual_override": {**_BRIDGE_OVERRIDE, **(visual_override or {})},
        "max_hz": {**_BRIDGE_MAX_HZ, **(max_hz or {})},
    }


# Bridge streams + teleop only: no mapping, no planning. cmd_vel still reaches the robot
# through MovementManager, so this is the layer to drive from when something upstream is
# suspect.
go2_zenoh_basic = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config()),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=4, robot_model="unitree_go2")

# Adds the voxel map. It consumes GO2Zenoh's lidar + odometry directly: the bridge stamps
# them exactly as PointLio does locally (frames odom / mid360_link, xyz+intensity at
# point_step 16).
_ray_tracer = RayTracingVoxelMap.blueprint(
    voxel_size=voxel_size,
    emit_every=1,
    global_emit_every=50,
    min_health=-1,
    max_health=5,
    support_min=4,
)

go2_zenoh_raycaster = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer, rerun_config=_rerun_config(_MAP_OVERRIDE, _MAP_MAX_HZ)
    ),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    _ray_tracer,
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=6, robot_model="unitree_go2")

go2_zenoh_nav = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config({**_MAP_OVERRIDE, **_NAV_OVERRIDE}, _MAP_MAX_HZ),
    ),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    # Level the tilted-sensor odometry into the body frame so the follower steers on a
    # true heading. The ray tracer keeps the raw sensor odometry.
    OdomBodyFrame.blueprint(mount_rotation=_mount_rotation()),
    _ray_tracer,
    # global_map is remapped off so the planner runs purely on the
    # incremental local_map + region_bounds pair.
    MLSPlannerNative.blueprint(
        world_frame="odom",
        voxel_size=voxel_size,
        robot_height=0.3,
        surface_closing_radius=0.3,
        wall_clearance_m=0.1,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=4.0,
        viz_publish_hz=planner_viz_hz,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    GoalRelay.blueprint(),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6).remappings(
        [(BasicPathFollower, "odometry", "body_odometry")]
    ),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=8, robot_model="unitree_go2")
