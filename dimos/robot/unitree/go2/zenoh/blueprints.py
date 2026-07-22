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

"""3d navigation on a Go2 that runs the go2web zenoh bridge.

The same nav stack as ``unitree_go2_nav_3d``, minus the two modules the robot now runs
itself: no WebRTC ``GO2Connection`` and no local ``PointLio`` — odom, lidar and video
arrive over zenoh already (see :class:`GO2Zenoh`), which is why the graph runs on the
zenoh transport.
"""

import math
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2Zenoh
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08

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
    return grid.to_rerun(voxel_size=0.04)


def _convert_scan(grid: PointCloud2) -> Any:
    return grid.to_rerun(color=[255, 0, 0], voxel_size=0.05)


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/camera_info": _camera_info_to_pinhole,
        "world/pointlio_map": _convert_map,
        "world/lidar": _convert_scan,
    },
    # The clouds are what costs: the world map runs up to 300k points x 16 B per message
    # at ~2 Hz, the per-scan lidar ~10 Hz. Deliberately no entry for world/video — the
    # throttle drops whole messages, and dropping H.264 samples breaks decoding until the
    # next keyframe, so the camera has to be slowed at the source instead. (0 here would
    # mean unlimited, not off.)
    "max_hz": {
        #        "world/lidar": 2.0,
        #        "world/pointlio_map": 0.5,
    },
}

unitree_go2_nav_3d_zenoh = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
    GO2Zenoh.blueprint(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG),
    # Level the tilted-sensor odometry into the body frame so the follower steers on a
    # true heading. The ray tracer keeps the raw sensor odometry.
    # #    OdomBodyFrame.blueprint(mount_rotation=_mount_rotation()),
    RayTracingVoxelMap.blueprint(
        voxel_size=voxel_size,
        emit_every=1,
        global_emit_every=50,
        min_health=-1,
        max_health=5,
        support_min=4,
    ),
    #     # global_map is remapped off so the planner runs purely on the
    #     # incremental local_map + region_bounds pair.
    #     MLSPlannerNative.blueprint(
    #         world_frame="odom",
    #         voxel_size=voxel_size,
    #         robot_height=0.3,
    #         surface_closing_radius=0.3,
    #         wall_clearance_m=0.1,
    #         wall_buffer_m=0.75,
    #         wall_buffer_weight=100.0,
    #         step_threshold_m=0.16,
    #         step_penalty_weight=4.0,
    #         viz_publish_hz=0.0,
    #     ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    # #    GoalRelay.blueprint(),
    # #    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6).remappings(
    # #        [(BasicPathFollower, "odometry", "body_odometry")]
    # #    ),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=8, robot_model="unitree_go2")
