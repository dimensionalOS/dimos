# Copyright 2025-2026 Dimensional Inc.
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

"""R1 Pro 3D navigation on Point-LIO: lidar + lidar-anchored head depth into one ray-traced map: ``dimos run r1pro-nav``."""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.experimental.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.experimental.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.viz import planner_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloud
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_lidar_odometry,
)
from dimos.robot.galaxea.r1pro.head_cameras import HeadLeftCameraConfig
from dimos.robot.galaxea.r1pro.head_depth import r1pro_head_depth
from dimos.robot.galaxea.r1pro.lio import R1ProLioConfig
from dimos.visualization.vis_module import vis_module

# First-pass R1 Pro clearances; tune on the robot.
CHASSIS_WIDTH_M = 0.65
ROTATION_DIAMETER_M = 0.9
OVERHEAD_CLEARANCE_M = 1.9
MAX_STEP_HEIGHT_M = 0.03

VOXEL_SIZE_M = 0.07
WALL_CLEARANCE_M = 0.3
PLANNER_VIZ_HZ = 0.0

# Head depth kept only in the band from the floor to just above the lidar's plane, what the lidar sees worst;
# dropping (not snapping) anything under the floor, since nothing can be there.
HEAD_CLOUD_MIN_HEIGHT_M = -0.15
HEAD_CLOUD_MAX_HEIGHT_M = 0.35

# The ray tracer shares the Orin with everything else; head depth at one point per 11x11 pixels, at most 1500,
# keeps it near one Point-LIO's worth of work with every lidar ray traced. Live, heavier settings fell ~10 s
# behind; a cloud older than MAX_CLOUD_AGE_S is skipped so it can never work through a stale queue.
HEAD_CLOUD_DECIMATION = 11
HEAD_CLOUD_MAX_POINTS = 1500
MAX_CLOUD_AGE_S = 1.0


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg


# Both clouds share the `lidar` port, so the viewer splits them by frame_id.
_CLOUD_COLOR_UNKNOWN = [170, 170, 170]


def _render_cloud(msg: Any) -> Any:
    import rerun as rr

    frame_id = getattr(msg, "frame_id", "") or "unknown"
    path = f"world/lidar/{frame_id}"
    xyz = msg.points_f32()
    head_frame = HeadLeftCameraConfig.model_fields["frame_id"].default
    lidar_frame = R1ProLioConfig.model_fields["lidar_frame"].default
    if frame_id in (head_frame, lidar_frame) and len(xyz):
        # Rainbow by the frame's own up axis (the optical frame's is -y), inverted so dark blue is never on black.
        up = -xyz[:, 1] if frame_id == head_frame else xyz[:, 2]
        level = 1.0 - np.clip((up - up.min()) / max(float(np.ptp(up)), 1e-3), 0.0, 1.0)
        colors = np.stack(
            [255 * level, 255 * (1 - np.abs(2 * level - 1)), 255 * (1 - level)], axis=1
        )
        points = rr.Points3D(xyz, colors=(0.35 * 255 + 0.65 * colors).astype(np.uint8), radii=0.02)
    else:
        points = msg.to_rerun(colors=_CLOUD_COLOR_UNKNOWN, rgb=False)
    # A list skips the bridge's frame attach, so each cloud hangs off its own tf frame here.
    return [(path, points), (path, rr.Transform3D(parent_frame=f"tf#/{frame_id}"))]


_MAX_HZ: dict[str, float] = {
    "world/local_map": 0.5,
    "world/lidar": 0.2,
}

_rerun_config = {
    "tf_axes": 0.35,
    # Keep the viewer's bandwidth off the Orin.
    "max_hz": _MAX_HZ,
    "visual_override": {
        "world/head_left_color": None,
        "world/head_right_color": None,
        "world/wrist_left_color": None,
        "world/wrist_right_color": None,
        # Full-resolution intrinsics sharing the depth image's frame would give it the wrong pinhole.
        "world/head_left_info": None,
        "world/head_right_info": None,
        "world/lidar": _render_cloud,
        # Raw sensor streams, heavy over a laptop's link and shown better by what is built from them.
        "world/lidar_raw": None,
        "world/imu": None,
        "world/imu_torso": None,
        "world/r1pro/imu": None,
        "world/region_bounds": None,
        "world/planner_path": _render_path,
        "world/path": None,
        **planner_visual_override(PLANNER_VIZ_HZ, VOXEL_SIZE_M, WALL_CLEARANCE_M),
    },
}


r1pro_nav = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
    # Point-LIO owns odom -> base_link, so the connection's wheel odometry stays off tf.
    r1pro_control(publish_odom_tf=False, enable_wrist_color=False, stop_vendor_lidar=True),
    r1pro_lidar_odometry(),
    r1pro_head_depth(
        min_height_m=HEAD_CLOUD_MIN_HEIGHT_M,
        max_height_m=HEAD_CLOUD_MAX_HEIGHT_M,
        decimation=HEAD_CLOUD_DECIMATION,
        max_points=HEAD_CLOUD_MAX_POINTS,
    ).remappings([(Depth2DepthCloud, "depth_cloud", "lidar")]),
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE_M,
        max_range=10.0,
        max_cloud_age_s=MAX_CLOUD_AGE_S,
        emit_every=1,
        global_emit_every=50,
        support_min=4,
        worker_threads=3,
        # The Livox cloud arrives ~0.11 s behind its stamp; 0.1 dropped clouds on jitter.
        tf_match_tolerance_s=0.25,
    ),
    MLSPlannerNative.blueprint(
        voxel_size=VOXEL_SIZE_M,
        robot_height=OVERHEAD_CLEARANCE_M,
        start_z_offset_m=0.0,
        wall_clearance_m=WALL_CLEARANCE_M,
        wall_buffer_m=CHASSIS_WIDTH_M,
        wall_buffer_weight=100.0,
        step_threshold_m=MAX_STEP_HEIGHT_M,
        step_penalty_weight=4.0,
        viz_publish_hz=PLANNER_VIZ_HZ,
        worker_threads=2,
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    DanLocalPlanner.blueprint(
        lock_replan=0.4,
        # Keep MLS's 3D waypoints; the 2D resampler zeroes every Z.
        resample_spacing_m=0.0,
    ).remappings([(DanLocalPlanner, "odom", "chassis_odom")]),
    DanHolonomicTC.blueprint(control_frequency=10.0).remappings(
        [(DanHolonomicTC, "odom", "chassis_odom")]
    ),
    MovementManager.blueprint(),
).global_config(
    n_workers=8,
    robot_width=CHASSIS_WIDTH_M,
    robot_rotation_diameter=ROTATION_DIAMETER_M,
)
