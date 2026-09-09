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

"""R1 Pro 3D navigation on two point clouds: chassis lidar + head stereo depth.

``r1pro-nav`` maps in 2D from the chassis lidar alone, which sits at 0.29 m and
sees nothing above it — a table edge or an open cabinet door is invisible to it.
This runs the 3D stack (raytracing voxel map -> MLS planner -> Dan local planner
-> holonomic tracker) and adds the head camera as a second obstacle source.

Both clouds publish onto the single ``lidar`` stream. That is a genuine fan-in
bus, and the raytracing module resolves ``frame_id`` through tf per message, so
each cloud is placed by its own frame. They are deliberately *not* merged:
``PointCloud2.__add__`` keeps only the first cloud's ``frame_id``, which would
silently cast the head camera's rays from the lidar's origin.

The head camera sits beyond the four revolute torso joints, so its pose is only
correct if it is recomputed from live joint angles. ``R1ProConnection`` publishes
that edge itself, off the same joint feedback it turns into ``motor_states``, so
nothing here has to know the robot's kinematics.

``r1pro-kronknav-replay`` swaps the robot for a recording and leaves every
planning module identical, so a plan that looks wrong on the replay is the same
bug as on the robot.

Usage:
    dimos run r1pro-kronknav --head-camera-info-path ./head_left.yaml
    dimos run r1pro-kronknav-replay --dataset ./r1pro_office.db
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.depth_cloud.module import DepthCloud
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.constants import (
    CHASSIS_WIDTH_M,
    MAX_STEP_HEIGHT_M,
    OVERHEAD_CLEARANCE_M,
    ROTATION_DIAMETER_M,
)
from dimos.robot.galaxea.r1pro.replay import R1ProReplay
from dimos.visualization.vis_module import vis_module

# Frames this blueprint plans in. They match R1ProConnectionConfig's defaults;
# override both sides together to run against a differently-framed robot.
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

VOXEL_SIZE_M = 0.05
PLANNER_VIZ_HZ = 0.0


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg


_rerun_config = {
    "tf_axes": 0.35,
    "max_hz": {"world/local_map": 0.5},
    "visual_override": {
        # The navigation view shows the maps, not the raw clouds feeding them.
        "world/lidar": None,
        "world/head_depth": None,
        "world/head_left_color": None,
        "world/head_right_color": None,
        "world/wrist_left_color": None,
        "world/wrist_right_color": None,
        "world/planner_path": _render_path,
        "world/path": None,
        **planner_visual_override(PLANNER_VIZ_HZ),
    },
}


def _nav_stack() -> Blueprint:
    """Everything downstream of the sensors, shared by the live and replay runs.

    Carries no sensor source, so it is composed with one that supplies
    ``lidar``, ``head_depth``, ``head_camera_info``, ``motor_states``,
    ``chassis_odom`` and ``tf``.
    """
    return autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        DepthCloud.blueprint(
            # 6 m is roughly where stereo range error exceeds a voxel.
            max_range_m=6.0,
        ).remappings(
            [
                (DepthCloud, "depth", "head_depth"),
                (DepthCloud, "camera_info", "head_camera_info"),
                # Fan in beside the chassis lidar; each keeps its own frame_id.
                (DepthCloud, "cloud", "lidar"),
            ]
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=15.0,
            ray_subsample=5,
            emit_every=1,
            global_emit_every=50,
            support_min=4,
            world_frame=ODOM_FRAME,
            worker_threads=3,
        ),
        MLSPlannerNative.blueprint(
            world_frame=ODOM_FRAME,
            base_frame=BASE_FRAME,
            voxel_size=VOXEL_SIZE_M,
            robot_height=OVERHEAD_CLEARANCE_M,
            start_z_offset_m=0.0,
            wall_clearance_m=0.3,
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
            # Preserve MLS's 3D waypoints; the 2D resampler zeroes every Z.
            resample_spacing_m=0.0,
        ).remappings([(DanLocalPlanner, "odom", "chassis_odom")]),
        DanHolonomicTC.blueprint(control_frequency=10.0).remappings(
            [(DanHolonomicTC, "odom", "chassis_odom")]
        ),
        MovementManager.blueprint(),
    )


_shared_global_config = {
    "robot_width": CHASSIS_WIDTH_M,
    "robot_rotation_diameter": ROTATION_DIAMETER_M,
    "transport": "zenoh",
}


r1pro_kronknav = autoconnect(r1pro_control(), _nav_stack()).global_config(
    n_workers=6, **_shared_global_config
)


r1pro_kronknav_replay = autoconnect(R1ProReplay.blueprint(), _nav_stack()).global_config(
    n_workers=5,
    # Recorders sit out a replay rather than overwrite the recording driving it.
    replay=True,
    **_shared_global_config,
)
