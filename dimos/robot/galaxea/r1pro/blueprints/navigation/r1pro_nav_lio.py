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

"""3D navigation for the R1 Pro on lidar-inertial odometry.

The robot is placed by Point-LIO on the chassis Mid-360 (``r1pro-pointlio``),
not by the speed the chassis was told to go. Two clouds feed one voxel map:
Point-LIO's own deskewed scan for everything the lidar can see, and the head's
stereo depth (``r1pro-stereo``) cut down to the band the lidar cannot -- the
floor and what stands on it below the lidar's plane, out to a few metres. The
map is ray traced so a cleared obstacle clears, the MLS planner plans a 3D
path across it, and the holonomic local planner and controller drive it.

Usage, on the robot::

    dimos run r1pro-nav-lio --g.transport lcm

and on a recording made with ``r1pro-recorder`` (both eyes and infos,
Point-LIO's lidar and odometry, tf)::

    dimos run r1pro-nav-lio-replay --dataset <recording>

Run on ``lcm``: the C++ estimator's cloud does not register in the Rust voxel
map over zenoh (see the R1 README). Point-LIO and the stereo matcher are
native binaries built on first run.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloud
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_pointlio import r1pro_lidar_odometry
from dimos.robot.galaxea.r1pro.constants import (
    CHASSIS_WIDTH_M,
    MAX_STEP_HEIGHT_M,
    OVERHEAD_CLEARANCE_M,
    ROTATION_DIAMETER_M,
)
from dimos.robot.galaxea.r1pro.lio import BASE_FRAME, ODOM_FRAME
from dimos.robot.galaxea.r1pro.replay import R1ProReplay
from dimos.robot.galaxea.r1pro.stereo import HEAD_CAMERA_FRAME, r1pro_stereo_cloud
from dimos.visualization.vis_module import vis_module

VOXEL_SIZE_M = 0.05
PLANNER_VIZ_HZ = 0.0

# The band of the head's depth the map takes, in metres above base_link.
#
# The Mid-360 sits 0.29 m up and sees 7 degrees below its own plane, so the
# floor only enters its view 2.4 m out, and a box on the floor at 1 m is
# invisible to it. The head, 1.6 m up and looking 20 degrees down, sees
# exactly that ground from about 1 m out. Everything the head sees ABOVE the
# lidar's plane the lidar sees better -- it is metrically exact where stereo is
# noisy to centimetres -- so the head's cloud is cut to the lidar's blind band
# and no higher. The floor itself is kept with a little slack below zero for
# the stereo's own scatter and a tilted floor.
HEAD_CLOUD_MIN_HEIGHT_M = -0.15
HEAD_CLOUD_MAX_HEIGHT_M = 0.35


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg


# The two clouds share the `lidar` port, so in the viewer they arrive on one
# entity and there is no telling which returns came from which sensor -- which
# is the whole question when you are checking whether the stereo depth agrees
# with the lidar. frame_id is what distinguishes them on the wire, so use it:
# one colour and one entity path each, so either can be toggled on its own.
_CLOUD_COLORS = {
    "lidar_pointlio_link": [80, 160, 255],
    "lidar_chassis_left_link": [80, 160, 255],
    HEAD_CAMERA_FRAME: [255, 140, 40],
}
_CLOUD_COLOR_UNKNOWN = [170, 170, 170]


def _render_cloud(msg: Any) -> Any:
    frame_id = getattr(msg, "frame_id", "") or "unknown"
    color = _CLOUD_COLORS.get(frame_id, _CLOUD_COLOR_UNKNOWN)
    return [(f"world/lidar/{frame_id}", msg.to_rerun(colors=color, rgb=False))]


_rerun_config = {
    "tf_axes": 0.35,
    # The viewer is not what the Orin is for. A 480x384 float depth frame is
    # 737 KB, so even 1 Hz of it is 5.9 Mbit/s; the maps and the depth they
    # feed are what this view is for, at a rate that shows they are sane.
    "max_hz": {
        "world/local_map": 0.5,
        "world/head_depth": 0.5,
        "world/lidar": 0.2,
    },
    "visual_override": {
        "world/head_left_color": None,
        "world/head_right_color": None,
        "world/wrist_left_color": None,
        "world/wrist_right_color": None,
        "world/lidar": _render_cloud,
        "world/planner_path": _render_path,
        "world/path": None,
        **planner_visual_override(PLANNER_VIZ_HZ),
    },
}


def _nav_stack() -> Blueprint:
    """Everything downstream of the sensors, shared by the live and replay runs.

    Carries no sensor source, so it is composed with one that supplies
    ``lidar``, ``head_left_color``, ``head_right_color``, ``head_left_info``,
    ``head_right_info``, ``motor_states``, ``chassis_odom`` and ``tf``.
    """
    return autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        # The head's depth, cut to the lidar's blind band, onto the same fan-in
        # bus as the lidar. Each cloud keeps its own frame_id.
        r1pro_stereo_cloud(
            min_height_m=HEAD_CLOUD_MIN_HEIGHT_M,
            max_height_m=HEAD_CLOUD_MAX_HEIGHT_M,
        ).remappings([(StereoCloud, "cloud", "lidar")]),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=10.0,
            ray_subsample=10,
            # Point-LIO publishes its cloud at 10 Hz; this is "do not thin it".
            # The cap exists for the day the Orin falls behind registering
            # clouds: the tell is clouds dropped for want of a transform with
            # the staleness parked at the tf window, and this is the number to
            # lower when that shows up in the log.
            max_cloud_rate_hz=10.0,
            emit_every=1,
            global_emit_every=50,
            # And never longer than this in cloud time, however few clouds
            # arrive, or a slow stretch of input stretches the global map's
            # cadence from seconds to never.
            global_max_interval_s=20.0,
            support_min=4,
            world_frame=ODOM_FRAME,
            worker_threads=3,
            # The Livox driver's stamps arrive a consistent ~0.11 s in the past
            # (sensor to driver to publish latency, not skew); the default 0.1
            # sat right on that and dropped clouds on jitter alone.
            tf_match_tolerance_s=0.25,
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
}


r1pro_nav_lio = autoconnect(
    # The connection's own odometry comes off: Point-LIO owns `odom -> base_link`
    # now, and two publishers of one edge is a tf tree whose answer depends on
    # arrival order. The wrists cost a JPEG decode each and nothing here reads
    # them. See dimos.robot.galaxea.r1pro.lio.
    r1pro_control(publish_odom=False, enable_wrist_color=False),
    r1pro_lidar_odometry(),
    _nav_stack(),
).global_config(n_workers=8, **_shared_global_config)


r1pro_nav_lio_replay = autoconnect(R1ProReplay.blueprint(), _nav_stack()).global_config(
    n_workers=5,
    # Recorders sit out a replay rather than overwrite the recording driving it.
    replay=True,
    **_shared_global_config,
)
