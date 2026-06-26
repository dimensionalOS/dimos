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

"""3d navigation on Go2 with ray tracing and MLS planning"""

import math
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.hack import PointLioHack
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.urdf_loader import UrdfLoader
from dimos.visualization.vis_module import vis_module

_navigation_dir = Path(__file__).parent
# The physical mid-360 mount (rotated_urdf) and the "forward = forward" mount the
# rest of the stack pretends it has (normal_urdf). PointLioHack rewrites the cloud
# from the first into the second, so everything downstream uses the normal mount.
_rotated_urdf = _navigation_dir / "go2_mid360_rotated.urdf"
_normal_urdf = _navigation_dir / "go2_mid360_normal.urdf"

# GO2Connection is a TfModule; feeding it the normal mount publishes those frames
# on its static interval, matching the cloud the hack emits.
go2_mid360_model = UrdfLoader(name="go2_mid360", model_path=_normal_urdf)

voxel_size = 0.08
# Height of the head-mounted lidar above the ground while standing.
go2_lidar_height = 0.5

# The lidar is physically pitched down 45 deg and rolled 45 deg, so pointlio's
# body frame is tilted. Counter-rotate the robot body box by the inverse of that
# mount so it renders level (flip a sign here if the box leans the wrong way).
_body_box_roll_deg = -45.0
_body_box_pitch_deg = -45.0
_body_box_yaw_deg = 0.0
_body_box_rotation = Quaternion.from_euler(
    Vector3(
        math.radians(_body_box_roll_deg),
        math.radians(_body_box_pitch_deg),
        math.radians(_body_box_yaw_deg),
    )
)


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box + forward arrow on pointlio's body frame, counter-rotated
    so the physically pitched/rolled lidar mount renders level."""
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        # Red arrow out the nose marks the robot's forward (+x) direction.
        rr.Arrows3D(
            origins=[(0.35, 0.0, 0.0)],
            vectors=[(0.3, 0.0, 0.0)],
            colors=[(255, 40, 40)],
        ),
        rr.Transform3D(
            parent_frame="tf#/body",
            rotation=_body_box_rotation.to_rerun(),
        ),
    ]


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        "world/global_map": 1.0,
        "world/local_map": 1.0,
    },
    "memory_limit": "256MB",
    # base_link tf comes from the go2 internal odometry, which is not the map
    # frame. Anchor the robot box to pointlio's body frame instead and hide the
    # camera frustum that rides base_link.
    "static": {"world/tf/body": _static_robot_body},
    "visual_override": {
        **rerun_config["visual_override"],
        "world/global_map": _render_global_map,
        "world/path": _render_path,
        "world/camera_info": None,
        "world/color_image": None,
        "world/lidar": None,
        "world/surface_map": None,
        "world/nodes": None,
        "world/node_edges": None,
    },
}

unitree_go2_nav_3d = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
    # "mcf" for stair traversal
    GO2Connection.blueprint(
        static_transforms=dict(go2_mid360_model.static_transforms),
        lidar=False,
        camera=False,
        motion_mode="mcf",
    ).remappings(
        [
            (GO2Connection, "lidar", "lidar_l1"),
            (GO2Connection, "odom", "odom_go2"),
        ]
    ),
    # gravity_align is off (no_gravity_align.yaml) so pointlio leaves the odometry
    # in the raw mount frame; the hack applies the known rotated->normal transform
    # instead. Only the lidar is rerouted through the hack; odometry flows straight
    # to consumers.
    PointLio.blueprint(body_frame_id="body", config="no_gravity_align.yaml").remappings(
        [
            (PointLio, "lidar", "rotated_lidar"),
        ]
    ),
    PointLioHack.blueprint(rotated_urdf=_rotated_urdf, normal_urdf=_normal_urdf),
    RayTracingVoxelMap.blueprint(
        voxel_size=voxel_size,
        emit_every=1,
        global_emit_every=50,
        max_health=10,
        graze_cos=0.85,
    ),
    # global_map is remapped off so the planner runs purely on the
    # incremental local_map + region_bounds pair.
    MLSPlannerNative.blueprint(
        world_frame="odom",
        voxel_size=voxel_size,
        robot_height=go2_lidar_height,
        wall_clearance_m=0.2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=1.0,
        viz_publish_hz=0.0,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    GoalRelay.blueprint(),
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)
