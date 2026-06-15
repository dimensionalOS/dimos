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

"""Go2 3D nav stack: fastlio odometry, raytraced voxel map, MLS planning, path following.

Expects a mid360 + fastlio running against a lidar mounted on the robot.
The go2's built-in lidar and odometry are remapped aside and unused for
navigation. The map, paths, and robot pose all live in fastlio's world frame.
"""

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

voxel_size = 0.1
# Height of the head-mounted lidar above the ground while standing.
go2_lidar_height = 0.5


def _render_global_map(msg: Any) -> Any:
    # don't use a min z height because why would we want that
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner clears its plan with an empty path on every start-pose change.
    # Logging those would blank the line. Drop them so the last path stays shown.
    if len(msg.poses) == 0:
        return None
    return msg


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box on fastlio's body frame, counter-rotated for the lidar pitch."""
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        rr.Transform3D(
            parent_frame="tf#/body",
            rotation=rr.RotationAxisAngle(axis=(0, 1, 0), degrees=-45.0),
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
    # frame. Anchor the robot box to fastlio's body frame instead and hide the
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
    GO2Connection.blueprint(lidar=False, camera=False).remappings(
        [
            (GO2Connection, "lidar", "lidar_l1"),
            (GO2Connection, "odom", "odom_go2"),
        ]
    ),
    FastLio2.blueprint(
        host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
        lidar_ip=os.getenv("LIDAR_IP", "192.168.1.155"),
        map_freq=-1.0,
    ).remappings([(FastLio2, "global_map", "global_map_fastlio")]),
    RayTracingVoxelMap.blueprint(
        voxel_size=voxel_size, emit_every=1, global_emit_every=1500, max_health=5
    ),
    # global_map is remapped off so the planner runs purely on the
    # incremental local_map + region_bounds pair.
    MLSPlannerNative.blueprint(
        world_frame="odom",
        voxel_size=voxel_size,
        robot_height=go2_lidar_height,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    GoalRelay.blueprint(),
    BasicPathFollower.blueprint(lookahead_m=0.5, heading_gain=0.8, max_angular=0.6),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2")

__all__ = ["unitree_go2_nav_3d"]
