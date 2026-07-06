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

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative

# Local planner disabled for testing; re-enable with the RepulsiveFieldNative block below.
# from dimos.navigation.nav_3d.repulsive_local_planner.repulsive_field_native import (
#     RepulsiveFieldNative,
# )
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connect_relative_sensor import RelativeSensorConnection
from dimos.robot.unitree.go2.go2_mid360_static_transforms import FRAMES
from dimos.visualization.vis_module import vis_module

voxel_size = 0.04
# Height of the head-mounted lidar above the ground while standing.
go2_lidar_height = 0.5

# base_link -> mid360_link from the measured mount tree; inverted so the onboard
# L1 cloud (landed in base_link) is re-expressed in the Mid-360 frame Point-LIO
# owns, letting both clouds share the pointlio_lidar stream.
_mount_edges = {edge.child_frame_id: edge for edge in frames_to_edge_transforms(FRAMES)}
_base_link_to_mid360 = _mount_edges["front_camera"] + _mount_edges["mid360_link"]
_mid360_from_base_link = _base_link_to_mid360.inverse().to_matrix().flatten().tolist()


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box on pointlio's sensor frame, counter-rotated for the lidar pitch."""
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        rr.Transform3D(
            parent_frame="tf#/mid360_link",
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
    # frame. Anchor the robot box to pointlio's sensor frame instead and hide the
    # camera frustum that rides base_link.
    "static": {"world/tf/mid360_link": _static_robot_body},
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
    # "mcf" for stair traversal. The onboard L1 cloud is re-expressed into the
    # Mid-360 frame and shares the pointlio_lidar stream with Point-LIO.
    RelativeSensorConnection.blueprint(
        lidar=True,
        camera=False,
        motion_mode="mcf",
        odom_frame_id="go2_odom",
        base_frame="go2_base_link",
        lidar_frame="mid360_link",
        transform=_mid360_from_base_link,
    ).remappings(
        [
            # Merged (Mid-360 + re-expressed L1) cloud consumed by the ray-tracing map.
            (RelativeSensorConnection, "lidar", "pointlio_lidar"),
            # Listen to Point-LIO's raw Mid-360 cloud so it can be folded into the L1
            # scan before publishing. A distinct topic from `lidar` above avoids a
            # self-feedback loop where the merged output re-enters the merge.
            (RelativeSensorConnection, "pointlio_lidar", "pointlio_raw"),
            (RelativeSensorConnection, "odom", "odom_go2"),
        ]
    ),
    PointLio.blueprint().remappings([(PointLio, "lidar", "pointlio_raw")]),
    RayTracingVoxelMap.blueprint(
        voxel_size=voxel_size,
        emit_every=1,
        global_emit_every=50,
        max_health=10,
        graze_cos=0.85,
    ).remappings([(RayTracingVoxelMap, "lidar", "pointlio_lidar")]),
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
    # Local planner disabled for testing — BasicPathFollower follows the MLS
    # global route (`path`) directly instead of a smoothed local path.
    # Smooths the MLS global route into an obstacle-avoiding local path, solving
    # fresh on the ray-tracing terrain cloud (its own internal costmap — no
    # CostMapper needed). output_base_frame=False: BasicPathFollower consumes a
    # world-frame path + odometry, not a vehicle-frame route.
    # RepulsiveFieldNative.blueprint(
    #     world_frame="odom",
    #     output_base_frame=False,
    # ).remappings(
    #     [
    #         (RepulsiveFieldNative, "terrain_map", "local_map"),
    #         (RepulsiveFieldNative, "global_path", "path"),
    #     ]
    # ),
    # Follow the MLS global route directly (`path` is BasicPathFollower's default topic).
    BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)
