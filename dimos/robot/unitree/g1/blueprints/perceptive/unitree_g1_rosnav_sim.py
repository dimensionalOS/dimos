#!/usr/bin/env python3
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

"""G1 with ROSNav + external Unity simulation.

The Unity simulator runs on the host (via UnityBridgeModule) and provides
lidar, camera, and odometry data.  ROSNav runs in hardware mode inside
Docker — its nav stack receives the external sensor data via ROS2 topics
republished by ROSNav's ext_* input streams.

cmd_vel flows back from the nav stack (or teleop) through LCM to the
UnityBridgeModule, which drives the simulated robot.
"""

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.core.global_config import global_config
from dimos.navigation.rosnav.rosnav_module import ROSNav
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.g1.blueprints.primitive._mapper import _mapper
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _static_path_frame(rr: Any) -> list[Any]:
    return [rr.Transform3D(parent_frame="tf#/sensor")]


def _static_base_link(rr: Any) -> list[Any]:
    """Green wireframe box tracking the robot.

    Attached to ``tf#/sensor`` because the UnityBridgeModule publishes
    ``map → sensor`` (there is no separate ``base_link`` frame in external
    sim mode).
    """
    return [
        rr.Boxes3D(
            half_sizes=[0.2, 0.15, 0.62],
            centers=[[0, 0, -0.62]],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/sensor"),
    ]


_vis_sim = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "pubsubs": [LCM()],
        "visual_override": {
            "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
        },
        "static": {
            "world/color_image": UnityBridgeModule.rerun_static_pinhole,
            "world/tf/base_link": _static_base_link,
            "world/path": _static_path_frame,
        },
    },
)

unitree_g1_rosnav_sim = (
    autoconnect(
        _vis_sim,
        _mapper,
        UnityBridgeModule.blueprint(),
        ROSNav.blueprint(mode="external_sim", vehicle_height=1.24, mount_sim_assets=True),
    )
    .remappings(
        [
            # Wire Unity sensor outputs → ROSNav external inputs.
            # Use "ext_*" names matching the UnityBridgeModule output names
            # to avoid colliding with ROSNav's own output streams of the same type.
            (UnityBridgeModule, "registered_scan", "ext_registered_scan"),
            (UnityBridgeModule, "odometry", "ext_odometry"),
            # Feed local terrain data from nav stack to Unity for Z-height adjustment
            # Rename VoxelGridMapper/CostMapper streams to avoid collisions
            (VoxelGridMapper, "lidar", "global_pointcloud"),
            (VoxelGridMapper, "global_map", "global_voxel_map"),
            (CostMapper, "global_map", "global_voxel_map"),
            # Teleop: WebsocketVisModule cmd_vel → ROSNav tele_cmd_vel
            (WebsocketVisModule, "cmd_vel", "tele_cmd_vel"),
        ]
    )
    .global_config(n_workers=4, robot_model="unitree_g1")
)

__all__ = ["unitree_g1_rosnav_sim"]
