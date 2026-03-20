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

"""G1 with ROSNav in simulation mode (Unity).

Unlike the onboard blueprint, the sim variant does NOT include
G1HighLevelDdsSdk (which requires the Unitree SDK and real hardware).
In simulation the ROSNav container drives cmd_vel internally.
"""

import math
from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.rosnav.rosnav_module import ROSNav
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.g1.blueprints.primitive._mapper import _mapper
from dimos.robot.unitree.g1.blueprints.primitive._vis import (
    _convert_camera_info,
    _convert_global_map,
    _convert_navigation_costmap,
    _static_base_link,
    _static_path_frame,
)
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule, websocket_vis


def _static_sim_pinhole(rr: Any) -> list[Any]:
    """Pinhole + transform for the sim equirectangular camera.

    Connects ``world/color_image`` directly to ``tf#/base_link`` with the
    combined camera-link translation [0.05, 0, 0.6] and ROS optical-frame
    rotation so that Rerun can resolve the full transform chain to the view
    root without intermediate entity-path hops.
    """
    width, height = 1920, 640
    hfov_rad = math.radians(120.0)
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx  # square pixels
    cx, cy = width / 2.0, height / 2.0
    return [
        rr.Pinhole(
            resolution=[width, height],
            focal_length=[fx, fy],
            principal_point=[cx, cy],
            camera_xyz=rr.ViewCoordinates.RDF,
        ),
        rr.Transform3D(
            parent_frame="tf#/base_link",
            translation=[0.05, 0.0, 0.6],
            rotation=rr.Quaternion(xyzw=[0.5, -0.5, 0.5, -0.5]),
        ),
    ]


_vis_sim = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "pubsubs": [LCM()],
        "visual_override": {
            "world/camera_info": _convert_camera_info,
            "world/global_map": _convert_global_map,
            "world/navigation_costmap": _convert_navigation_costmap,
        },
        "static": {
            "world/tf/base_link": _static_base_link,
            "world/color_image": _static_sim_pinhole,
            "world/path": _static_path_frame,
        },
    },
)

unitree_g1_rosnav_sim = (
    autoconnect(
        _vis_sim,
        _mapper,
        websocket_vis(),
        ROSNav.blueprint(mode="simulation", vehicle_height=1.24),
    )
    .remappings(
        [
            (WebsocketVisModule, "cmd_vel", "teleop_cmd_vel"),
        ]
    )
    .global_config(n_workers=4, robot_model="unitree_g1")
)

__all__ = ["unitree_g1_rosnav_sim"]
