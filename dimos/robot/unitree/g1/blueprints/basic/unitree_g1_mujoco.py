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

"""G1 MuJoCo simulation stack: visualization + mapping + MuJoCo connection + planner.

This is the new-architecture equivalent of the legacy ``unitree_g1_basic_sim``
blueprint.  It uses the shared ``_vis`` / ``_mapper`` primitives and the
``G1SimConnection`` module which wraps :class:`MujocoConnection`.
"""

import math
from typing import Any

from dimos_lcm.sensor_msgs import CameraInfo as LCMCameraInfo

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.g1.blueprints.primitive._mapper import _mapper
from dimos.robot.unitree.g1.blueprints.primitive._vis import (
    _convert_camera_info,
    _convert_global_map,
    _convert_navigation_costmap,
    _static_base_link,
)
from dimos.robot.unitree.g1.legacy.sim import G1SimConnection
from dimos.simulation.mujoco.constants import VIDEO_CAMERA_FOV, VIDEO_HEIGHT, VIDEO_WIDTH
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _static_mujoco_pinhole(rr: Any) -> list[Any]:
    """Pinhole + transform for the MuJoCo head camera.

    The MuJoCo camera sits at roughly [0.05, 0, 0.4] on the G1 torso.
    Resolution and FOV come from :mod:`dimos.simulation.mujoco.constants`.
    """
    fovy_rad = math.radians(VIDEO_CAMERA_FOV)
    fy = (VIDEO_HEIGHT / 2.0) / math.tan(fovy_rad / 2.0)
    fx = fy  # square pixels
    cx, cy = VIDEO_WIDTH / 2.0, VIDEO_HEIGHT / 2.0
    return [
        rr.Pinhole(
            resolution=[VIDEO_WIDTH, VIDEO_HEIGHT],
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


_vis_mujoco = vis_module(
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
            "world/color_image": _static_mujoco_pinhole,
        },
    },
)

unitree_g1_mujoco = (
    autoconnect(
        _vis_mujoco,
        _mapper,
        WebsocketVisModule.blueprint(),
        G1SimConnection.blueprint(),
        ReplanningAStarPlanner.blueprint(),
    )
    .global_config(n_workers=4, robot_model="unitree_g1")
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("color_image", Image): LCMTransport("/color_image", Image),
            ("camera_info", LCMCameraInfo): LCMTransport("/camera_info", LCMCameraInfo),
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("path", Path): LCMTransport("/path", Path),
            ("goal_reached", Bool): LCMTransport("/goal_reached", Bool),
            ("goal_request", PoseStamped): LCMTransport("/goal_request", PoseStamped),
            ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
        }
    )
)

__all__ = ["unitree_g1_mujoco"]
