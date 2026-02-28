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

"""Minimal G1 stack without navigation, used as a base for larger blueprints."""

from dimos_lcm.sensor_msgs import CameraInfo

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera import zed
from dimos.hardware.sensors.camera.module import camera_module  # type: ignore[attr-defined]
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Twist, Vector3
from dimos.msgs.nav_msgs import Odometry, Path
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

rerun_config = {
    "pubsubs": [LCM(autoconf=True)],
    "visual_override": {
        "world/camera_info": lambda camera_info: camera_info.to_rerun(
            image_topic="/world/color_image",
            optical_frame="camera_optical",
        ),
        "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1, mode="boxes"),
        "world/navigation_costmap": lambda grid: grid.to_rerun(
            colormap="Accent",
            z_offset=0.015,
            opacity=0.2,
            background="#484981",
        ),
    },
    "static": {
        "world/tf/base_link": lambda rr: [
            rr.Boxes3D(
                half_sizes=[0.2, 0.15, 0.75],
                colors=[(0, 255, 127)],
                fill_mode="MajorWireframe",
            ),
            rr.Transform3D(parent_frame="tf#/base_link"),
        ]
    },
}

match global_config.viewer_backend:
    case "foxglove":
        from dimos.robot.foxglove_bridge import foxglove_bridge

        _with_vis = autoconnect(foxglove_bridge())
    case "rerun":
        from dimos.visualization.rerun.bridge import rerun_bridge

        _with_vis = autoconnect(rerun_bridge(**rerun_config))
    case "rerun-web":
        from dimos.visualization.rerun.bridge import rerun_bridge

        _with_vis = autoconnect(rerun_bridge(viewer_mode="web", **rerun_config))
    case _:
        _with_vis = autoconnect()

_camera = (
    autoconnect(
        camera_module(
            transform=Transform(
                translation=Vector3(0.05, 0.0, 0.6),  # height of camera on G1 robot
                rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
                frame_id="sensor",
                child_frame_id="camera_link",
            ),
            hardware=lambda: Webcam(
                camera_index=0,
                fps=15,
                stereo_slice="left",
                camera_info=zed.CameraInfo.SingleWebcam,
            ),
        ),
    )
    if not global_config.simulation
    else autoconnect()
)

uintree_g1_primitive_no_nav = (
    autoconnect(
        _with_vis,
        _camera,
        voxel_mapper(voxel_size=0.1),
        cost_mapper(),
        wavefront_frontier_explorer(),
        # Visualization
        websocket_vis(),
    )
    .global_config(n_dask_workers=4, robot_model="unitree_g1")
    .transports(
        {
            # G1 uses Twist for movement commands
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            # State estimation from ROS
            ("state_estimation", Odometry): LCMTransport("/state_estimation", Odometry),
            # Odometry output from ROSNavigationModule
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            # Navigation module topics from nav_bot
            ("goal_req", PoseStamped): LCMTransport("/goal_req", PoseStamped),
            ("goal_active", PoseStamped): LCMTransport("/goal_active", PoseStamped),
            ("path_active", Path): LCMTransport("/path_active", Path),
            ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("global_pointcloud", PointCloud2): LCMTransport("/map", PointCloud2),
            # Original navigation topics for backwards compatibility
            ("goal_pose", PoseStamped): LCMTransport("/goal_pose", PoseStamped),
            ("goal_reached", Bool): LCMTransport("/goal_reached", Bool),
            ("cancel_goal", Bool): LCMTransport("/cancel_goal", Bool),
            # Camera topics
            ("color_image", Image): LCMTransport("/color_image", Image),
            ("camera_info", CameraInfo): LCMTransport("/camera_info", CameraInfo),
        }
    )
)

__all__ = ["uintree_g1_primitive_no_nav"]
