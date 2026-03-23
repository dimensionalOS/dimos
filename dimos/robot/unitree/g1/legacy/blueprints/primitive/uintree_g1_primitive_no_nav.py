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

from typing import Any

from dimos_lcm.sensor_msgs import CameraInfo

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.module import CameraModule  # type: ignore[attr-defined]
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.hardware.sensors.camera.zed import compat as zed
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(voxel_size=0.1, mode="boxes")


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.2, 0.15, 0.75],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _g1_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        ),
    )


rerun_config = {
    "blueprint": _g1_rerun_blueprint,
    "pubsubs": [LCM()],
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/global_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}

if global_config.viewer == "foxglove":
    from dimos.robot.foxglove_bridge import FoxgloveBridge

    _with_vis = autoconnect(FoxgloveBridge.blueprint())
elif global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import RerunBridgeModule, _resolve_viewer_mode

    _with_vis = autoconnect(
        RerunBridgeModule.blueprint(viewer_mode=_resolve_viewer_mode(), **rerun_config)
    )
else:
    _with_vis = autoconnect()


def _create_webcam() -> Webcam:
    return Webcam(
        camera_index=0,
        fps=15,
        stereo_slice="left",
        camera_info=zed.CameraInfo.SingleWebcam,
    )


_camera = (
    autoconnect(
        CameraModule.blueprint(
            transform=Transform(
                translation=Vector3(0.05, 0.0, 0.6),  # height of camera on G1 robot
                rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
                frame_id="sensor",
                child_frame_id="camera_link",
            ),
            hardware=_create_webcam,
        ),
    )
    if not global_config.simulation
    else autoconnect()
)

uintree_g1_primitive_no_nav = (
    autoconnect(
        _with_vis,
        _camera,
        VoxelGridMapper.blueprint(voxel_size=0.1),
        CostMapper.blueprint(),
        WavefrontFrontierExplorer.blueprint(),
        # Visualization
        WebsocketVisModule.blueprint(),
    )
    .global_config(n_workers=4, robot_model="unitree_g1")
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
