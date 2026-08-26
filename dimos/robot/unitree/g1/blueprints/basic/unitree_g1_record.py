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

"""Drive-and-record blueprint for the G1.

Rerun viewer keyboard teleop walks the robot while Point-LIO odom+lidar, the
RealSense color+depth streams, and tf are recorded into a timestamped folder
under recordings/.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.robot.unitree.g1.g1_recorder import G1Recorder
from dimos.robot.unitree.g1.g1_tf_publisher import G1TfPublisher
from dimos.visualization.vis_module import vis_module


def _g1_record_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.Spatial2DView(origin="world/realsense_depth_image", name="Depth"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                line_grid=rrb.LineGrid3D(visible=False),
            ),
            column_shares=[1, 2],
        ),
    )


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.20, 0.7],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _convert_camera_info(camera_info: CameraInfo) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_color_optical_frame",
    )


def _convert_depth_camera_info(camera_info: CameraInfo) -> Any:
    # Depth is aligned to color, so it shares the color optical frame.
    return camera_info.to_rerun(
        image_topic="/world/realsense_depth_image",
        optical_frame="camera_color_optical_frame",
    )


_record_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "blueprint": _g1_record_rerun_blueprint,
        "visual_override": {
            "world/realsense_camera_info": _convert_camera_info,
            "world/realsense_depth_camera_info": _convert_depth_camera_info,
        },
        # G1-sized wireframe on the live base_link frame.
        "static": {"world/robot_body": _static_robot_body},
        "tf_axes": 0.5,
        "memory_limit": "1GB",
    },
)


unitree_g1_record = autoconnect(
    MovementManager.blueprint(),
    G1HighLevelDdsSdk.blueprint(),
    PointLio.blueprint(
        frame_id="world",
        host_ip="192.168.123.164",
        lidar_ip="192.168.123.120",
    ).remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    # The camera anchors its optical tf subtree to the d435_link frame.
    RealSenseCamera.blueprint(
        base_frame_id="d435_link",
    ).remappings(
        [
            (RealSenseCamera, "depth_image", "realsense_depth_image"),
            (RealSenseCamera, "camera_info", "realsense_camera_info"),
            (RealSenseCamera, "depth_camera_info", "realsense_depth_camera_info"),
        ]
    ),
    G1Recorder.blueprint(),
    # Mount frames onto tf, base_link edge live from the waist joints.
    G1TfPublisher.blueprint(),
    # Viewer keyboard teleop feeds MovementManager via tele_cmd_vel.
    _record_vis,
).global_config(n_workers=12, robot_model="unitree_g1", record=True)


if __name__ == "__main__":
    coordinator = ModuleCoordinator.build(unitree_g1_record)
    coordinator.loop()
