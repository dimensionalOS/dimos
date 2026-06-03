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

"""G1 stack with person tracking and 3D detection."""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.zed import compat as zed
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.moduleDB import ObjectDBModule
from dimos.perception.detection.person_tracker import PersonTracker
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_basic import unitree_g1_basic


def _person_only(det: Any) -> bool:
    return bool(det.class_id == 0)


unitree_g1_detection = (
    autoconnect(
        unitree_g1_basic,
        # Person detection modules with YOLO
        Detection3DModule.blueprint(
            camera_info=zed.CameraInfo.SingleWebcam,
            detector=YoloPersonDetector,
        ),
        ObjectDBModule.blueprint(
            camera_info=zed.CameraInfo.SingleWebcam,
            filter=_person_only,  # Filter for person class only
        ),
        PersonTracker.blueprint(
            cameraInfo=zed.CameraInfo.SingleWebcam,
        ),
    )
    .global_config(n_workers=8)
    .remappings(
        [
            # Connect detection modules to camera and lidar
            (ObjectDBModule, "image", "color_image"),
            (ObjectDBModule, "pointcloud", "pointcloud"),
            (PersonTracker, "image", "color_image"),
            (PersonTracker, "detections", "detections_2d"),
        ]
    )
    .transports(
        {
            # Detection 3D module outputs
            ("detections_2d", Detection3DModule): LCMTransport(
                "/detector3d/detections", Detection2DArray
            ),
            ("detections_3d", Detection3DModule): LCMTransport(
                "/detector3d/detections_3d", Detection3DArray
            ),
            ("detected_pointcloud_0", Detection3DModule): LCMTransport(
                "/detector3d/pointcloud/0", PointCloud2
            ),
            ("detected_pointcloud_1", Detection3DModule): LCMTransport(
                "/detector3d/pointcloud/1", PointCloud2
            ),
            ("detected_pointcloud_2", Detection3DModule): LCMTransport(
                "/detector3d/pointcloud/2", PointCloud2
            ),
            ("detected_image_0", Detection3DModule): LCMTransport("/detector3d/image/0", Image),
            ("detected_image_1", Detection3DModule): LCMTransport("/detector3d/image/1", Image),
            ("detected_image_2", Detection3DModule): LCMTransport("/detector3d/image/2", Image),
            # Detection DB module outputs
            ("detections_2d", ObjectDBModule): LCMTransport(
                "/detectorDB/detections", Detection2DArray
            ),
            ("detections_3d", ObjectDBModule): LCMTransport(
                "/detectorDB/detections_3d", Detection3DArray
            ),
            ("detected_pointcloud_0", ObjectDBModule): LCMTransport(
                "/detectorDB/pointcloud/0", PointCloud2
            ),
            ("detected_pointcloud_1", ObjectDBModule): LCMTransport(
                "/detectorDB/pointcloud/1", PointCloud2
            ),
            ("detected_pointcloud_2", ObjectDBModule): LCMTransport(
                "/detectorDB/pointcloud/2", PointCloud2
            ),
            ("detected_image_0", ObjectDBModule): LCMTransport("/detectorDB/image/0", Image),
            ("detected_image_1", ObjectDBModule): LCMTransport("/detectorDB/image/1", Image),
            ("detected_image_2", ObjectDBModule): LCMTransport("/detectorDB/image/2", Image),
            # Person tracker outputs
            ("target", PersonTracker): LCMTransport("/person_tracker/target", PoseStamped),
        }
    )
)

__all__ = ["unitree_g1_detection"]
