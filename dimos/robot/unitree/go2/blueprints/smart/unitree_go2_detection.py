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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_detection = (
    autoconnect(
        unitree_go2,
        Detection3DModule.blueprint(
            camera_info=GO2Connection.camera_info_static,
        ),
    )
    .remappings(
        [
            (Detection3DModule, "pointcloud", "global_map"),
        ]
    )
    .transports(
        {
            # Detection 3D module outputs
            ("detections_2d", Detection2DArray): LCMTransport(
                "/detector3d/detections", Detection2DArray
            ),
            ("detections_3d", Detection3DArray): LCMTransport(
                "/detector3d/detections_3d", Detection3DArray
            ),
            ("detected_pointcloud_0", PointCloud2): LCMTransport(
                "/detector3d/pointcloud/0", PointCloud2
            ),
            ("detected_pointcloud_1", PointCloud2): LCMTransport(
                "/detector3d/pointcloud/1", PointCloud2
            ),
            ("detected_pointcloud_2", PointCloud2): LCMTransport(
                "/detector3d/pointcloud/2", PointCloud2
            ),
            ("detected_image_0", Image): LCMTransport("/detector3d/image/0", Image),
            ("detected_image_1", Image): LCMTransport("/detector3d/image/1", Image),
            ("detected_image_2", Image): LCMTransport("/detector3d/image/2", Image),
        }
    )
)

__all__ = ["unitree_go2_detection"]
