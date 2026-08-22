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
from dimos.perception.detection.module3D import Detection3DModule
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _topic_path(topic: object) -> str:
    topic_str = getattr(topic, "name", None) or str(topic)
    raw = getattr(topic, "topic", topic_str)
    if isinstance(raw, str):
        topic_str = raw
    topic_str = topic_str.split("#")[0]
    if topic_str.startswith("dimos/"):
        topic_str = "/" + topic_str.removeprefix("dimos/")
    elif not topic_str.startswith("/"):
        topic_str = "/" + topic_str
    return topic_str


def _detection_topic_to_entity(topic: object) -> str:
    path = _topic_path(topic)
    parts = path.strip("/").split("/")
    if len(parts) == 4 and parts[:2] == ["detector3d", "3d"]:
        return f"world/detections/3d/{parts[2]}"
    return f"world{path}"


detection_rerun_config = {
    **rerun_config,
    "topic_to_entity": _detection_topic_to_entity,
}

unitree_go2_detection = (
    autoconnect(
        unitree_go2,
        # Replaces the RerunBridgeModule already present in unitree_go2 while
        # leaving its single vis_module bundle and websocket modules intact.
        RerunBridgeModule.blueprint(**detection_rerun_config),
        Detection3DModule.blueprint(
            camera_info=GO2Connection.camera_info_static,
            publish_detection_images=False,
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
            ("detections", Detection2DArray): LCMTransport(
                "/detector3d/detections", Detection2DArray
            ),
            ("detected_pointcloud_0", PointCloud2): LCMTransport(
                "/detector3d/3d/slot_0/pointcloud", PointCloud2
            ),
            ("detected_pointcloud_1", PointCloud2): LCMTransport(
                "/detector3d/3d/slot_1/pointcloud", PointCloud2
            ),
            ("detected_pointcloud_2", PointCloud2): LCMTransport(
                "/detector3d/3d/slot_2/pointcloud", PointCloud2
            ),
            ("detected_3d_image_0", Image): LCMTransport("/detector3d/3d/slot_0/image", Image),
            ("detected_3d_image_1", Image): LCMTransport("/detector3d/3d/slot_1/image", Image),
            ("detected_3d_image_2", Image): LCMTransport("/detector3d/3d/slot_2/image", Image),
        }
    )
)
