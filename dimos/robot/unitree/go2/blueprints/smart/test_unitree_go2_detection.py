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

from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_detection import unitree_go2_detection

# Wire topics LCM consumers already listen on, keyed exactly the way the
# coordinator resolves transports: (stream_name, message_type).
EXPECTED_TOPICS = {
    ("detections_2d", Detection2DArray): "/detector3d/detections",
    ("detections_3d", Detection3DArray): "/detector3d/detections_3d",
    ("detected_pointcloud_0", PointCloud2): "/detector3d/pointcloud/0",
    ("detected_pointcloud_1", PointCloud2): "/detector3d/pointcloud/1",
    ("detected_pointcloud_2", PointCloud2): "/detector3d/pointcloud/2",
    ("detected_image_0", Image): "/detector3d/image/0",
    ("detected_image_1", Image): "/detector3d/image/1",
    ("detected_image_2", Image): "/detector3d/image/2",
}


def test_detector_transports_resolve_at_runtime() -> None:
    """The coordinator resolves transports through a (stream_name, message_type)
    key. A key authored in any other shape (e.g. a module class as the second
    element) never matches; the lookup then falls back silently and the stream
    publishes on an auto topic like /detections_2d instead of the authored wire
    topic. Indexing transport_map by that key fails loudly here if a transport
    was authored under the wrong shape."""
    for (name, msg_type), topic in EXPECTED_TOPICS.items():
        transport = unitree_go2_detection.transport_map[(name, msg_type)]
        assert transport.topic.topic == topic
        assert transport.topic.lcm_type is msg_type
