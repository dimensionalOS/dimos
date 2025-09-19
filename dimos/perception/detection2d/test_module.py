# Copyright 2025 Dimensional Inc.
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
import pytest
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from dimos_lcm.sensor_msgs import Image, PointCloud2

from dimos.core import LCMTransport
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.msgs.sensor_msgs import PointCloud2 as PointCloud2Msg
from dimos.msgs.geometry_msgs import Transform, Vector3, PoseStamped
from dimos.perception.detection2d.conftest import Moment, dimos_cluster, publish_lcm
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map


def test_module2d(moment: Moment):
    detections2d = Detection2DModule().process_image_frame(moment["image_frame"])
    print(detections2d)

    # Print actual values for inspection
    print(f"\n=== test_module2d Output ===")
    print(f"Type: {type(detections2d)}")
    print(f"Number of detections: {len(detections2d)}")
    print(f"Image timestamp: {detections2d.image.ts}")
    print(f"Image shape: {detections2d.image.shape}")
    print(f"Image frame_id: {detections2d.image.frame_id}")

    if len(detections2d) > 0:
        det = detections2d.detections[0]
        print(f"\n--- First detection ---")
        print(f"Name: {det.name}")
        print(f"Class ID: {det.class_id}")
        print(f"Track ID: {det.track_id}")
        print(f"Confidence: {det.confidence}")
        print(f"Bbox: {det.bbox}")

    # Assertions for test_module2d
    assert isinstance(detections2d, ImageDetections2D)
    assert len(detections2d) == 1
    assert detections2d.image.ts == 1757960670.490248
    assert detections2d.image.shape == (720, 1280, 3)
    assert detections2d.image.frame_id == "camera_optical"

    # Check first detection
    det = detections2d.detections[0]
    assert isinstance(det, Detection2D)
    assert det.name == "suitcase"
    assert det.class_id == 28
    assert det.track_id == 1
    assert det.confidence == 0.8145349025726318

    # Check bbox values
    assert det.bbox == [503.437255859375, 249.89385986328125, 655.950439453125, 469.82879638671875]

    annotations = detections2d.to_image_annotations()
    publish_lcm({"annotations": annotations, **moment})


def test_module3d(moment: Moment):
    detections2d = Detection2DModule().process_image_frame(moment["image_frame"])
    pointcloud = moment["lidar_frame"]
    camera_transform = moment["tf"].get("camera_optical", "world")
    if camera_transform is None:
        raise ValueError("No camera_optical transform in tf")
    annotations = detections2d.to_image_annotations()

    detections3d = Detection3DModule(camera_info=moment["camera_info"]).process_frame(
        detections2d, pointcloud, camera_transform
    )

    publish_lcm(
        {
            **moment,
            "annotations": annotations,
            "detections": detections3d,
        }
    )

    print(detections3d)

    # Print actual values for inspection
    print(f"\n=== test_module3d Output ===")
    print(f"Type: {type(detections3d)}")
    print(f"Number of detections: {len(detections3d)}")
    print(f"Image timestamp: {detections3d.image.ts}")
    print(f"Image shape: {detections3d.image.shape}")
    print(f"Image frame_id: {detections3d.image.frame_id}")

    if len(detections3d) > 0:
        det = detections3d.detections[0]
        print(f"\n--- First 3D detection ---")
        print(f"Name: {det.name}")
        print(f"Class ID: {det.class_id}")
        print(f"Track ID: {det.track_id}")
        print(f"Confidence: {det.confidence}")
        print(f"Bbox: {det.bbox}")
        print(f"Pointcloud points: {len(det.pointcloud)}")
        print(f"Pointcloud frame_id: {det.pointcloud.frame_id}")
        print(f"Center: {det.center}")
        print(f"Pose: {det.pose}")

        # Check distance from repr_dict
        repr_dict = det.to_repr_dict()
        print(f"Distance: {repr_dict.get('dist', 'N/A')}")
        print(f"Points in repr: {repr_dict.get('points', 'N/A')}")

    # Assertions for test_module3d
    assert isinstance(detections3d, ImageDetections3D)
    assert len(detections3d) == 1
    assert detections3d.image.ts == 1757960670.490248
    assert detections3d.image.shape == (720, 1280, 3)
    assert detections3d.image.frame_id == "camera_optical"

    # Check first 3D detection
    det = detections3d.detections[0]
    assert isinstance(det, Detection3D)
    assert det.name == "suitcase"
    assert det.class_id == 28
    assert det.track_id == 1
    assert det.confidence == 0.8145349025726318

    # Check bbox values (should match 2D)
    assert det.bbox == [503.437255859375, 249.89385986328125, 655.950439453125, 469.82879638671875]

    # 3D-specific assertions
    assert isinstance(det.pointcloud, PointCloud2Msg)
    assert len(det.pointcloud) == 81
    assert det.pointcloud.frame_id == "world"
    assert isinstance(det.transform, Transform)

    # Check center
    center = det.center
    assert isinstance(center, Vector3)
    # Values from output: Vector([    -3.3565    -0.26265     0.18549])
    assert abs(center.x - (-3.3565)) < 1e-4
    assert abs(center.y - (-0.26265)) < 1e-4
    assert abs(center.z - 0.18549) < 1e-4

    # Check pose
    pose = det.pose
    assert isinstance(pose, PoseStamped)
    assert pose.frame_id == "world"
    assert pose.ts == det.ts

    # Check repr dict values
    repr_dict = det.to_repr_dict()
    assert repr_dict["dist"] == "0.88m"
    assert repr_dict["points"] == "81"


@pytest.mark.tool
def test_module3d_replay(dimos_cluster):
    connection = deploy_connection(dimos_cluster, loop=False, speed=1.0)
    # mapper = deploy_navigation(dimos_cluster, connection)
    mapper = dimos_cluster.deploy(
        Map, voxel_size=0.5, cost_resolution=0.05, global_publish_interval=1.0
    )
    mapper.lidar.connect(connection.lidar)
    mapper.global_map.transport = LCMTransport("/global_map", LidarMessage)
    mapper.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    mapper.local_costmap.transport = LCMTransport("/local_costmap", OccupancyGrid)

    mapper.start()

    module3D = dimos_cluster.deploy(Detection3DModule, camera_info=ConnectionModule._camera_info())

    module3D.image.connect(connection.video)
    module3D.pointcloud.connect(connection.lidar)

    module3D.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    module3D.detections.transport = LCMTransport("/detections", Detection2DArray)

    module3D.detected_pointcloud_0.transport = LCMTransport("/detected/pointcloud/0", PointCloud2)
    module3D.detected_pointcloud_1.transport = LCMTransport("/detected/pointcloud/1", PointCloud2)
    module3D.detected_pointcloud_2.transport = LCMTransport("/detected/pointcloud/2", PointCloud2)

    module3D.detected_image_0.transport = LCMTransport("/detected/image/0", Image)
    module3D.detected_image_1.transport = LCMTransport("/detected/image/1", Image)
    module3D.detected_image_2.transport = LCMTransport("/detected/image/2", Image)

    module3D.start()
    connection.start()
    import time

    while True:
        time.sleep(1)
