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
from dimos.perception.detection2d.conftest import Moment, dimos_cluster, publish_lcm
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map


def test_module2d(moment: Moment):
    detections2d = Detection2DModule().process_image_frame(moment.get("image_frame"))
    print(detections2d)

    annotations = detections2d.to_image_annotations()
    publish_lcm({annotations: "annotations", **moment})


def test_module3d(moment: Moment):
    detections2d = Detection2DModule().process_image_frame(moment.get("image_frame"))
    pointcloud = moment.get("lidar_frame")
    camera_transform = moment.get("tf").get("camera_optical", "world")
    annotations = detections2d.to_image_annotations()

    detections3d = Detection3DModule(camera_info=moment.get("camera_info")).process_frame(
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
