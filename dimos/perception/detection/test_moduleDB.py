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
import time

from lcm_msgs.foxglove_msgs import SceneUpdate
import pytest

from dimos.core import LCMTransport
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.moduleDB import ObjectDBModule
from dimos.robot.unitree.connection import go2


@pytest.mark.module
def test_moduleDB(dimos_cluster) -> None:
    connection = go2.deploy(dimos_cluster, "fake")

    moduleDB = dimos_cluster.deploy(
        ObjectDBModule,
        camera_info=go2._camera_info_static(),
        goto=lambda obj_id: print(f"Going to {obj_id}"),
    )
    moduleDB.image.connect(connection.video)
    moduleDB.pointcloud.connect(connection.lidar)

    moduleDB.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    moduleDB.detections.transport = LCMTransport("/detections", Detection2DArray)

    moduleDB.detected_pointcloud_0.transport = LCMTransport("/detected/pointcloud/0", PointCloud2)
    moduleDB.detected_pointcloud_1.transport = LCMTransport("/detected/pointcloud/1", PointCloud2)
    moduleDB.detected_pointcloud_2.transport = LCMTransport("/detected/pointcloud/2", PointCloud2)

    moduleDB.detected_image_0.transport = LCMTransport("/detected/image/0", Image)
    moduleDB.detected_image_1.transport = LCMTransport("/detected/image/1", Image)
    moduleDB.detected_image_2.transport = LCMTransport("/detected/image/2", Image)

    moduleDB.scene_update.transport = LCMTransport("/scene_update", SceneUpdate)
    moduleDB.target.transport = LCMTransport("/target", PoseStamped)

    connection.start()
    moduleDB.start()

    time.sleep(4)
    print("VLM RES", moduleDB.navigate_to_object_in_view("white floor"))
    time.sleep(30)
