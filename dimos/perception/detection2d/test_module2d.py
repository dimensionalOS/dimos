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

from dimos.perception.detection2d.conftest import Moment, publish_lcm
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule


def test_module2d(moment: Moment):
    detections2d = Detection2DModule().process_frame(moment.get("image_frame"))
    annotations = detections2d.to_image_annotations()
    publish_lcm({annotations: "annotations", **moment})


def test_module3d(moment: Moment):
    detections2d = Detection2DModule().process_frame(moment.get("image_frame"))
    pointcloud = moment.get("lidar_frame")
    camera_transform = moment.get("tf").get("camera_optical", "world")

    detections3d = Detection3DModule(camera_info=moment.get("camera_info")).process_frame(
        detections2d, pointcloud, camera_transform
    )

    print(detections3d)
