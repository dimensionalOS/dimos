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
from typing import List, Optional, Tuple

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from dimos_lcm.sensor_msgs import CameraInfo
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.type import (
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.perception.detection2d.type.detection3d import Detection3D
from dimos.utils.reactive import backpressure


class Detection3DModule(Detection2DModule):
    camera_info: CameraInfo
    height_filter: Optional[float]

    image: In[Image] = None  # type: ignore
    pointcloud: In[PointCloud2] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    detected_pointcloud_0: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_1: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_2: Out[PointCloud2] = None  # type: ignore
    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    detection_3d_stream: Observable[ImageDetections3D] = None

    def __init__(
        self, camera_info: CameraInfo, height_filter: Optional[float] = -0.05, *args, **kwargs
    ):
        self.height_filter = height_filter
        self.camera_info = camera_info

        Detection2DModule.__init__(self, *args, **kwargs)

    def process_frame(
        self,
        # these have to be timestamp aligned
        detections: ImageDetections2D,
        pointcloud: PointCloud2,
        transform: Transform,
    ) -> ImageDetections3D:
        if not transform:
            return ImageDetections3D(detections.image, [])

        detection3d_list = []
        for detection in detections:
            detection3d = Detection3D.from_2d(
                detection,
                world_pointcloud=pointcloud,
                camera_info=self.camera_info,
                world_to_camera_transform=transform,
                height_filter=self.height_filter,
            )
            if detection3d is not None:
                detection3d_list.append(detection3d)

        return ImageDetections3D(detections.image, detection3d_list)

    @rpc
    def start(self):
        super().start()

        def detection2d_to_3d(args):
            detections, pc = args
            transform = self.tf.get("camera_optical", pc.frame_id, detections.image.ts, 5.0)
            return self.process_frame(detections, pc, transform)

        self.detection_3d_stream = backpressure(
            self.detection_2d_stream().pipe(
                ops.with_latest_from(self.pointcloud.observable()), ops.map(detection2d_to_3d)
            )
        )

        self.detection_3d_stream.subscribe(self._publish_detections)

    def _publish_detections(self, detections: ImageDetections3D):
        if not detections:
            return

        print(detections)

        for index, detection in enumerate(detections[:3]):
            pointcloud_topic = self.get("detected_pointcloud_" + str(index))
            image_topic = self.get("detected_image_" + str(index))
            pointcloud_topic.publish(detection.pointcloud)
            image_topic.publish(detection.cropped_image())
