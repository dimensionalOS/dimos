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


from typing import Optional

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from lcm_msgs.foxglove_msgs import SceneUpdate
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.agents2 import skill
from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module2D import Config as Module2DConfig
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.detection.type import (
    ImageDetections2D,
    ImageDetections3DPC,
)
from dimos.perception.detection.type.detection3d import Detection3DPC
from dimos.types.timestamped import align_timestamped
from dimos.utils.reactive import backpressure


class Config(Module2DConfig): ...


class Detection3DModule(Detection2DModule):
    image: In[Image] = None  # type: ignore
    pointcloud: In[PointCloud2] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore
    scene_update: Out[SceneUpdate] = None  # type: ignore

    # just for visualization,
    # emits latest pointclouds of detected objects in a frame
    detected_pointcloud_0: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_1: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_2: Out[PointCloud2] = None  # type: ignore

    # just for visualization, emits latest top 3 detections in a frame
    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    detection_3d_stream: Optional[Observable[ImageDetections3DPC]] = None

    def process_frame(
        self,
        detections: ImageDetections2D,
        pointcloud: PointCloud2,
        transform: Transform,
    ) -> ImageDetections3DPC:
        if not transform:
            return ImageDetections3DPC(detections.image, [])

        detection3d_list: list[Detection3DPC] = []
        for detection in detections:
            detection3d = Detection3DPC.from_2d(
                detection,
                world_pointcloud=pointcloud,
                camera_info=self.config.camera_info,
                world_to_optical_transform=transform,
            )
            if detection3d is not None:
                detection3d_list.append(detection3d)

        return ImageDetections3DPC(detections.image, detection3d_list)

    @skill  # type: ignore[arg-type]
    def ask_vlm(self, question: str) -> str | ImageDetections3DPC:
        """
        query visual model about the view in front of the camera
        you can ask to mark objects like:

        "red cup on the table left of the pencil"
        "laptop on the desk"
        "a person wearing a red shirt"
        """
        from dimos.models.vl.qwen import QwenVlModel

        model = QwenVlModel()
        result = model.query(self.image.get_next(), question)

        if isinstance(result, str) or not result or not len(result):
            return "No detections"

        detections: ImageDetections2D = result
        pc = self.pointcloud.get_next()
        transform = self.tf.get("camera_optical", pc.frame_id, detections.image.ts, 5.0)
        return self.process_frame(detections, pc, transform)

    @rpc
    def start(self):
        super().start()

        def detection2d_to_3d(args):
            detections, pc = args
            transform = self.tf.get("camera_optical", pc.frame_id, detections.image.ts, 5.0)
            return self.process_frame(detections, pc, transform)

        self.detection_stream_3d = align_timestamped(
            backpressure(self.detection_stream_2d()),
            self.pointcloud.observable(),
            match_tolerance=0.25,
            buffer_size=20.0,
        ).pipe(ops.map(detection2d_to_3d))

        self.detection_stream_3d.subscribe(self._publish_detections)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _publish_detections(self, detections: ImageDetections3DPC):
        if not detections:
            return

        for index, detection in enumerate(detections[:3]):
            pointcloud_topic = getattr(self, "detected_pointcloud_" + str(index))
            pointcloud_topic.publish(detection.pointcloud)
