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
from collections.abc import Callable, Sequence
from typing import Annotated, Any

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import Detection2DArray
from pydantic.experimental.pipeline import validate_as
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.image import image_sharpness
from dimos.msgs.time import to_seconds
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.type.detection2d.base import Filter2D
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure, quality_barrier


class Config(ModuleConfig):
    max_freq: float = 10
    detector: Callable[[Any], Detector] | None = Yolo2DDetector
    publish_detection_images: bool = True
    camera_info: CameraInfo
    filter: Annotated[
        Sequence[Filter2D],
        validate_as(Sequence[Filter2D] | Filter2D).transform(
            lambda f: f if isinstance(f, Sequence) else (f,)
        ),
    ] = ()


class Detection2DModule(Module):
    config: Config
    detector: Detector

    color_image: In[Image]
    tf: IO[TFMessage]

    detections: Out[Detection2DArray]
    detected_image_0: Out[Image]
    detected_image_1: Out[Image]
    detected_image_2: Out[Image]

    cnt: int = 0

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.detector = self.config.detector()  # type: ignore[call-arg, misc]
        self.vlm_detections_subject = Subject()  # type: ignore[var-annotated]
        self.previous_detection_count = 0

    def process_image_frame(self, image: Image) -> ImageDetections2D:
        imageDetections = self.detector.process_image(image)
        if not self.config.filter:
            return imageDetections
        filtered: ImageDetections2D = imageDetections.filter(*self.config.filter)
        return filtered

    @simple_mcache
    def sharp_image_stream(self) -> Observable[Image]:
        return backpressure(
            self.color_image.pure_observable().pipe(
                quality_barrier(image_sharpness, self.config.max_freq),
            )
        )

    @simple_mcache
    def detection_stream_2d(self) -> Observable[ImageDetections2D]:
        return backpressure(self.sharp_image_stream().pipe(ops.map(self.process_image_frame)))

    def track(self, detections: ImageDetections2D) -> None:
        sensor_frame = self.tfbuffer.get(
            "sensor", "camera_optical", to_seconds(detections.image.header.stamp), 5.0
        )

        if not sensor_frame:
            return

        if not detections.detections:
            return

        sensor_frame.child_frame_id = "sensor_frame"
        transforms = [sensor_frame]

        current_count = len(detections.detections)
        max_count = max(current_count, self.previous_detection_count)

        # Publish transforms for all detection slots up to max_count
        for index in range(max_count):
            if index < current_count:
                # Active detection - compute real position
                detection = detections.detections[index]
                position_3d = self.pixel_to_3d(  # type: ignore[attr-defined]
                    detection.center_bbox,
                    assumed_depth=1.0,
                )
            else:
                # No detection at this index - publish zero transform
                position_3d = Vector3()

            transforms.append(
                TransformStamped(
                    header=Header(
                        frame_id=sensor_frame.child_frame_id, stamp=detections.image.header.stamp
                    ),
                    child_frame_id=f"det_{index}",
                    transform=Transform(translation=position_3d, rotation=Quaternion(w=1)),
                )
            )

        self.previous_detection_count = current_count
        self.tfbuffer.publish(*transforms)

    @rpc
    def start(self) -> None:
        # self.detection_stream_2d().subscribe(self.track)

        self.detection_stream_2d().subscribe(
            lambda det: self.detections.publish(det.to_ros_detection2d_array())
        )

        def publish_cropped_images(detections: ImageDetections2D) -> None:
            for index, detection in enumerate(detections[:3]):
                image_topic = getattr(self, "detected_image_" + str(index))
                image_topic.publish(detection.cropped_image())

        if self.config.publish_detection_images:
            self.detection_stream_2d().subscribe(publish_cropped_images)

    @rpc
    def stop(self) -> None:
        return super().stop()
