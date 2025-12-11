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
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    ImageAnnotations,
)
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos import spec
from dimos.core import DimosCluster, In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.detectors import Detector  # type: ignore[attr-defined]
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.type import Filter2D, ImageDetections2D
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure


@dataclass
class Config(ModuleConfig):
    max_freq: float = 10
    detector: Callable[[Any], Detector] | None = Yolo2DDetector
    publish_detection_images: bool = True
    camera_info: CameraInfo = None  # type: ignore
    filter: list[Filter2D] | Filter2D | None = None

    def __post_init__(self) -> None:
        if self.filter is None:
            self.filter = []
        elif not isinstance(self.filter, list):
            self.filter = [self.filter]


class Detection2DModule(Module):
    default_config = Config
    config: Config
    detector: Detector

    image: In[Image]

    detections: Out[Detection2DArray]
    annotations: Out[ImageAnnotations]

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
        return imageDetections.filter(*self.config.filter)  # type: ignore[misc, return-value]

    @simple_mcache
    def sharp_image_stream(self) -> Observable[Image]:
        return backpressure(
            self.image.pure_observable().pipe(
                sharpness_barrier(self.config.max_freq),
            )
        )

    @simple_mcache
    def detection_stream_2d(self) -> Observable[ImageDetections2D]:
        return backpressure(self.sharp_image_stream().pipe(ops.map(self.process_image_frame)))

    def track(self, detections: ImageDetections2D) -> None:
        sensor_frame = self.tf.get("sensor", "camera_optical", detections.image.ts, 5.0)

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
                    detection.center_bbox,  # type: ignore[attr-defined]
                    self.config.camera_info,
                    assumed_depth=1.0,
                )
            else:
                # No detection at this index - publish zero transform
                position_3d = Vector3(0.0, 0.0, 0.0)

            transforms.append(
                Transform(
                    frame_id=sensor_frame.child_frame_id,
                    child_frame_id=f"det_{index}",
                    ts=detections.image.ts,
                    translation=position_3d,
                )
            )

        self.previous_detection_count = current_count
        self.tf.publish(*transforms)

    @rpc
    def start(self) -> None:
        # self.detection_stream_2d().subscribe(self.track)

        self.detection_stream_2d().subscribe(
            lambda det: self.detections.publish(det.to_ros_detection2d_array())
        )

        self.detection_stream_2d().subscribe(
            lambda det: self.annotations.publish(det.to_foxglove_annotations())
        )

        def publish_cropped_images(detections: ImageDetections2D) -> None:
            for index, detection in enumerate(detections[:3]):
                image_topic = getattr(self, "detected_image_" + str(index))
                image_topic.publish(detection.cropped_image())

        if self.config.publish_detection_images:
            self.detection_stream_2d().subscribe(publish_cropped_images)

    @rpc
    def stop(self) -> None:
        return super().stop()  # type: ignore[no-any-return]


def deploy(  # type: ignore[no-untyped-def]
    dimos: DimosCluster,
    camera: spec.Camera,
    prefix: str = "/detector2d",
    **kwargs,
) -> Detection2DModule:
    from dimos.core import LCMTransport

    detector = Detection2DModule(**kwargs)
    detector.image.connect(camera.color_image)

    detector.annotations.transport = LCMTransport(f"{prefix}/annotations", ImageAnnotations)
    detector.detections.transport = LCMTransport(f"{prefix}/detections", Detection2DArray)

    detector.detected_image_0.transport = LCMTransport(f"{prefix}/image/0", Image)
    detector.detected_image_1.transport = LCMTransport(f"{prefix}/image/1", Image)
    detector.detected_image_2.transport = LCMTransport(f"{prefix}/image/2", Image)

    detector.start()
    return detector
