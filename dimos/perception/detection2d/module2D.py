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
import functools
from dataclasses import dataclass
from typing import Any, Callable, Optional

import numpy as np
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.type import ImageDetections2D
from dimos.perception.detection2d.detectors import Detector, Yolo2DDetector
from dimos.perception.detection2d.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection2d.type import ImageDetections2D
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure


@dataclass
class Config:
    max_freq: float = 5  # hz
    detector: Optional[Callable[[Any], Detector]] = lambda: Yolo2DDetector()


class Detection2DModule(Module):
    config: Config
    detector: Detector

    image: In[Image] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    # just for visualization, emits latest top 3 detections in a frame
    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config: Config = Config(**kwargs)
        self.detector = self.config.detector()
        self.vlm_detections_subject = Subject()

    def process_image_frame(self, image: Image) -> ImageDetections2D:
        # Use person detection specifically if it's a YoloPersonDetector
        if isinstance(self.detector, YoloPersonDetector):
            people = self.detector.detect_people(image)
            return ImageDetections2D.from_pose_detector(image, people)
        else:
            # Fallback to generic dettection for other detectors
            return ImageDetections2D.from_bbox_detector(image, self.detector.process_image(image))

    @simple_mcache
    def sharp_image_stream(self) -> Observable[Image]:
        return backpressure(
            self.image.pure_observable().pipe(
                sharpness_barrier(self.config.max_freq),
            )
        )

    @simple_mcache
    def detection_stream_2d(self) -> Observable[ImageDetections2D]:
        # return self.vlm_detections_subject
        # Regular detection stream from the detector
        regular_detections = self.sharp_image_stream().pipe(ops.map(self.process_image_frame))
        # Merge with VL model detections
        return backpressure(regular_detections.pipe(ops.merge(self.vlm_detections_subject)))

    @rpc
    def start(self):
        super().start()
        unsub = self.detection_stream_2d().subscribe(
            lambda det: self.detections.publish(det.to_ros_detection2d_array())
        )
        self._disposables.add(unsub)

        unsub = self.detection_stream_2d().subscribe(
            lambda det: self.annotations.publish(det.to_foxglove_annotations())
        )
        self._disposables.add(unsub)

        def publish_cropped_images(detections: ImageDetections2D):
            for index, detection in enumerate(detections[:3]):
                image_topic = getattr(self, "detected_image_" + str(index))
                image_topic.publish(detection.cropped_image())

        self.detection_stream_2d().subscribe(publish_cropped_images)

    @rpc
    def stop(self) -> None:
        super().stop()
