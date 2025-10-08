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
import json
import time
from abc import ABC, abstractmethod
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
from dimos.models.vl import QwenVlModel, VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.type import (
    Detection2D,
    ImageDetections2D,
    InconvinientDetectionFormat,
)
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.utils.reactive import backpressure


class Detector(ABC):
    @abstractmethod
    def process_image(self, image: np.ndarray) -> InconvinientDetectionFormat: ...


@dataclass
class Config:
    detector: Optional[Callable[[Any], Detector]] = Yolo2DDetector
    max_freq: float = 0.5  # hz
    vlmodel: VlModel = QwenVlModel


class Detection2DModule(Module):
    config: Config
    detector: Detector

    image: In[Image] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config: Config = Config(**kwargs)
        self.detector = self.config.detector()
        self.vlmodel = self.config.vlmodel()
        self.vlm_detections_subject = Subject()

    def vlm_query(self, query: str) -> ImageDetections2D:
        image = self.sharp_image_stream().pipe(ops.take(1)).run()

        full_query = f"""show me a bounding boxes in pixels for this query: `{query}`

        format should be:
        `[
        [label, x1, y1, x2, y2]
        ...
        ]`

        (etc, multiple matches are possible)

        If there's no match return `[]`. Label is whatever you think is appropriate

        Only respond with the coordinates, no other text."""

        response = self.vlmodel.query(image, full_query)
        coords = json.loads(response)

        imageDetections = ImageDetections2D(image)

        for track_id, detection_list in enumerate(coords):
            if len(detection_list) != 5:
                continue
            name = detection_list[0]
            bbox = list(map(float, detection_list[1:]))
            imageDetections.detections.append(
                Detection2D(
                    bbox=bbox,
                    track_id=track_id,
                    class_id=-100,
                    confidence=1.0,
                    name=name,
                    ts=time.time(),
                    image=image,
                )
            )

        print("vlm detected", imageDetections)
        # Emit the VLM detections to the subject
        self.vlm_detections_subject.on_next(imageDetections)

        return imageDetections

    def process_image_frame(self, image: Image) -> ImageDetections2D:
        print("Processing image frame for detections", image)
        return ImageDetections2D.from_detector(
            image, self.detector.process_image(image.to_opencv())
        )

    @functools.cache
    def sharp_image_stream(self) -> Observable[Image]:
        return backpressure(
            self.image.observable().pipe(
                sharpness_barrier(self.config.max_freq),
            )
        )

    @functools.cache
    def detection_stream_2d(self) -> Observable[ImageDetections2D]:
        # self.vlm_detections_subject
        # Regular detection stream from the detector
        regular_detections = self.sharp_image_stream().pipe(ops.map(self.process_image_frame))
        # Merge with VL model detections
        return backpressure(regular_detections.pipe(ops.merge(self.vlm_detections_subject)))

    @rpc
    def start(self):
        # self.detection_stream_2d().subscribe(
        #    lambda det: self.detections.publish(det.to_ros_detection2d_array())
        # )

        def publish_cropped_images(detections: ImageDetections2D):
            for index, detection in enumerate(detections[:3]):
                image_topic = getattr(self, "detected_image_" + str(index))
                image_topic.publish(detection.cropped_image())

        self.detection_stream_2d().subscribe(
            lambda det: self.annotations.publish(det.to_foxglove_annotations())
        )

        def publish_cropped(detections: ImageDetections2D):
            for index, detection in enumerate(detections[:3]):
                image_topic = getattr(self, "detected_image_" + str(index))
                image_topic.publish(detection.cropped_image())

        self.detection_stream_2d().subscribe(publish_cropped)

    @rpc
    def stop(self): ...
