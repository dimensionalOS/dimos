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
from typing import Any, Callable, List, Optional

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.type import Detection2D, ImageDetections2D
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.utils.reactive import backpressure


class Detection2DModule(Module):
    image: In[Image] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    _initDetector = Yolo2DDetector

    def __init__(self, *args, detector=Optional[Callable[[Any], Any]], **kwargs):
        super().__init__(*args, **kwargs)
        if detector:
            self._detectorClass = detector
        self.detector = self._initDetector()

    def process_image_frame(self, image: Image) -> ImageDetections2D:
        detections = ImageDetections2D.from_detector(
            image, self.detector.process_image(image.to_opencv())
        )
        return detections

    @functools.cache
    def detection_stream(self) -> Observable[ImageDetections2D]:
        return backpressure(self.image.observable().pipe(ops.map(self.process_image_frame)))

    @rpc
    def start(self):
        self.detection_stream().subscribe(
            lambda det: self.detections.publish(det.to_ros_detection2d_array())
        )

        self.detection_stream().subscribe(
            lambda det: self.annotations.publish(det.to_foxglove_annotations())
        )

    @rpc
    def stop(self): ...
