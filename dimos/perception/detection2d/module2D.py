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

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.type import (
    Detection2D,
)
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector


class Detection2DModule(Module):
    image: In[Image] = None  # type: ignore
    detections: Out[Detection2D] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    # _initDetector = Detic2DDetector
    _initDetector = Yolo2DDetector

    def __init__(self, *args, detector=Optional[Callable[[Any], Any]], **kwargs):
        super().__init__(*args, **kwargs)
        if detector:
            self._detectorClass = detector
        self.detector = self._initDetector()

    def process_frame(self, image: Image) -> List[Detection2D]:
        detections = Detection2D.from_detector(
            self.detector.process_image(image.to_opencv()), image=image
        )
        return detections

    @functools.cache
    def detection_stream(self):
        # Returns stream of individual Detection2D objects
        detection_stream = self.image.observable().pipe(
            ops.map(self.process_frame),
            ops.flat_map(
                lambda detections: ops.from_iterable(detections)
            ),  # Flatten list to individual items
        )

        # Publish each detection individually
        detection_stream.subscribe(self.detections.publish)

        # Convert each Detection2D to ImageAnnotations
        detection_stream.pipe(ops.map(lambda detection: detection.to_imageannotations())).subscribe(
            self.annotations.publish
        )

        return detection_stream

    @rpc
    def start(self):
        self.detection_stream()

    @rpc
    def stop(self): ...
