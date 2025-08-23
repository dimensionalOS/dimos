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

from typing import Any, Callable, Optional

from dimos_lcm.sensor_msgs import CameraInfo

# Import LCM messages
from dimos_lcm.vision_msgs import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
)
from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Header
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector


class Detect2DModule(Module):
    image: In[Image] = None
    detections: Out[Detection2DArray] = None

    _initDetector = Yolo2DDetector

    def __init__(self, *args, detector=Optional[Callable[[Any], Any]], **kwargs):
        if detector:
            self._detectorClass = detector
        super().__init__(*args, **kwargs)

    def detect(self, image):
        detections = self.detector.process_image(image.to_opencv())
        print(detections)
        return Detection2DArray(detections_length=0, header=Header(), detections=[])

    def publish(self, data):
        print("PUBLSIH", data)

    @rpc
    def start(self):
        self.detector = self._initDetector()
        self.image.observable().pipe(ops.map(self.detect)).subscribe(self.publish)

    @rpc
    def stop(self): ...
