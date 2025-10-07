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
import time
from typing import List, Optional, Tuple

import numpy as np
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection2D as ROSDetection2D
from reactivex import operators as ops

from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.protocol.skill import skill


class DetectionDBModule(Detection3DModule):
    @rpc
    def start(self):
        super().start()
        self.pointcloud_stream().subscribe(self.add_detections)

    def add_detections(self, detections: List[Detection3DArray]):
        for det in detections:
            self.add_detection(det)

    def add_detection(self, detection: Detection3D):
        print("DETECTION", detection)

    def lookup(self, label: str) -> List[Detection3D]:
        """Look up a detection by label."""
        return []
