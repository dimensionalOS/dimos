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
from typing import List, Optional

from dimos.core import rpc
from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import (
    Detection3D,
)
from dimos.types.timestamped import TimestampedCollection


# Represents an object in space, as collection of 3d detections over time
class Object(Detection3D, TimestampedCollection[Detection3D]):
    image: Image = None
    center: Vector3 = None

    def __init__(self, *detections: List[Detection3D]):
        TimestampedCollection.__init__(self)
        for det in detections:
            self.add(det)

    def add(self, detection: Detection3D):
        super().add(detection)

        # initial detection
        if not self.image:
            self.track_id = detection.track_id
            self.class_id = detection.class_id
            self.confidence = detection.confidence  # TODO need to update properly
            self.name = detection.name
            self.image = detection.image
            self.center = detection.center
            return

        if self.image:
            self.center = (detection.center + self.center) / 2.0
            if detection.image.sharpness > self.image.sharpness:
                self.image = detection.image


class ObjectDBModule(Detection3DModule):
    objects: List[Object] = []
    distance_threshold: float = 1.0  # meters

    def find_closest_object(self, detection: Detection3D) -> Optional[Object]:
        if not self.objects:
            return None

        closest_obj = None
        min_distance = float("inf")

        for obj in self.objects:
            distance = detection.center.distance(obj.center)
            if distance < min_distance and distance < self.distance_threshold:
                min_distance = distance
                closest_obj = obj

        return closest_obj

    def add_detection(self, detection: Detection3D):
        """Add detection to existing object or create new one."""
        closest = self.find_closest_object(detection)

        if closest:
            closest.add(detection)
        else:
            new_object = Object(detection)
            self.objects.append(new_object)

    def lookup(self, label: str) -> List[Detection3D]:
        """Look up a detection by label."""
        return []
