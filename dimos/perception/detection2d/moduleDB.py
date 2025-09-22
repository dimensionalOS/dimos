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
from typing import Any, Dict, List, Optional

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from lcm_msgs.foxglove_msgs import SceneUpdate
from reactivex.observable import Observable

from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import (
    Detection3D,
    ImageDetections3D,
)
from dimos.perception.detection2d.type.detection3d import Detection3D


# Represents an object in space, as collection of 3d detections over time
class Object3D(Detection3D):
    image: Image = None
    center: Vector3 = None
    track_id: str = None

    def to_repr_dict(self) -> Dict[str, Any]:
        return {"object_id": self.track_id}

    def __init__(self, track_id: str, detection: Optional[Detection3D] = None, *args, **kwargs):
        if detection is None:
            return
        self.ts = detection.ts
        self.track_id = track_id
        self.class_id = detection.class_id
        self.name = detection.name
        self.confidence = detection.confidence
        self.pointcloud = detection.pointcloud
        self.image = detection.image
        self.bbox = detection.bbox
        self.transform = detection.transform

    def __add__(self, detection: Detection3D) -> "Object3D":
        new_object = Object3D(self.track_id)
        new_object.bbox = detection.bbox
        new_object.confidence = max(self.confidence, detection.confidence)
        new_object.ts = max(self.ts, detection.ts)
        new_object.track_id = self.track_id
        new_object.class_id = self.class_id
        new_object.name = self.name
        new_object.transform = self.transform
        new_object.pointcloud = self.pointcloud + detection.pointcloud

        if detection.image.sharpness > self.image.sharpness:
            new_object.image = detection.image
        else:
            new_object.image = self.image

        new_object.center = (self.center + detection.center) / 2

        return new_object


class ObjectDBModule(Detection3DModule):
    cnt: int = 0
    objects: dict[str, Object3D]
    object_stream: Observable[Object3D] = None

    image: In[Image] = None  # type: ignore
    pointcloud: In[PointCloud2] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    detected_pointcloud_0: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_1: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_2: Out[PointCloud2] = None  # type: ignore

    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    scene_update: Out[SceneUpdate] = None  # type: ignore

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.objects = {}

    def closest_object(self, detection: Detection3D) -> Optional[Object3D]:
        return None
        distances = sorted(
            self.objects.values(), key=lambda obj: detection.center.distance(obj.center)
        )

        if not distances:
            return None

        print(f"Distances to existing objects: {distances}")
        return distances[0]

    def add_detection(self, detection: Detection3D):
        """Add detection to existing object or create new one."""
        closest = self.closest_object(detection)
        if closest and closest.bounding_box_intersects(detection):
            new_obj = self.add_to_object(closest, detection)
        else:
            new_obj = self.create_new_object(detection)

        print(f"Adding/Updating object: {new_obj}")
        self.scene_update.publish(new_obj.to_foxglove_scene_entity())

    def add_to_object(self, closest: Object3D, detection: Detection3D):
        new_object = closest + detection
        self[closest.track_id] = new_object
        return new_object

    def create_new_object(self, detection: Detection3D):
        new_object = Object3D(f"obj_{self.cnt}", detection)
        self.objects[new_object.track_id] = new_object
        self.cnt += 1
        return new_object

    def lookup(self, label: str) -> List[Detection3D]:
        """Look up a detection by label."""
        return []

    @rpc
    def start(self):
        super().start()

        def add_image_detections(imageDetections: ImageDetections3D):
            print(self.objects)
            for detection in imageDetections.detections:
                try:
                    self.add_detection(detection)
                except Exception as e:
                    print(f"✗ Error adding detection to object: {e}")
                    import traceback

                    traceback.print_exc()

        self.detection_stream_3d.subscribe(add_image_detections)
