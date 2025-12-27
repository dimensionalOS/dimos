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
from copy import copy
import threading
import time
from typing import Any

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    ImageAnnotations,
)
from lcm_msgs.foxglove_msgs import SceneUpdate  # type: ignore[import-not-found]
from reactivex.observable import Observable

from dimos import spec
from dimos.core import DimosCluster, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.type import ImageDetections3DPC, TableStr
from dimos.perception.detection.type.detection3d import Detection3DPC


# Represents an object in space, as collection of 3d detections over time
class Object3D(Detection3DPC):
    best_detection: Detection3DPC | None = None
    center: Vector3 | None = None  # type: ignore
    track_id: str | None = None  # type: ignore
    detections: int = 0

    def to_repr_dict(self) -> dict[str, Any]:
        if self.center is None:
            center_str = "None"
        else:
            center_str = (
                "[" + ", ".join(list(map(lambda n: f"{n:1f}", self.center.to_list()))) + "]"
            )
        return {
            "object_id": self.track_id,
            "detections": self.detections,
            "center": center_str,
        }

    def __init__(  # type: ignore[no-untyped-def]
        self, track_id: str, detection: Detection3DPC | None = None, *args, **kwargs
    ) -> None:
        if detection is None:
            return
        self.ts = detection.ts
        self.track_id = track_id
        self.class_id = detection.class_id
        self.name = detection.name
        self.confidence = detection.confidence
        self.pointcloud = detection.pointcloud
        self.bbox = detection.bbox
        self.transform = detection.transform
        self.center = detection.center
        self.frame_id = detection.frame_id
        self.detections = self.detections + 1
        self.best_detection = detection

    def __add__(self, detection: Detection3DPC) -> "Object3D":
        if self.track_id is None:
            raise ValueError("Cannot add detection to object with None track_id")
        new_object = Object3D(self.track_id)
        new_object.bbox = detection.bbox
        new_object.confidence = max(self.confidence, detection.confidence)
        new_object.ts = max(self.ts, detection.ts)
        new_object.track_id = self.track_id
        new_object.class_id = self.class_id
        new_object.name = self.name
        new_object.transform = self.transform
        new_object.pointcloud = self.pointcloud + detection.pointcloud
        new_object.frame_id = self.frame_id
        new_object.center = (self.center + detection.center) / 2
        new_object.detections = self.detections + 1

        if detection.bbox_2d_volume() > self.bbox_2d_volume():
            new_object.best_detection = detection
        else:
            new_object.best_detection = self.best_detection

        return new_object

    def get_image(self) -> Image | None:
        return self.best_detection.image if self.best_detection else None

    def scene_entity_label(self) -> str:
        return f"{self.name} ({self.detections})"

    def agent_encode(self):  # type: ignore[no-untyped-def]
        return {
            "id": self.track_id,
            "name": self.name,
            "detections": self.detections,
            "last_seen": f"{round(time.time() - self.ts)}s ago",
            # "position": self.to_pose().position.agent_encode(),
        }

    def to_pose(self) -> PoseStamped:
        if self.best_detection is None or self.center is None:
            raise ValueError("Cannot compute pose without best_detection and center")

        optical_inverse = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
        ).inverse()

        print("transform is", self.best_detection.transform)

        global_transform = optical_inverse + self.best_detection.transform

        print("inverse optical is", global_transform)

        print("obj center is", self.center)
        global_pose = global_transform.to_pose()
        print("Global pose:", global_pose)
        global_pose.frame_id = self.best_detection.frame_id
        print("remap to", self.best_detection.frame_id)
        return PoseStamped(
            position=self.center, orientation=Quaternion(), frame_id=self.best_detection.frame_id
        )


class ObjectDBModule(Detection3DModule, TableStr):
    cnt: int = 0
    objects: dict[str, Object3D]
    object_stream: Observable[Object3D] | None = None

    goto: Callable[[PoseStamped], Any] | None = None

    image: In[Image]
    pointcloud: In[PointCloud2]

    detections: Out[Detection2DArray]
    annotations: Out[ImageAnnotations]

    detected_pointcloud_0: Out[PointCloud2]
    detected_pointcloud_1: Out[PointCloud2]
    detected_pointcloud_2: Out[PointCloud2]

    detected_image_0: Out[Image]
    detected_image_1: Out[Image]
    detected_image_2: Out[Image]

    scene_update: Out[SceneUpdate]

    target: Out[PoseStamped]

    remembered_locations: dict[str, PoseStamped]

    @rpc
    def start(self) -> None:
        Detection3DModule.start(self)

        def update_objects(imageDetections: ImageDetections3DPC) -> None:
            for detection in imageDetections.detections:
                self.add_detection(detection)

        def scene_thread() -> None:
            while True:
                scene_update = self.to_foxglove_scene_update()
                self.scene_update.publish(scene_update)
                time.sleep(1.0)

        threading.Thread(target=scene_thread, daemon=True).start()

        self.detection_stream_3d.subscribe(update_objects)

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.goto = None
        self.objects = {}
        self.remembered_locations = {}

    def closest_object(self, detection: Detection3DPC) -> Object3D | None:
        # Filter objects to only those with matching names
        matching_objects = [obj for obj in self.objects.values() if obj.name == detection.name]

        if not matching_objects:
            return None

        # Sort by distance
        distances = sorted(matching_objects, key=lambda obj: detection.center.distance(obj.center))

        return distances[0]

    def add_detections(self, detections: list[Detection3DPC]) -> list[Object3D]:
        return [
            detection for detection in map(self.add_detection, detections) if detection is not None
        ]

    def add_detection(self, detection: Detection3DPC):  # type: ignore[no-untyped-def]
        """Add detection to existing object or create new one."""
        closest = self.closest_object(detection)
        if closest and closest.bounding_box_intersects(detection):
            return self.add_to_object(closest, detection)
        else:
            return self.create_new_object(detection)

    def add_to_object(self, closest: Object3D, detection: Detection3DPC):  # type: ignore[no-untyped-def]
        new_object = closest + detection
        if closest.track_id is not None:
            self.objects[closest.track_id] = new_object
        return new_object

    def create_new_object(self, detection: Detection3DPC):  # type: ignore[no-untyped-def]
        new_object = Object3D(f"obj_{self.cnt}", detection)
        if new_object.track_id is not None:
            self.objects[new_object.track_id] = new_object
        self.cnt += 1
        return new_object

    def agent_encode(self) -> str:
        ret = []
        for obj in copy(self.objects).values():
            # we need at least 3 detectieons to consider it a valid object
            # for this to be serious we need a ratio of detections within the window of observations
            if len(obj.detections) < 4:  # type: ignore[arg-type]
                continue
            ret.append(str(obj.agent_encode()))  # type: ignore[no-untyped-call]
        if not ret:
            return "No objects detected yet."
        return "\n".join(ret)

    # @rpc
    # def vlm_query(self, description: str) -> Object3D | None:  # type: ignore[override]
    #     imageDetections2D = super().ask_vlm(description)
    #     print("VLM query found", imageDetections2D, "detections")
    #     time.sleep(3)

    #     if not imageDetections2D.detections:
    #         return None

    #     ret = []
    #     for obj in self.objects.values():
    #         if obj.ts != imageDetections2D.ts:
    #             print(
    #                 "Skipping",
    #                 obj.track_id,
    #                 "ts",
    #                 obj.ts,
    #                 "!=",
    #                 imageDetections2D.ts,
    #             )
    #             continue
    #         if obj.class_id != -100:
    #             continue
    #         if obj.name != imageDetections2D.detections[0].name:
    #             print("Skipping", obj.name, "!=", imageDetections2D.detections[0].name)
    #             continue
    #         ret.append(obj)
    #     ret.sort(key=lambda x: x.ts)

    #     return ret[0] if ret else None

    def lookup(self, label: str) -> list[Detection3DPC]:
        """Look up a detection by label."""
        return []

    @rpc
    def stop(self):  # type: ignore[no-untyped-def]
        return super().stop()

    def goto_object(self, object_id: str) -> Object3D | None:
        """Go to object by id."""
        return self.objects.get(object_id, None)

    def to_foxglove_scene_update(self) -> "SceneUpdate":
        """Convert all detections to a Foxglove SceneUpdate message.

        Returns:
            SceneUpdate containing SceneEntity objects for all detections
        """

        # Create SceneUpdate message with all detections
        scene_update = SceneUpdate()
        scene_update.deletions_length = 0
        scene_update.deletions = []
        scene_update.entities = []

        for obj in self.objects:
            try:
                scene_update.entities.append(
                    obj.to_foxglove_scene_entity(entity_id=f"{obj.name}_{obj.track_id}")  # type: ignore[attr-defined]
                )
            except Exception:
                pass

        scene_update.entities_length = len(scene_update.entities)
        return scene_update

    def __len__(self) -> int:
        return len(self.objects.values())


def deploy(  # type: ignore[no-untyped-def]
    dimos: DimosCluster,
    lidar: spec.Pointcloud,
    camera: spec.Camera,
    prefix: str = "/detectorDB",
    **kwargs,
) -> Detection3DModule:
    from dimos.core import LCMTransport

    detector = dimos.deploy(ObjectDBModule, camera_info=camera.camera_info_stream, **kwargs)  # type: ignore[attr-defined]

    detector.image.connect(camera.color_image)
    detector.pointcloud.connect(lidar.pointcloud)

    detector.annotations.transport = LCMTransport(f"{prefix}/annotations", ImageAnnotations)
    detector.detections.transport = LCMTransport(f"{prefix}/detections", Detection2DArray)
    detector.scene_update.transport = LCMTransport(f"{prefix}/scene_update", SceneUpdate)

    detector.detected_image_0.transport = LCMTransport(f"{prefix}/image/0", Image)
    detector.detected_image_1.transport = LCMTransport(f"{prefix}/image/1", Image)
    detector.detected_image_2.transport = LCMTransport(f"{prefix}/image/2", Image)

    detector.detected_pointcloud_0.transport = LCMTransport(f"{prefix}/pointcloud/0", PointCloud2)
    detector.detected_pointcloud_1.transport = LCMTransport(f"{prefix}/pointcloud/1", PointCloud2)
    detector.detected_pointcloud_2.transport = LCMTransport(f"{prefix}/pointcloud/2", PointCloud2)

    detector.start()
    return detector  # type: ignore[no-any-return]


detectionDB_module = ObjectDBModule.blueprint

__all__ = ["ObjectDBModule", "deploy", "detectionDB_module"]
