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
import threading
import time
from copy import copy
from typing import Any, Callable, Dict, List, Optional

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from lcm_msgs.foxglove_msgs import SceneUpdate
from reactivex.observable import Observable

from dimos.agents2 import Agent, Output, Reducer, Stream, skill
from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import Detection3D, ImageDetections3D, TableStr
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Output, Reducer, Stream
from dimos.types.timestamped import to_datetime


# Represents an object in space, as collection of 3d detections over time
class Object3D(Detection3D):
    best_detection: Detection3D = None
    center: Vector3 = None
    track_id: str = None
    detections: List[Detection3D]

    def to_repr_dict(self) -> Dict[str, Any]:
        return {
            "object_id": self.track_id,
            "detections": len(self.detections),
            "center": "[" + ", ".join(list(map(lambda n: f"{n:1f}", self.center.to_list()))) + "]",
        }

    def __init__(self, track_id: str, detection: Optional[Detection3D] = None, *args, **kwargs):
        if detection is None:
            self.detections = []
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
        self.detections = [detection]
        self.best_detection = detection

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
        new_object.frame_id = self.frame_id
        new_object.center = (self.center + detection.center) / 2
        new_object.detections = self.detections + [detection]

        if detection.bbox_2d_volume() > self.bbox_2d_volume():
            new_object.best_detection = detection
        else:
            new_object.best_detection = self.best_detection

        return new_object

    @property
    def image(self) -> Image:
        return self.best_detection.image

    def scene_entity_label(self) -> str:
        return f"{self.name} ({len(self.detections)})"

    def agent_encode(self):
        return {
            "id": self.track_id,
            "name": self.name,
            "detections": len(self.detections),
            "last_seen": f"{round((time.time() - self.ts))}s ago",
            # "position": self.to_pose().position.agent_encode(),
        }

    def to_pose(self) -> PoseStamped:
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
    object_stream: Observable[Object3D] = None

    goto: Callable[[PoseStamped], Any] = None

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

    target: Out[PoseStamped] = None  # type: ignore

    remembered_locations: Dict[str, PoseStamped]

    def __init__(self, goto: Callable[[PoseStamped], Any], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.goto = goto
        self.objects = {}
        self.remembered_locations = {}

    def closest_object(self, detection: Detection3D) -> Optional[Object3D]:
        # Filter objects to only those with matching names
        matching_objects = [obj for obj in self.objects.values() if obj.name == detection.name]

        if not matching_objects:
            return None

        # Sort by distance
        distances = sorted(matching_objects, key=lambda obj: detection.center.distance(obj.center))

        return distances[0]

    def add_detections(self, detections: List[Detection3D]) -> List[Object3D]:
        return [
            detection for detection in map(self.add_detection, detections) if detection is not None
        ]

    def add_detection(self, detection: Detection3D):
        """Add detection to existing object or create new one."""
        closest = self.closest_object(detection)
        if closest and closest.bounding_box_intersects(detection):
            return self.add_to_object(closest, detection)
        else:
            return self.create_new_object(detection)

    def add_to_object(self, closest: Object3D, detection: Detection3D):
        new_object = closest + detection
        self.objects[closest.track_id] = new_object
        return new_object

    def create_new_object(self, detection: Detection3D):
        new_object = Object3D(f"obj_{self.cnt}", detection)
        self.objects[new_object.track_id] = new_object
        self.cnt += 1
        return new_object

    def agent_encode(self) -> List[Any]:
        ret = []
        for obj in copy(self.objects).values():
            # we need at least 3 detectieons to consider it a valid object
            # for this to be serious we need a ratio of detections within the window of observations
            # if len(obj.detections) < 3:
            #    continue
            ret.append(str(obj.agent_encode()))
        if not ret:
            return "No objects detected yet."
        return "\n".join(ret)

    def vlm_query(self, description: str) -> str:
        imageDetections2D = super().vlm_query(description)
        print("VLM query found", imageDetections2D, "detections")
        time.sleep(3)

        if not imageDetections2D.detections:
            return None

        ret = []
        for obj in self.objects.values():
            if obj.ts != imageDetections2D.ts:
                print(
                    "Skipping",
                    obj.track_id,
                    "ts",
                    obj.ts,
                    "!=",
                    imageDetections2D.ts,
                )
                continue
            if obj.class_id != -100:
                continue
            if obj.name != imageDetections2D.detections[0].name:
                print("Skipping", obj.name, "!=", imageDetections2D.detections[0].name)
                continue
            ret.append(obj)
        ret.sort(key=lambda x: x.ts)

        return ret[0] if ret else None

    @skill()
    def remember_location(self, name: str) -> str:
        """Remember the current location with a name."""
        transform = self.tf.get("map", "sensor", time_point=time.time(), time_tolerance=1.0)
        if not transform:
            return f"Could not get current location transform from map to sensor"

        pose = transform.to_pose()
        pose.frame_id = "map"
        self.remembered_locations[name] = pose
        return f"Location '{name}' saved at position: {pose.position}"

    @skill()
    def goto_remembered_location(self, name: str) -> str:
        """Go to a remembered location by name."""
        pose = self.remembered_locations.get(name, None)
        if not pose:
            return f"Location {name} not found. Known locations: {list(self.remembered_locations.keys())}"
        self.goto(pose)
        return f"Navigating to remembered location {name} and pose {pose}"

    @skill()
    def list_remembered_locations(self) -> List[str]:
        """List all remembered locations."""
        return str(list(self.remembered_locations.keys()))

    def nav_to(self, target_pose) -> str:
        target_pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.target.publish(target_pose)
        time.sleep(0.1)
        self.target.publish(target_pose)
        self.goto(target_pose)

    @skill()
    def navigate_to_object_in_view(self, query: str) -> str:
        """Navigate to an object in your current image view via natural language query using vision-language model to find it."""
        target_obj = self.vlm_query(query)
        if not target_obj:
            return f"No objects found matching '{query}'"
        return self.navigate_to_object_by_id(target_obj.track_id)

    @skill(reducer=Reducer.all)
    def list_objects(self):
        """List all detected objects that the system remembers and can navigate to."""
        data = self.agent_encode()
        return data

    @skill()
    def navigate_to_object_by_id(self, object_id: str):
        """Navigate to an object by an object id"""
        target_obj = self.objects.get(object_id, None)
        if not target_obj:
            return f"Object {object_id} not found\nHere are the known objects:\n{str(self.agent_encode())}"
        target_pose = target_obj.to_pose()
        target_pose.frame_id = "map"
        self.target.publish(target_pose)
        time.sleep(0.1)
        self.target.publish(target_pose)
        self.nav_to(target_pose)
        return f"Navigating to f{object_id} f{target_obj.name}"

    def lookup(self, label: str) -> List[Detection3D]:
        """Look up a detection by label."""
        return []

    @rpc
    def start(self):
        super().start()

        def update_objects(imageDetections: ImageDetections3D):
            for detection in imageDetections.detections:
                # print(detection)
                return self.add_detection(detection)

        def scene_thread():
            while True:
                scene_update = self.to_foxglove_scene_update()
                self.scene_update.publish(scene_update)
                time.sleep(1.0)

        threading.Thread(target=scene_thread, daemon=True).start()

        self.detection_stream_3d.subscribe(update_objects)

    def goto_object(self, object_id: str) -> Optional[Object3D]:
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

        for obj in copy(self.objects).values():
            # we need at least 3 detectieons to consider it a valid object
            # for this to be serious we need a ratio of detections within the window of observations
            if obj.class_id != -100 and len(obj.detections) < 3:
                continue

            # print(
            #    f"Object {obj.track_id}: {len(obj.detections)} detections, confidence {obj.confidence}"
            # )
            # print(obj.to_pose())

            scene_update.entities.append(
                obj.to_foxglove_scene_entity(
                    entity_id=f"object_{obj.name}_{obj.track_id}_{len(obj.detections)}"
                )
            )

        scene_update.entities_length = len(scene_update.entities)
        return scene_update

    def __len__(self):
        return len(self.objects.values())

    def __iter__(self):
        return iter(self.detections.values())
