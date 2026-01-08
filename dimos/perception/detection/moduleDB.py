# Copyright 2025-2026 Dimensional Inc.
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

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from lcm_msgs.foxglove_msgs import SceneUpdate  # type: ignore[import-not-found]
from reactivex.observable import Observable

from dimos.core import In, Out, rpc
from dimos.models.vl.qwen import QwenVlModel  # ← Correct path
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.type import ImageDetections3DPC, TableStr
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Represents an object in space, as collection of 3d detections over time
class Object3D(Detection3DPC):
    best_detection: Detection3DPC | None = None
    center: Vector3 | None = None  # type: ignore[assignment]
    track_id: str | None = None  # type: ignore[assignment]
    detections: int = 0
    yolo_label: str | None = None  # Original YOLO detection class
    vlm_label: str | None = None  # VLM-enriched description

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

        new_object.yolo_label = self.yolo_label
        new_object.vlm_label = self.vlm_label

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

    _vlm_model: QwenVlModel | None = None
    enable_vlm_enrichment: bool = True

    color_image: In[Image]
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

    def vlm_model(self):
        """Lazy load VLM model."""
        if self._vlm_model is None and self.enable_vlm_enrichment:
            logger.info("Initializing VLM model for label enrichment")
            self._vlm_model = QwenVlModel()
        return self._vlm_model

    def _enrich_with_vlm(self, obj: Object3D) -> str:
        """Generate rich description using VLM.

        Args:
            obj: Object3D with detection

        Returns:
            Rich description string
        """
        if not self.enable_vlm_enrichment or self.vlm_model is None:
            return obj.yolo_label  # Fall back to YOLO label

        try:
            # Get image from best detection
            image = obj.get_image()
            if image is None:
                logger.warning(f"No image for {obj.track_id}, using YOLO label")
                return obj.yolo_label

            # Generate rich description with VLM
            prompt = f"Describe this {obj.yolo_label} in detail. Include color, appearance, and distinguishing features. Keep it concise (under 10 words)."

            description = self.vlm_model.query(image, prompt)

            # Clean up the description
            rich_label = description.strip()

            logger.info(f"VLM enrichment: '{obj.yolo_label}' → '{rich_label}'")
            return rich_label

        except Exception as e:
            logger.error(f"VLM enrichment failed for {obj.track_id}: {e}")
            return obj.yolo_label  # Fall back to YOLO label

    def closest_object(self, detection: Detection3DPC) -> Object3D | None:
        matching_objects = [
            obj
            for obj in self.objects.values()
            if obj.yolo_label == detection.name  # ← Use yolo_label for tracking
        ]

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

    def add_to_object(self, closest: Object3D, detection: Detection3DPC):
        """Add detection to existing object."""
        new_object = closest + detection

        # Re-enrich every 5 detections
        if self.enable_vlm_enrichment and new_object.detections % 5 == 0:
            logger.info(
                f"Re-enriching {new_object.track_id} after {new_object.detections} detections"
            )
            rich_label = self._enrich_with_vlm(new_object)
            new_object.vlm_label = rich_label
            new_object.name = rich_label
        else:
            new_object.name = new_object.vlm_label or new_object.yolo_label

        if closest.track_id is not None:
            self.objects[closest.track_id] = new_object

        return new_object

    def create_new_object(self, detection: Detection3DPC):
        """Create new object and enrich with VLM."""
        new_object = Object3D(f"obj_{self.cnt}", detection)

        # Enrich label with VLM
        if self.enable_vlm_enrichment and new_object.track_id is not None:
            rich_label = self._enrich_with_vlm(new_object)
            new_object.vlm_label = rich_label
            new_object.name = rich_label
        else:
            new_object.name = new_object.yolo_label

        if new_object.track_id is not None:
            self.objects[new_object.track_id] = new_object

        self.cnt += 1
        logger.info(
            f"Created object {new_object.track_id}: YOLO='{new_object.yolo_label}' VLM='{new_object.vlm_label}'"
        )
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
    # def vlm_query(self, description: str) -> Object3D | None:
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

    @rpc
    def lookup(self, label: str, min_detections: int = 0.5) -> list[dict]:
        """Look up objects by label/name.

        Returns lightweight dict instead of full Object3D to avoid RPC timeout.

        Args:
            label: Name/class to search for
            min_detections: Minimum number of detections required (default: 1)

        Returns:
            List of dicts with object info (track_id, name, position, etc.)
        """
        import time

        # DEBUG: Log search details
        logger.error(f"Lookup called for '{label}'")
        logger.error(f"Total objects in DB: {len(self.objects)}")

        # DEBUG: Log ALL object names in database
        all_names = [obj.name for obj in self.objects.values() if obj.name]
        logger.error(f"All object names in DB: {all_names}")

        matching = []

        for obj in self.objects.values():
            # DEBUG: Log each comparison
            if obj.name:
                logger.debug(
                    f"  Checking: '{obj.name}' vs '{label}' (detections: {obj.detections})"
                )

            # Check name match
            if obj.name and label.lower() in obj.name.lower():
                # Check detection threshold
                if obj.detections >= min_detections:
                    # Create lightweight dict (NO pointcloud, NO heavy data)
                    try:
                        pose = obj.to_pose()
                        result = {
                            "track_id": obj.track_id,
                            "name": obj.name,
                            "detections": obj.detections,
                            "confidence": obj.confidence,
                            "pos_x": pose.position.x,
                            "pos_y": pose.position.y,
                            "pos_z": pose.position.z,
                            "frame_id": pose.frame_id,
                            "last_seen": time.time() - obj.ts,
                        }
                        matching.append(result)
                        logger.error(f"Added to results: {result}")
                    except Exception as e:
                        logger.error(f"Failed to get pose for {obj.track_id}: {e}")
                        continue

        logger.error(f"Returning {len(matching)} matches")
        return matching

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


detectionDB_module = ObjectDBModule.blueprint

__all__ = ["ObjectDBModule", "detectionDB_module"]
