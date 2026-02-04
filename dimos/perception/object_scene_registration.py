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

import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.core import In, Out, rpc
from dimos.core.skill_module import SkillModule
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.objectDB import ObjectDB
from dimos.perception.detection.type import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import (
    Object,
    aggregate_pointclouds,
    to_detection3d_array,
)
from dimos.protocol.skill.skill import skill
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class ObjectSceneRegistrationModule(SkillModule):
    """Module for detecting objects in camera images using YOLO-E with 2D and 3D detection."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]

    detections_2d: Out[Detection2DArray]
    detections_3d: Out[Detection3DArray]
    overlay: Out[ImageAnnotations]
    pointcloud: Out[PointCloud2]

    _detector: Yoloe2DDetector | None = None
    _camera_info: CameraInfo | None = None
    _object_db: ObjectDB

    def __init__(
        self,
        target_frame: str = "map",
        prompt_mode: YoloePromptMode = YoloePromptMode.LRPC,
    ) -> None:
        super().__init__()
        self._target_frame = target_frame
        self._prompt_mode = prompt_mode
        self._object_db = ObjectDB()

    @rpc
    def start(self) -> None:
        super().start()

        if self._prompt_mode == YoloePromptMode.LRPC:
            model_name = "yoloe-11l-seg-pf.pt"
        else:
            model_name = "yoloe-11l-seg.pt"

        self._detector = Yoloe2DDetector(
            model_name=model_name,
            prompt_mode=self._prompt_mode,
        )

        self.camera_info.subscribe(lambda msg: setattr(self, "_camera_info", msg))

        aligned_frames = align_timestamped(
            self.color_image.observable(),  # type: ignore[no-untyped-call]
            self.depth_image.observable(),  # type: ignore[no-untyped-call]
            buffer_size=2.0,
            match_tolerance=0.1,
        )
        backpressure(aligned_frames).subscribe(self._on_aligned_frames)

    @rpc
    def stop(self) -> None:
        """Stop the module and clean up resources."""

        if self._detector:
            self._detector.stop()
            self._detector = None

        self._object_db.clear()

        logger.info("ObjectSceneRegistrationModule stopped")
        super().stop()

    @rpc
    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None:
        """Set prompts for detection. Provide either text or bboxes, not both."""
        if self._detector is not None:
            self._detector.set_prompts(text=text, bboxes=bboxes)

    @rpc
    def select_object(self, track_id: int) -> dict[str, Any] | None:
        """Get object data by track_id and promote to permanent."""
        for obj in self._object_db.get_all_objects():
            if obj.track_id == track_id:
                self._object_db.promote(obj.object_id)
                return obj.to_dict()
        return None

    @rpc
    def get_object_track_ids(self) -> list[int]:
        """Get track_ids of all permanent objects."""
        return [obj.track_id for obj in self._object_db.get_all_objects()]

    @skill()
    def detect(self, *prompts: str) -> str:
        """Detect objects matching the given text prompts. Returns track_ids after 2 seconds of detection.

        Do NOT call this tool multiple times for one query. Pass all objects in a single call.
        For example, to detect a cup and mouse, call detect("cup", "mouse") not detect("cup") then detect("mouse").

        Args:
            prompts (str): Text descriptions of objects to detect (e.g., "person", "car", "dog")

        Returns:
            str: A message containing the track_ids of detected objects

        Example:
            detect("person", "car", "dog")
            detect("person")
        """
        if not prompts:
            return "No prompts provided."
        if self._detector is None:
            return "Detector not initialized."

        self._detector.set_prompts(text=list(prompts))
        time.sleep(2.0)

        track_ids = self.get_object_track_ids()
        if not track_ids:
            return "No objects detected."
        return f"Detected objects with track_ids: {track_ids}"

    @skill()
    def select(self, track_id: int) -> str:
        """Select an object by track_id and promote it to permanent.

        Example:
            select(5)
        """
        result = self.select_object(track_id)
        if result is None:
            return f"No object found with track_id {track_id}."
        return f"Selected object {track_id}: {result['name']}"

    def _on_aligned_frames(self, frames) -> None:  # type: ignore[no-untyped-def]
        color_msg, depth_msg = frames
        self._process_images(color_msg, depth_msg)

    def _process_images(self, color_msg: Image, depth_msg: Image) -> None:
        """Process synchronized color and depth images (runs in background thread)."""
        if not self._detector or not self._camera_info:
            return

        color_image = color_msg
        # Convert depth to meters (float32)
        depth_cv = depth_msg.to_opencv()
        if depth_msg.format == ImageFormat.DEPTH16:
            depth_cv = depth_cv.astype(np.float32) / 1000.0
        elif depth_cv.dtype != np.float32:
            depth_cv = depth_cv.astype(np.float32)
        depth_image = Image(
            data=depth_cv, format=ImageFormat.DEPTH, frame_id=depth_msg.frame_id, ts=depth_msg.ts
        )

        # Run 2D detection
        detections_2d: ImageDetections2D[Any] = self._detector.process_image(color_image)

        detections_2d_msg = Detection2DArray(
            detections_length=len(detections_2d.detections),
            header=Header(color_image.ts, color_image.frame_id or ""),
            detections=[det.to_ros_detection2d() for det in detections_2d.detections],
        )
        self.detections_2d.publish(detections_2d_msg)

        overlay_annotations = detections_2d.to_foxglove_annotations()
        self.overlay.publish(overlay_annotations)

        # Process 3D detections
        self._process_3d_detections(detections_2d, color_image, depth_image)

    def _process_3d_detections(
        self,
        detections_2d: ImageDetections2D[Any],
        color_image: Image,
        depth_image: Image,
    ) -> None:
        """Convert 2D detections to 3D and publish."""
        if self._camera_info is None:
            return

        # Look up transform from camera frame to target frame (e.g., map)
        camera_transform = None
        if self._target_frame != color_image.frame_id:
            camera_transform = self.tf.get(
                self._target_frame,
                color_image.frame_id,
                color_image.ts,
                0.1,
            )
            if camera_transform is None:
                logger.warning("Failed to lookup transform from camera frame to target frame")
                return

        objects = Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
        )
        if not objects:
            return

        # Add objects to spatial memory database
        objects = self._object_db.add_objects(objects)

        detections_3d = to_detection3d_array(objects)
        self.detections_3d.publish(detections_3d)

        objects_for_pc = self._object_db.get_objects()
        aggregated_pc = aggregate_pointclouds(objects_for_pc)
        self.pointcloud.publish(aggregated_pc)
        return


object_scene_registration_module = ObjectSceneRegistrationModule.blueprint

__all__ = ["ObjectSceneRegistrationModule", "object_scene_registration_module"]
