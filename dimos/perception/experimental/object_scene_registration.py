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
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import AliasChoices, Field, model_validator

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.experimental.object import (
    Object,
    Object as DetObject,
    aggregate_pointclouds,
    to_detection3d_array,
)
from dimos.perception.experimental.objectDB import ObjectDB
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class ObjectSceneRegistrationConfig(ModuleConfig):
    """Configurable detector, segmenter, and RGB-D object reconstruction settings."""

    target_frame: str = "map"
    prompt_mode: YoloePromptMode = YoloePromptMode.LRPC
    distance_threshold: float = 0.2
    min_detections_for_permanent: int = 6
    register_objects: bool = True
    detect_on_request: bool = False
    detector_confidence: float = 0.6
    det: Literal["yoloe", "moondream"] = Field(
        default="yoloe", validation_alias=AliasChoices("det", "detector_backend")
    )
    seg: Literal["yolo", "edgetam"] = Field(
        default="yolo", validation_alias=AliasChoices("seg", "segmentation_backend")
    )
    object_voxel_downsample: float = 0.005
    max_distance: float = 0.0
    use_aabb: bool = False
    max_obstacle_width: float = 0.0

    @model_validator(mode="after")
    def _require_edgetam_for_moondream(self) -> "ObjectSceneRegistrationConfig":
        if self.det == "moondream" and self.seg != "edgetam":
            raise ValueError("osr.det=moondream requires osr.seg=edgetam")
        return self


class ObjectSceneRegistrationModule(Module):
    """Module for prompted 2D detection, segmentation, and RGB-D object reconstruction."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    detections_2d: Out[Detection2DArray]
    detections_3d: Out[Detection3DArray]
    annotated_image: Out[Image]
    objects: Out[list[DetObject]]
    pointcloud: Out[PointCloud2]

    _detector: Detector | None = None
    _segmenter: Any | None = None
    _camera_info: CameraInfo | None = None
    _object_db: ObjectDB
    _latest_objects: list[Object]
    _latest_output_objects: tuple[Object, ...]
    _latest_aligned_frames: tuple[Image, Image] | None = None
    # A tuple assignment/read is atomic, so depth and its transform cannot be
    # observed from different frames by get_full_scene_pointcloud().
    _latest_scene_snapshot: tuple[Image, Transform | None] | None = None
    config: ObjectSceneRegistrationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._target_frame = self.config.target_frame
        self._prompt_mode = self.config.prompt_mode
        self._register_objects = self.config.register_objects
        self._detect_on_request = self.config.detect_on_request
        self._detector_confidence = self.config.detector_confidence
        self._detector_backend = self.config.det
        self._segmentation_backend = self.config.seg
        self._object_db = ObjectDB(
            distance_threshold=self.config.distance_threshold,
            min_detections_for_permanent=self.config.min_detections_for_permanent,
        )
        self._latest_objects = []
        self._latest_output_objects = ()
        self._object_voxel_downsample = self.config.object_voxel_downsample
        self._max_distance = self.config.max_distance
        self._use_aabb = self.config.use_aabb
        self._max_obstacle_width = self.config.max_obstacle_width

    @rpc
    def start(self) -> None:
        super().start()

        if self._detector_backend == "moondream":
            from dimos.perception.detection.detectors.moondream import Moondream2DDetector

            self._detector = Moondream2DDetector()
        else:
            if self._prompt_mode == YoloePromptMode.LRPC:
                model_name = "yoloe-11l-seg-pf.pt"
            else:
                model_name = "yoloe-11l-seg.pt"
            self._detector = Yoloe2DDetector(
                model_name=model_name,
                prompt_mode=self._prompt_mode,
                conf=self._detector_confidence,
            )
        if self._segmentation_backend == "edgetam":
            from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter

            self._segmenter = EdgeTAMImageSegmenter()

        self.camera_info.subscribe(lambda msg: setattr(self, "_camera_info", msg))

        aligned_frames = align_timestamped(
            self.color_image.observable(),
            self.depth_image.observable(),
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
        self._segmenter = None

        self._object_db.clear()
        self._latest_objects = []
        self._latest_output_objects = ()
        self._latest_aligned_frames = None

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
            set_prompts = getattr(self._detector, "set_prompts", None)
            if not callable(set_prompts):
                raise RuntimeError("configured detector does not support prompts")
            set_prompts(text=text, bboxes=bboxes)

    @rpc
    def select_object(self, track_id: int) -> dict[str, Any] | None:
        """Get object data by track_id and promote to permanent."""
        for obj in self._known_objects():
            if obj.track_id == track_id:
                if self._register_objects:
                    self._object_db.promote(obj.object_id)
                return obj.to_dict()
        return None

    @rpc
    def get_object_track_ids(self) -> list[int]:
        """Get track_ids of all permanent objects."""
        return [obj.track_id for obj in self._known_objects()]

    @rpc
    def get_detected_objects(self) -> list[dict[str, Any]]:
        """Get all detected objects with object_id (UUID) and name."""
        return [obj.agent_encode() for obj in self._known_objects()]

    @rpc
    def scan_scene(self) -> Detection3DArray:
        """Run detection on the latest aligned RGB-D frame and return its 3D detections."""
        frames = self._latest_aligned_frames
        if frames is None:
            return to_detection3d_array([], frame_id=self._target_frame)

        if not self._register_objects:
            self._latest_objects = []
            self._latest_output_objects = ()
        self._process_images(*frames)
        return to_detection3d_array(
            list(self._latest_output_objects),
            frame_id=self._target_frame,
            ts=frames[0].ts,
        )

    @rpc
    def describe_scene(self, question: str) -> str:
        """Answer an open-ended scene question using the configured Moondream detector."""
        frames = self._latest_aligned_frames
        if frames is None:
            raise RuntimeError("No aligned RGB-D frame is available")
        describe_image = getattr(self._detector, "describe_image", None)
        if not callable(describe_image):
            raise RuntimeError("Scene description requires osr.det=moondream")
        return str(describe_image(frames[0], question))

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        """Get pointcloud for an object by class name."""
        objects = [obj for obj in self._known_objects() if obj.name == name]
        return objects[0].pointcloud if objects else None

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """Get pointcloud for an object by its stable object_id (searches all objects)."""
        obj = next((obj for obj in self._known_objects() if obj.object_id == object_id), None)
        if obj is None:
            logger.warning(f"No object found with object_id='{object_id}'")
            return None
        pc = obj.pointcloud
        num_points = len(pc.pointcloud.points) if pc else 0
        logger.info(f"Found object '{object_id}' ({obj.name}) with {num_points} points")
        return pc

    def _get_object_mask(self, object_id: str) -> NDArray[np.uint8] | None:
        """Get dilated mask for an object by ID."""
        import cv2

        for obj in self._known_objects():
            if obj.object_id != object_id:
                continue
            if obj.mask is None:
                return None

            mask = obj.mask.astype(np.uint8)
            if mask.max() == 1:
                mask = (mask * 255).astype(np.uint8)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            return cv2.dilate(mask, kernel).astype(np.uint8)

        return None

    def _known_objects(self) -> list[Object]:
        if self._register_objects:
            return self._object_db.get_all_objects()
        return self._latest_objects

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """Get full scene pointcloud from depth, including table/surfaces for collision filtering."""
        import open3d as o3d  # type: ignore[import-untyped]

        scene_snapshot = self._latest_scene_snapshot
        if scene_snapshot is None or self._camera_info is None:
            return None

        depth_image, camera_transform = scene_snapshot
        depth_cv = depth_image.to_opencv()
        h, w = depth_cv.shape[:2]

        # Zero out excluded object's depth
        if exclude_object_id:
            exclude_mask = self._get_object_mask(exclude_object_id)
            if exclude_mask is not None:
                depth_cv = depth_cv.copy()
                depth_cv[exclude_mask > 0] = 0

        # Build pointcloud from depth
        fx, fy = self._camera_info.K[0], self._camera_info.K[4]
        cx, cy = self._camera_info.K[2], self._camera_info.K[5]
        intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

        depth_o3d = o3d.geometry.Image(depth_cv.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d, intrinsic, depth_scale=1.0, depth_trunc=depth_trunc
        )

        if len(pcd.points) < 100:
            return None

        pcd = pcd.voxel_down_sample(voxel_size)

        pc = PointCloud2(
            pcd,
            frame_id=depth_image.frame_id,
            ts=depth_image.ts,
        )

        if camera_transform is not None:
            pc = pc.transform(camera_transform)

        return pc

    @skill
    def detect(self, *prompts: str) -> str:
        """Detect objects matching the given text prompts.

        Do NOT call this tool multiple times for one query. Pass all objects in a single call.
        For example, to detect a cup and mouse, call detect("cup", "mouse") not detect("cup") then detect("mouse").

        Args:
            prompts (str): Text descriptions of objects to detect (e.g., "person", "car", "dog")

        Returns:
            str: Detected objects with their object_id (stable UUID) and name.

        Example:
            detect("person", "car", "dog")
            detect("cup")
        """
        if not prompts:
            return "No prompts provided."
        if self._detector is None:
            return "Detector not initialized."

        self._detector.set_prompts(text=list(prompts))
        time.sleep(2.0)

        detected = self.get_detected_objects()
        if not detected:
            return "No objects detected."

        obj_list = [f"  - {obj['name']} (object_id='{obj['object_id']}')" for obj in detected]
        return f"Detected {len(detected)} object(s):\n" + "\n".join(obj_list)

    @skill
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
        if self._detect_on_request:
            self._latest_aligned_frames = (color_msg, depth_msg)
            return
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

        # Log each expensive stage separately so a stalled on-demand scan can be localized.
        t0 = time.monotonic()
        logger.info("Object detection started", detector=self._detector_backend)
        detections_2d: ImageDetections2D[Any] = self._detector.process_image(color_image)
        logger.info(
            "Object detection completed",
            detector=self._detector_backend,
            duration_s=round(time.monotonic() - t0, 3),
            detections=len(detections_2d.detections),
        )
        if self._segmenter is not None:
            t0 = time.monotonic()
            logger.info("Object segmentation started", segmenter=self._segmentation_backend)
            detections_2d = self._segmenter.segment(detections_2d)
            logger.info(
                "Object segmentation completed",
                segmenter=self._segmentation_backend,
                duration_s=round(time.monotonic() - t0, 3),
                detections=len(detections_2d.detections),
            )

        detections_2d_msg = Detection2DArray(
            detections_length=len(detections_2d.detections),
            header=Header(color_image.ts, color_image.frame_id or ""),
            detections=[det.to_ros_detection2d() for det in detections_2d.detections],
        )
        self.detections_2d.publish(detections_2d_msg)
        self.annotated_image.publish(detections_2d.annotated_image())

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
            camera_transform = self.tfbuffer.get(
                self._target_frame,
                color_image.frame_id,
                color_image.ts,
                # Request-driven scans can use a cached camera frame while
                # inference starts; retain temporal alignment within that cache.
                3.0,
                forward_tolerance=0.2,
            )
            if camera_transform is None:
                logger.info("Failed to lookup transform from camera frame to target frame")
                return

        # Cache depth and transform together, only after the lookup succeeds.
        self._latest_scene_snapshot = (depth_image, camera_transform)

        objects = Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
            voxel_downsample=self._object_voxel_downsample,
            max_distance=self._max_distance,
            use_aabb=self._use_aabb,
            max_obstacle_width=self._max_obstacle_width,
        )
        if self._register_objects:
            if not objects:
                return
            self._object_db.add_objects(objects)
            # Registered mode publishes the complete confirmed scene, not just this frame.
            output_objects = self._object_db.get_objects()
        else:
            self._latest_objects = objects
            output_objects = objects

        self._latest_output_objects = tuple(output_objects)

        detections_3d = to_detection3d_array(
            output_objects,
            frame_id=self._target_frame,
            ts=color_image.ts,
        )
        self.detections_3d.publish(detections_3d)
        self.objects.publish(output_objects)

        aggregated_pc = aggregate_pointclouds(output_objects)
        if not output_objects:
            aggregated_pc.frame_id = self._target_frame
            aggregated_pc.ts = color_image.ts
        self.pointcloud.publish(aggregated_pc)
