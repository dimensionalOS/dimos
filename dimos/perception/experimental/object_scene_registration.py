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

from __future__ import annotations

from contextlib import suppress
from copy import deepcopy
import threading
import time
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.segmentation.edge_tam import BoxPromptImageSegmenter
from dimos.models.segmentation.yoloe import YoloeBoxSegmenter
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.experimental.object import (
    Object,
    Object as DetObject,
    aggregate_pointclouds,
    to_detection3d_array,
)
from dimos.perception.experimental.objectDB import ObjectDB
from dimos.perception.experimental.rgbd_quality import filter_depth_detections, validate_rgbd
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class ObjectSceneRegistrationConfig(ModuleConfig):
    target_frame: str = "base_link"
    prompt_mode: YoloePromptMode = YoloePromptMode.LRPC
    detector_backend: Literal["yoloe", "owlv2", "moondream"] = "yoloe"
    segmentation_backend: Literal["yolo", "edgetam"] = "yolo"
    detector_confidence: float = 0.6
    segmentation_confidence: float = 0.05
    detect_on_request: bool = False
    distance_threshold: float = 0.2
    min_detections_for_permanent: int = 6
    max_distance: float = 0.0
    use_aabb: bool = False
    max_obstacle_width: float = 0.0
    max_input_age_s: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    max_result_age_s: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    max_pair_skew_s: float = Field(default=0.1, ge=0, allow_inf_nan=False)
    min_valid_depth_fraction: float = Field(default=0.0, ge=0, le=1, allow_inf_nan=False)
    min_valid_depth_pixels: int = Field(default=0, ge=0)
    require_segmentation: bool = False
    snapshot_only: bool = False


class ObjectSceneRegistrationModule(Module):
    """Register prompted camera detections as stable 3D scene objects."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    detections_2d: Out[Detection2DArray]
    detections_3d: Out[Detection3DArray]
    objects: Out[list[DetObject]]
    pointcloud: Out[PointCloud2]

    _detector: Any | None = None
    _segmenter: BoxPromptImageSegmenter | None = None
    _camera_info: CameraInfo | None = None
    _object_db: ObjectDB
    _text_prompts: list[str]
    _latest_aligned_frames: tuple[Image, Image] | None = None
    _processing_lock: threading.RLock
    # A tuple assignment/read keeps capture geometry and intrinsics together.
    _latest_scene_snapshot: tuple[Image, Transform | None, CameraInfo] | None = None

    config: ObjectSceneRegistrationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._target_frame = self.config.target_frame
        self._prompt_mode = self.config.prompt_mode
        self._detector_backend = self.config.detector_backend
        self._segmentation_backend = self.config.segmentation_backend
        self._detector_confidence = self.config.detector_confidence
        self._segmentation_confidence = self.config.segmentation_confidence
        self._detect_on_request = self.config.detect_on_request
        self._object_db = ObjectDB(
            distance_threshold=self.config.distance_threshold,
            min_detections_for_permanent=self.config.min_detections_for_permanent,
        )
        self._processing_lock = threading.RLock()
        self._scan_images: tuple[Image, Image] | None = None
        self._scan_objects: list[DetObject] = []
        self._scan_quality: list[dict[str, Any]] = []
        self._scan_error: str | None = None
        self._scan_duration_s: float | None = None
        self._scan_capture_ts: float | None = None
        self._text_prompts = []
        self._max_distance = self.config.max_distance
        self._use_aabb = self.config.use_aabb
        self._max_obstacle_width = self.config.max_obstacle_width

        # Initial deployment constructs modules serially within each worker.
        # Resolve only the selected detector before parallel start RPCs; importing
        # this module stays lazy, and model construction/loading stays in start().
        self._detector_class: type[Any] | None = None
        try:
            if self._detector_backend == "owlv2":
                from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

                self._detector_class = Owlv2Detector
            elif self._detector_backend == "moondream":
                from dimos.models.vl.moondream import MoondreamVlModel

                self._detector_class = MoondreamVlModel
        except BaseException:
            # The worker cannot track an instance whose constructor failed.
            # Release its resources without masking the original import error.
            with suppress(Exception):
                super().stop()
            raise

    @rpc
    def start(self) -> None:
        super().start()

        if self._detector_class is not None:
            self._detector = self._detector_class()
            self._detector.start()
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

        self._segmenter = self._create_segmenter()

        self.camera_info.subscribe(self._on_camera_info)

        aligned_frames = align_timestamped(
            self.color_image.observable(),
            self.depth_image.observable(),
            buffer_size=2.0,
            match_tolerance=0.1,
        )
        backpressure(aligned_frames).subscribe(self._on_aligned_frames)

    def _create_segmenter(self) -> BoxPromptImageSegmenter | None:
        if self._segmentation_backend == "yolo":
            if self._detector_backend == "yoloe":
                return None
            return YoloeBoxSegmenter(confidence=self._segmentation_confidence)

        if self._segmentation_backend == "edgetam":
            try:
                from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter

                return EdgeTAMImageSegmenter()
            except ModuleNotFoundError as e:
                raise ModuleNotFoundError(
                    "EdgeTAM requires the optional dependencies from dimos[perception]"
                ) from e
        return None

    def _on_camera_info(self, info: CameraInfo) -> None:
        with self._processing_lock:
            self._camera_info = info

    @rpc
    def stop(self) -> None:
        """Stop the module and clean up resources."""

        with self._processing_lock:
            if self._segmenter is not None:
                self._segmenter.stop()
            self._segmenter = None
            if self._detector:
                self._detector.stop()
                self._detector = None

            self._object_db.clear()
            self._latest_aligned_frames = None
            self._clear_scan()

        logger.info("ObjectSceneRegistrationModule stopped")
        super().stop()

    @rpc
    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None:
        """Set prompts for detection. Provide either text or bboxes, not both."""
        with self._processing_lock:
            self._set_prompts(text=text, bboxes=bboxes)

    def _set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None:
        if self._detector_backend in {"owlv2", "moondream"}:
            if bboxes is not None:
                raise ValueError(f"{self._detector_backend} supports text prompts only")
            self._text_prompts = text or []
        elif self._detector is not None:
            self._detector.set_prompts(text=text, bboxes=bboxes)

    @rpc
    def scan_scene(self, text: list[str] | None = None) -> Detection3DArray:
        """Run one serialized detection pass over the latest aligned RGB-D frame."""
        with self._processing_lock:
            if text is not None:
                self._set_prompts(text=text)
            frames = self._latest_aligned_frames
            if frames is None:
                if self.config.max_input_age_s is not None:
                    self._clear_scan()
                    self._object_db.clear()
                    self._scan_error = "No RGB-D frames available"
                    raise ValueError(self._scan_error)
                return to_detection3d_array([], frame_id=self._target_frame)
            strict_snapshot = (
                self.config.snapshot_only
                or self.config.max_input_age_s is not None
                or self.config.max_result_age_s is not None
            )
            self._clear_scan(clear_scene=strict_snapshot)
            if self.config.snapshot_only:
                self._object_db.clear()
            started = time.monotonic()
            try:
                objects = self._scan_scene_objects(frames)
                self._scan_capture_ts = frames[0].ts
                if not self._scene_is_fresh():
                    raise ValueError("Scan completed after its result freshness deadline")
            except Exception as error:
                self._clear_scan(clear_scene=strict_snapshot)
                if strict_snapshot:
                    self._object_db.clear()
                self._scan_error = str(error)
                raise
            finally:
                self._scan_duration_s = time.monotonic() - started
            return to_detection3d_array(
                objects,
                frame_id=self._target_frame,
                ts=frames[0].ts,
            )

    def _clear_scan(self, *, clear_scene: bool = True) -> None:
        self._scan_images = None
        self._scan_objects = []
        self._scan_quality = []
        self._scan_capture_ts = None
        if clear_scene:
            self._latest_scene_snapshot = None
        self._scan_error = None

    def _scene_is_fresh(self) -> bool:
        limit = self.config.max_result_age_s
        if limit is None:
            return True
        stamp = self._scan_capture_ts
        if stamp is None or not 0 <= time.time() - stamp <= limit:
            self._object_db.clear()
            self._scan_images = None
            self._scan_objects = []
            self._latest_scene_snapshot = None
            return False
        return True

    @rpc
    def get_scan_status(self) -> dict[str, Any]:
        """Read quality and freshness; an expired observation is never actionable geometry."""
        with self._processing_lock:
            fresh = self._scene_is_fresh() and self._scan_images is not None
            return {
                "available": fresh,
                "capture_ts": self._scan_capture_ts,
                "age_s": None
                if self._scan_capture_ts is None
                else time.time() - self._scan_capture_ts,
                "inference_s": self._scan_duration_s,
                "frame_id": self._target_frame,
                "object_count": len(self._scan_objects) if fresh else 0,
                "quality": self._scan_quality,
                "error": self._scan_error,
                "reason": "" if fresh else "no fresh scan",
            }

    @rpc
    def get_scan_visualization(self, max_points: int = 10000) -> dict[str, Any]:
        """Return a bounded, coherent read-only snapshot for perception viewers."""
        if not 1 <= max_points <= 50000:
            raise ValueError("max_points must be between 1 and 50000")
        with self._processing_lock:
            status = self.get_scan_status()
            if not status["available"] or self._scan_images is None:
                return {"status": status}
            overlay, depth = self._scan_images
            cloud = self.get_full_scene_pointcloud(depth_trunc=3.0)
            points = (
                np.empty((0, 3), dtype=np.float32)
                if cloud is None
                else np.asarray(cloud.pointcloud.points, dtype=np.float32)
            )
            points = points[:: max(1, int(np.ceil(len(points) / max_points)))]
            objects = []
            object_budget = max(1, max_points // max(1, len(self._scan_objects)))
            for obj in self._scan_objects:
                xyz = np.asarray(obj.pointcloud.pointcloud.points, dtype=np.float32)
                objects.append(
                    {
                        "object_id": obj.object_id,
                        "name": obj.name,
                        "center": [obj.center.x, obj.center.y, obj.center.z],
                        "size": [obj.size.x, obj.size.y, obj.size.z],
                        "points": xyz[:: max(1, int(np.ceil(len(xyz) / object_budget)))],
                        "point_count": len(xyz),
                    }
                )
            return {
                "status": status,
                "overlay": overlay,
                "depth": depth,
                "points": points,
                "objects": objects,
            }

    def _scan_scene_objects(self, frames: tuple[Image, Image] | None = None) -> list[DetObject]:
        """Process one frame and return only objects observed during this scan."""
        frames = frames or self._latest_aligned_frames
        if frames is None:
            return []
        return self._process_images(*frames)

    @rpc
    def select_object(self, track_id: int) -> dict[str, Any] | None:
        """Get object data by track_id and promote to permanent."""
        with self._processing_lock:
            if not self._scene_is_fresh():
                return None
            for obj in self._object_db.get_all_objects():
                if obj.track_id == track_id:
                    self._object_db.promote(obj.object_id)
                    return obj.to_dict()
            return None

    @rpc
    def get_object_track_ids(self) -> list[int]:
        """Get track_ids of all permanent objects."""
        with self._processing_lock:
            if not self._scene_is_fresh():
                return []
            return [obj.track_id for obj in self._object_db.get_all_objects()]

    @rpc
    def get_detected_objects(self) -> list[dict[str, Any]]:
        """Get all detected objects with object_id (UUID) and name."""
        with self._processing_lock:
            if not self._scene_is_fresh():
                return []
            return [obj.agent_encode() for obj in self._object_db.get_all_objects()]

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        """Get pointcloud for an object by class name."""
        with self._processing_lock:
            if not self._scene_is_fresh():
                return None
            objects = self._object_db.find_by_name(name)
            return objects[0].pointcloud if objects else None

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """Get pointcloud for an object by its stable object_id (searches all objects)."""
        with self._processing_lock:
            if not self._scene_is_fresh():
                return None
            obj = self._object_db.find_by_object_id(object_id)
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

        for obj in self._object_db.get_all_objects():
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

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """Get full scene pointcloud from depth, including table/surfaces for collision filtering."""
        with self._processing_lock:
            if not self._scene_is_fresh():
                return None
            import open3d as o3d  # type: ignore[import-untyped]

            scene_snapshot = self._latest_scene_snapshot
            if scene_snapshot is None or self._camera_info is None:
                return None

            depth_image, camera_transform, camera_info = scene_snapshot
            depth_cv = depth_image.to_opencv()
            h, w = depth_cv.shape[:2]

            # Zero out excluded object's depth
            if exclude_object_id:
                exclude_mask = self._get_object_mask(exclude_object_id)
                if exclude_mask is not None:
                    depth_cv = depth_cv.copy()
                    depth_cv[exclude_mask > 0] = 0

            # Build pointcloud from depth
            fx, fy = camera_info.K[0], camera_info.K[4]
            cx, cy = camera_info.K[2], camera_info.K[5]
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
    def detect(self, prompts: list[str]) -> str:
        """Detect objects matching the given text prompts.

        Do NOT call this tool multiple times for one query. Pass all objects in a single call.
        For example, to detect a cup and mouse, pass ["cup", "mouse"] in one call.

        Args:
            prompts: Text descriptions of concrete object categories to detect.

        Returns:
            str: Detected objects with their object_id (stable UUID) and name.

        Example:
            detect(["person", "car", "dog"])
            detect(["cup"])
        """
        if not prompts:
            return "No prompts provided."
        with self._processing_lock:
            if self._detector is None:
                return "Detector not initialized."
            self._set_prompts(text=prompts)
            if self._detect_on_request:
                detected = [obj.agent_encode() for obj in self._scan_scene_objects()]
            else:
                detected = None
        if detected is None:
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
        self._latest_aligned_frames = (color_msg, depth_msg)
        if self._detect_on_request:
            return
        with self._processing_lock:
            self._process_images(color_msg, depth_msg)

    def _process_images(self, color_msg: Image, depth_msg: Image) -> list[DetObject]:
        """Process synchronized color and depth images (runs in background thread)."""
        if not self._detector or not self._camera_info:
            if self.config.max_input_age_s is not None:
                raise ValueError("Detector or camera intrinsics not ready")
            return []

        if self.config.max_input_age_s is not None:
            validate_rgbd(
                color_msg,
                depth_msg,
                self._camera_info,
                time.time(),
                self.config.max_input_age_s,
                self.config.max_pair_skew_s,
            )
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

        camera_transform = None
        if self._target_frame != color_image.frame_id:
            camera_transform = self.tfbuffer.get(
                self._target_frame,
                color_image.frame_id,
                color_image.ts,
                0.1,
                forward_tolerance=0.2,
            )

        detections_2d: ImageDetections2D[Any]
        if self._detector_backend == "owlv2":
            if not self._text_prompts:
                detections_2d = ImageDetections2D(color_image, [])
            else:
                detections_2d = self._detector.query_detections(
                    color_image,
                    self._text_prompts,
                    threshold=self._detector_confidence,
                )
        elif self._detector_backend == "moondream":
            detections_2d = ImageDetections2D(color_image, [])
            for class_id, prompt in enumerate(self._text_prompts):
                prompted = self._detector.query_detections(color_image, prompt)
                for detection in prompted.detections:
                    # Moondream's per-query indices are not temporal track IDs.
                    detection.track_id = -1
                    detection.class_id = class_id
                detections_2d.detections.extend(prompted.detections)
        else:
            detections_2d = self._detector.process_image(color_image)

        if self._segmenter is not None:
            detections_2d = self._segmenter.segment(detections_2d)

        detections_2d_msg = Detection2DArray(
            detections_length=len(detections_2d.detections),
            header=Header(color_image.ts, color_image.frame_id or ""),
            detections=[det.to_ros_detection2d() for det in detections_2d.detections],
        )
        self.detections_2d.publish(detections_2d_msg)

        overlay = color_image.to_rgb().data.copy()
        if (
            self.config.require_segmentation
            or self.config.min_valid_depth_pixels
            or self.config.min_valid_depth_fraction
        ):
            detections_2d, reports, overlay = filter_depth_detections(
                detections_2d,
                depth_image.data,
                self.config.min_valid_depth_fraction,
                self.config.min_valid_depth_pixels,
                self.config.require_segmentation,
            )
            self._scan_quality = [report.as_dict() for report in reports]
        self._scan_images = (
            Image(overlay, ImageFormat.RGB, frame_id=color_image.frame_id, ts=color_image.ts),
            depth_image,
        )

        # Process 3D detections
        return self._process_3d_detections(
            detections_2d, color_image, depth_image, camera_transform
        )

    def _process_3d_detections(
        self,
        detections_2d: ImageDetections2D[Any],
        color_image: Image,
        depth_image: Image,
        camera_transform: Transform | None,
    ) -> list[DetObject]:
        """Convert 2D detections to 3D and publish."""
        if self._camera_info is None:
            return []

        if self._target_frame != color_image.frame_id and camera_transform is None:
            logger.warning("Failed to lookup transform from camera frame to target frame")
            return []

        # Cache depth and transform together, only after the lookup succeeds.
        self._latest_scene_snapshot = (depth_image, camera_transform, deepcopy(self._camera_info))
        self._scan_capture_ts = color_image.ts

        objects = Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
            max_distance=self._max_distance,
            use_aabb=self._use_aabb,
            max_obstacle_width=self._max_obstacle_width,
        )

        self._scan_objects = objects
        if self.config.max_result_age_s is not None and not self._scene_is_fresh():
            raise ValueError("3D observations exceeded result freshness deadline")

        # Empty observations still advance pending-object expiry.
        observed_objects = self._object_db.add_objects(objects)
        if not objects:
            return []

        # Publish ALL permanent objects so downstream consumers get the full set,
        # not just this frame's batch (which may be a subset of what's on the table).
        all_permanent = self._object_db.get_objects()

        detections_3d = to_detection3d_array(all_permanent)
        self.detections_3d.publish(detections_3d)
        self.objects.publish(all_permanent)

        objects_for_pc = all_permanent
        aggregated_pc = aggregate_pointclouds(objects_for_pc)
        self.pointcloud.publish(aggregated_pc)
        return observed_objects
