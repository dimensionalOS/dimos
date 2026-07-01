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

from dataclasses import dataclass
import hashlib
import threading
import time
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.objectDB import ObjectDB
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import (
    Object,
    aggregate_pointclouds,
    to_detection3d_array,
)
from dimos.perception.detection.world_belief import WorldBelief
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


@dataclass(frozen=True)
class _FrustumConfig:
    enabled: bool = True
    margin_px: float = 24.0
    near_m: float = 0.02


_CAMERA_TRANSLATION_MOTION_THRESHOLD_M = 0.005
_CAMERA_ROTATION_MOTION_THRESHOLD_RAD = 0.01
_CAMERA_MOTION_SETTLE_FRAMES = 6

_OBJECTDB_TTL_S = 5.0

_WB_REACQ = {
    "reacq_window": 30.0,
    "reacq_radius": 1.0,
    "reacq_cos": 0.6,
    "reacq_margin": 0.10,
}
_WB_CANDIDATE = {
    "suppress_partial_observation_creates": True,
    "candidate_gate_new_objects": True,
    "new_candidate_min_age_s": 1.0,
    "new_candidate_ttl_s": 10.0,
    "new_candidate_max_count": 32,
}


def _render_tracking_overlay(
    color_image: Image,
    frame_objects: list[Object],
    present_objects: list[Object],
) -> Image | None:
    img = color_image.to_opencv()
    if img is None or getattr(img, "ndim", 0) != 3:
        return None
    out = img.copy()
    height, width = out.shape[:2]
    present_ids = {o.object_id for o in present_objects}
    seen: set[str] = set()
    visible_ids: set[str] = set()

    for obj in frame_objects:
        oid = getattr(obj, "object_id", "") or "?"
        if oid in seen:
            continue
        seen.add(oid)
        bbox = getattr(obj, "bbox", None)
        if bbox is None:
            continue
        try:
            x1, y1, x2, y2 = (round(float(v)) for v in bbox)
        except Exception:
            continue
        x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
        y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
        if x2 - x1 < 2 or y2 - y1 < 2:
            continue

        visible_ids.add(oid)
        digest = hashlib.md5(oid.encode("utf-8")).digest()
        color = (int(60 + digest[0] % 180), int(60 + digest[1] % 180), int(60 + digest[2] % 180))
        state = "present" if oid in present_ids else "warming"
        label = f"{oid[:6]} {getattr(obj, 'name', '') or 'object'} {state}"
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2 if oid in present_ids else 1)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        top = max(0, y1 - th - 7)
        cv2.rectangle(out, (x1, top), (min(width - 1, x1 + tw + 5), y1), color, -1)
        cv2.putText(
            out,
            label,
            (x1 + 2, max(th + 1, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
            lineType=cv2.LINE_AA,
        )

    header = f"WorldBelief: {len(visible_ids)} visible, {len(present_objects)} present"
    memory_only = [o for o in present_objects if o.object_id not in visible_ids]
    if memory_only:
        header += f", {len(memory_only)} memory"
        text = "memory: " + ", ".join(
            f"{o.object_id[:6]} {getattr(o, 'name', '') or 'object'}" for o in memory_only[:4]
        )
        if len(memory_only) > 4:
            text += f" +{len(memory_only) - 4}"
        cv2.putText(
            out,
            text,
            (8, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (220, 220, 220),
            1,
            lineType=cv2.LINE_AA,
        )
    cv2.putText(
        out,
        header,
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        lineType=cv2.LINE_AA,
    )
    return Image.from_opencv(out, frame_id=color_image.frame_id, ts=color_image.ts)


def _camera_intrinsics_for_image(
    camera_info: CameraInfo,
    color_image: Image,
) -> tuple[float, float, float, float, int, int] | None:
    K = camera_info.get_K_matrix()
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        return None

    width = int(color_image.width)
    height = int(color_image.height)
    if width <= 0 or height <= 0:
        return None

    info_width = int(getattr(camera_info, "width", 0) or width)
    info_height = int(getattr(camera_info, "height", 0) or height)
    sx = width / info_width if info_width > 0 else 1.0
    sy = height / info_height if info_height > 0 else 1.0
    return fx * sx, fy * sy, cx * sx, cy * sy, width, height


def _object_frustum_points(obj: Object) -> NDArray[np.float64] | None:
    center = getattr(obj, "center", None)
    size = getattr(obj, "size", None)
    if center is None or size is None:
        return None

    hx = max(float(size.x), 1e-3) * 0.5
    hy = max(float(size.y), 1e-3) * 0.5
    hz = max(float(size.z), 1e-3) * 0.5
    points = [[float(center.x), float(center.y), float(center.z), 1.0]]
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                points.append(
                    [
                        float(center.x) + dx,
                        float(center.y) + dy,
                        float(center.z) + dz,
                        1.0,
                    ]
                )
    return np.asarray(points, dtype=np.float64)


def _is_object_inside_camera_frustum(
    obj: Object,
    *,
    camera_info: CameraInfo,
    color_image: Image,
    target_to_camera: Transform,
    config: _FrustumConfig,
) -> bool | None:
    """Return whether a target-frame object should be visible in the current camera image.

    None means the object lacks enough geometry/calibration for a safe decision; callers should not
    exempt it from eviction based on an unknown.
    """
    intrinsics = _camera_intrinsics_for_image(camera_info, color_image)
    if intrinsics is None:
        return None
    fx, fy, cx, cy, width, height = intrinsics

    points = _object_frustum_points(obj)
    if points is None:
        return None

    cam_points = (target_to_camera.to_matrix() @ points.T).T[:, :3]
    for x, y, z in cam_points:
        z = float(z)
        if z <= config.near_m:
            continue
        u = fx * (float(x) / z) + cx
        v = fy * (float(y) / z) + cy
        margin = config.margin_px
        if -margin <= u <= width + margin and -margin <= v <= height + margin:
            return True
    return False


def _frustum_exempt_object_ids(
    objects: list[Object],
    *,
    camera_info: CameraInfo,
    color_image: Image,
    camera_transform: Transform | None,
    config: _FrustumConfig,
) -> set[str]:
    if not config.enabled or not objects:
        return set()
    target_to_camera = (
        camera_transform.inverse()
        if camera_transform is not None
        else Transform(
            frame_id=color_image.frame_id or "",
            child_frame_id=color_image.frame_id or "",
            ts=color_image.ts,
        )
    )
    exempt: set[str] = set()
    for obj in objects:
        inside = _is_object_inside_camera_frustum(
            obj,
            camera_info=camera_info,
            color_image=color_image,
            target_to_camera=target_to_camera,
            config=config,
        )
        if inside is False:
            exempt.add(obj.object_id)
    return exempt


class ObjectSceneRegistrationModule(Module):
    """Module for registering detector observations as stable 3D scene objects."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]

    detections_2d: Out[Detection2DArray]
    detections_3d: Out[Detection3DArray]
    objects: Out[list[Object]]
    pointcloud: Out[PointCloud2]
    annotated_image: Out[Image]
    worldbelief_audit: Out[dict]

    _detector: Detector | None = None
    _camera_info: CameraInfo | None = None
    _object_db: ObjectDB | WorldBelief
    _latest_depth_image: Image | None = None
    _latest_camera_transform: Any = None

    def __init__(
        self,
        target_frame: str = "map",
        prompt_mode: Any = "lrpc",
        belief_engine: str = "objectdb",
        text_prompts: list[str] | None = None,
        detector_model_name: str | None = None,
        detector_device: str | None = None,
        detector_conf: float = 0.6,
        detector_iou: float = 0.6,
        detector_max_det: int | None = None,
        detector_max_area_ratio: float | None = 0.3,
        detector_tracking_enabled: bool = True,
        embedding_device: str | None = None,
        embedding_model_id: str = "openai/clip-vit-base-patch32",
        compute_visual_embeddings: bool = False,
        visual_embedding_model_id: str = "facebook/dinov2-base",
        visual_embedding_device: str | None = None,
        distance_threshold: float = 0.2,
        #   min_detections_for_permanent -> ObjectDB: cumulative detections (ever) to
        #     promote a pending object to permanent.
        #   min_support -> WorldBelief: detections within recent_window to trust an
        #     entity as "present".
        min_detections_for_permanent: int = 6,
        min_support: int = 4,
        recent_window: float = 8,
        eviction_ttl_s: float = 60.0,
        max_distance: float = 0.0,
        use_aabb: bool = False,
        max_obstacle_width: float = 0.0,
        history_path: str | None = None,
        compute_embeddings: bool = False,
        # On/off toggles for the WorldBelief-only frustum + camera-motion lifecycle handling
        frustum_eviction_exemption: bool = False,
        camera_motion_marks_partial: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._target_frame = target_frame
        self._prompt_mode = prompt_mode
        self._text_prompts = tuple(text_prompts) if text_prompts else None
        self._detector_model_name = detector_model_name
        self._detector_device = detector_device
        self._detector_conf = detector_conf
        self._detector_iou = detector_iou
        self._detector_max_det = detector_max_det
        self._detector_max_area_ratio = detector_max_area_ratio
        self._detector_tracking_enabled = detector_tracking_enabled
        self._embedding_device = embedding_device
        self._embedding_model_id = embedding_model_id
        self._compute_visual_embeddings = compute_visual_embeddings
        self._visual_embedding_model_id = visual_embedding_model_id
        self._visual_embedding_device = visual_embedding_device or embedding_device
        self._history_path = history_path
        self._compute_embeddings = compute_embeddings
        self._embedder: Any = None
        self._visual_embedder: Any = None
        self._active = False
        self._processing_lock = threading.RLock()
        self._last_camera_observation_transform: Transform | None = None
        self._camera_motion_grace_frames_remaining = 0
        self._camera_motion_marks_partial = camera_motion_marks_partial
        if belief_engine == "objectdb":
            self._object_db = ObjectDB(
                distance_threshold=distance_threshold,
                min_detections_for_permanent=min_detections_for_permanent,
                pending_ttl_s=_OBJECTDB_TTL_S,
                track_id_ttl_s=_OBJECTDB_TTL_S,
            )
        elif belief_engine == "world_belief":
            self._object_db = WorldBelief(
                distance_threshold=distance_threshold,
                min_support=min_support,
                recent_window=recent_window,
                eviction_ttl_s=eviction_ttl_s,
                history_path=history_path,
                enable_history=history_path is not None,
                **_WB_REACQ,
                **_WB_CANDIDATE,
            )
        else:
            raise ValueError(
                f"unknown belief engine: {belief_engine!r} (use 'world_belief' or 'objectdb')"
            )
        self._max_distance = max_distance
        self._use_aabb = use_aabb
        self._max_obstacle_width = max_obstacle_width
        self._frustum_config = _FrustumConfig(enabled=frustum_eviction_exemption)

    def _prompt_mode_value(self) -> str:
        return str(getattr(self._prompt_mode, "value", self._prompt_mode or "lrpc")).lower()

    def _coerce_yoloe_prompt_mode(self) -> Any:
        from dimos.perception.detection.detectors.yoloe import YoloePromptMode

        value = self._prompt_mode_value()
        try:
            return YoloePromptMode(value)
        except ValueError as exc:
            allowed = ", ".join(mode.value for mode in YoloePromptMode)
            raise ValueError(f"Unknown YOLO-E prompt mode {value!r}. Available: {allowed}") from exc

    def _apply_detector_prompts(self, detector: Detector) -> None:
        if self._prompt_mode_value() != "prompt" or not self._text_prompts:
            return
        set_prompts = getattr(detector, "set_prompts", None)
        if not callable(set_prompts):
            logger.warning(
                "Configured prompt text ignored because detector does not support set_prompts"
            )
            return
        set_prompts(text=list(self._text_prompts))

    def _detector_provenance(self) -> dict[str, str | None]:
        detector = self._detector
        if detector is None:
            return {}
        detector_id = getattr(detector, "detector_id", type(detector).__name__)
        detector_model = getattr(detector, "detector_model_name", None)
        tracker_source = getattr(detector, "tracker_source", None)
        if not self._detector_tracking_enabled:
            tracker_source = None
        return {
            "detector_id": str(detector_id) if detector_id is not None else None,
            "detector_model": str(detector_model) if detector_model is not None else None,
            "tracker_source": str(tracker_source) if tracker_source is not None else None,
            "observation_source": "object_scene_registration",
        }

    @rpc
    def start(self) -> None:
        super().start()
        self._active = True

        # Best-effort cross-session restore from the configured WorldBelief history DB.
        if self._history_path is not None and hasattr(self._object_db, "rehydrate"):
            try:
                self._object_db.rehydrate()
            except Exception as e:
                logger.warning(
                    f"WorldBelief cross-session rehydrate failed (continuing fresh): {e}"
                )

        from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode

        prompt_mode = self._coerce_yoloe_prompt_mode()
        detector_model_name = self._detector_model_name
        if detector_model_name is None:
            detector_model_name = (
                "yoloe-11l-seg-pf.pt"
                if prompt_mode == YoloePromptMode.LRPC
                else "yoloe-11l-seg.pt"
            )

        self._detector = Yoloe2DDetector(
            model_name=detector_model_name,
            device=self._detector_device,
            prompt_mode=prompt_mode,
            max_area_ratio=self._detector_max_area_ratio,
            conf=self._detector_conf,
            iou=self._detector_iou,
            max_det=self._detector_max_det,
        )
        self._apply_detector_prompts(self._detector)

        if self._compute_embeddings and self._embedder is None:
            try:
                from dimos.models.embedding.clip import CLIPModel

                kwargs = {"model_name": self._embedding_model_id}
                if self._embedding_device is not None:
                    kwargs["device"] = self._embedding_device
                self._embedder = CLIPModel(**kwargs)
                start = getattr(self._embedder, "start", None)
                if callable(start):
                    start()
                    logger.info(
                        "CLIP embedder loaded model=%s device=%s",
                        self._embedding_model_id,
                        self._embedding_device,
                    )
            except Exception as e:
                self._embedder = None
                raise RuntimeError("semantic embedder requested but unavailable") from e

        if self._compute_visual_embeddings and self._visual_embedder is None:
            try:
                from dimos.models.embedding.dino import DINOModel

                kwargs = {"model_name": self._visual_embedding_model_id}
                if self._visual_embedding_device is not None:
                    kwargs["device"] = self._visual_embedding_device
                self._visual_embedder = DINOModel(**kwargs)
                start = getattr(self._visual_embedder, "start", None)
                if callable(start):
                    start()
                    logger.info(
                        "DINO embedder loaded model=%s device=%s",
                        self._visual_embedding_model_id,
                        self._visual_embedding_device,
                    )
            except Exception as e:
                self._visual_embedder = None
                raise RuntimeError("visual embedder requested but unavailable") from e

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

        self._active = False

        detector = self._detector
        self._detector = None
        if detector:
            stop = getattr(detector, "stop", None)
            if callable(stop):
                stop()

        for embedder_name in ("_embedder", "_visual_embedder"):
            embedder = getattr(self, embedder_name, None)
            setattr(self, embedder_name, None)
            stop = getattr(embedder, "stop", None)
            if callable(stop):
                stop()

        with self._processing_lock:
            self._object_db.clear()
            close = getattr(self._object_db, "close", None)
            if callable(close):
                close()

        logger.info("ObjectSceneRegistrationModule stopped")
        super().stop()

    @rpc
    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None:
        """Set prompts for detection. Provide either text or bboxes, not both."""
        if self._detector is None:
            return
        set_prompts = getattr(self._detector, "set_prompts", None)
        if not callable(set_prompts):
            logger.warning("Current detector does not support prompts")
            return
        set_prompts(text=text, bboxes=bboxes)

    @rpc
    def select_object(self, object_ref: int | str) -> dict[str, Any] | None:
        """Get object data by stable object_id or detector track_id and promote it."""
        ref = str(object_ref)
        for obj in self._object_db.get_all_objects():
            if obj.object_id == ref or str(obj.track_id) == ref:
                self._object_db.promote(obj.object_id)
                return obj.to_dict()
        return None

    @rpc
    def get_object_track_ids(self) -> list[int]:
        """Get track_ids of currently present objects."""
        return [obj.track_id for obj in self._object_db.get_objects()]

    @rpc
    def get_detected_objects(self) -> list[dict[str, Any]]:
        """Get currently present objects with object_id (UUID) and name."""
        return [obj.agent_encode() for obj in self._object_db.get_objects()]

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        """Get pointcloud for an object by class name."""
        objects = self._object_db.find_by_name(name)
        return objects[0].pointcloud if objects else None

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """Get pointcloud for an object by its stable object_id (searches all objects)."""
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
        if self._latest_depth_image is None or self._camera_info is None:
            return None

        depth_cv = self._latest_depth_image.to_opencv()
        h, w = depth_cv.shape[:2]

        # Zero out excluded object's depth
        if exclude_object_id:
            exclude_mask = self._get_object_mask(exclude_object_id)
            if exclude_mask is not None:
                depth_cv = depth_cv.copy()
                depth_cv[exclude_mask > 0] = 0

        K = self._camera_info.get_K_matrix()
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            w, h, float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        )

        depth_o3d = o3d.geometry.Image(depth_cv.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d, intrinsic, depth_scale=1.0, depth_trunc=depth_trunc
        )

        if len(pcd.points) < 100:
            return None

        pcd = pcd.voxel_down_sample(voxel_size)

        pc = PointCloud2(
            pcd,
            frame_id=self._latest_depth_image.frame_id,
            ts=self._latest_depth_image.ts,
        )

        if self._latest_camera_transform is not None:
            pc = pc.transform(self._latest_camera_transform)

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

        set_prompts = getattr(self._detector, "set_prompts", None)
        if not callable(set_prompts):
            return "Current detector does not support prompts."
        set_prompts(text=list(prompts))
        time.sleep(2.0)

        detected = self.get_detected_objects()
        if not detected:
            return "No objects detected."

        obj_list = [f"  - {obj['name']} (object_id='{obj['object_id']}')" for obj in detected]
        return f"Detected {len(detected)} object(s):\n" + "\n".join(obj_list)

    @skill
    def select(self, object_ref: int | str) -> str:
        """Select an object by stable object_id or detector track_id and promote it.

        Example:
            select("2f4c...")
            select(5)
        """
        result = self.select_object(object_ref)
        if result is None:
            return f"No object found with object_id or track_id {object_ref!r}."
        return f"Selected object {object_ref}: {result['name']}"

    def _on_aligned_frames(self, frames) -> None:  # type: ignore[no-untyped-def]
        color_msg, depth_msg = frames
        self._process_images(color_msg, depth_msg)

    def _process_images(self, color_msg: Image, depth_msg: Image) -> None:
        """Process synchronized color and depth images (runs in background thread)."""
        detector = self._detector
        if not self._active or detector is None or self._camera_info is None:
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
        detections_2d: ImageDetections2D[Any] = detector.process_image(color_image)
        if not self._active:
            return

        detections_2d_msg = Detection2DArray(
            detections_length=len(detections_2d.detections),
            header=Header(color_image.ts, color_image.frame_id or ""),
            detections=[det.to_ros_detection2d() for det in detections_2d.detections],
        )
        self.detections_2d.publish(detections_2d_msg)

        # Process 3D detections
        self._process_3d_detections(detections_2d, color_image, depth_image)

    def _process_3d_detections(
        self,
        detections_2d: ImageDetections2D[Any],
        color_image: Image,
        depth_image: Image,
    ) -> None:
        """Convert 2D detections to 3D and publish."""
        if not self._active or self._camera_info is None:
            return

        # Cache depth image for full scene pointcloud generation
        self._latest_depth_image = depth_image

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
                logger.info("Failed to lookup transform from camera frame to target frame")
                evict_exempt = self._all_maintained_object_ids()
                self._advance_belief_time(color_image.ts, evict_exempt)
                self._publish_present_set(
                    color_image,
                    self._object_db.get_objects(),
                    frame_objects=[],
                    pointcloud_objects=[],
                )
                return

        # Cache camera transform for full scene pointcloud
        self._latest_camera_transform = camera_transform
        camera_moving = (
            self._camera_transform_moved(camera_transform)
            if self._camera_motion_marks_partial
            else False
        )
        evict_exempt = self._frustum_evict_exempt(color_image, camera_transform)

        objects = Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
            max_distance=self._max_distance,
            use_aabb=self._use_aabb,
            max_obstacle_width=self._max_obstacle_width,
            **self._detector_provenance(),
        )
        if not self._active:
            return
        width = int(color_image.width)
        height = int(color_image.height)
        for obj in objects:
            bbox = getattr(obj, "bbox", None)
            if bbox is None:
                continue
            try:
                x1, y1, x2, y2 = (float(v) for v in bbox)
            except Exception:
                continue
            obj.observation_partial = (
                x1 <= 3.0 or y1 <= 3.0 or x2 >= width - 3.0 or y2 >= height - 3.0
            )
            if camera_moving:
                obj.observation_partial = True
                obj.observation_partial_from_camera_motion = True

        if not objects:
            self._advance_belief_time(color_image.ts, evict_exempt)
            self._publish_present_set(
                color_image,
                self._object_db.get_objects(),
                frame_objects=[],
                pointcloud_objects=[],
            )
            return

        self._attach_embeddings(objects, color_image)
        if not self._active:
            return

        frame_objects = self._add_objects_to_memory(objects, evict_exempt, color_image.ts)

        present_objects = self._object_db.get_objects()
        self._publish_present_set(
            color_image,
            present_objects,
            frame_objects=frame_objects,
            pointcloud_objects=objects,
        )
        return

    def _strip_detector_tracking(self, objects: list[Object]) -> None:
        if self._detector_tracking_enabled:
            return
        for obj in objects:
            obj.track_id = -1
            obj.tracker_source = None

    def _all_maintained_object_ids(self) -> set[str]:
        if not isinstance(self._object_db, WorldBelief):
            return set()
        return {obj.object_id for obj in self._object_db.get_all_objects()}

    def _camera_transform_moved(self, camera_transform: Transform | None) -> bool:
        if camera_transform is None:
            self._last_camera_observation_transform = None
            self._camera_motion_grace_frames_remaining = 0
            return False
        prev = self._last_camera_observation_transform
        self._last_camera_observation_transform = camera_transform
        if prev is None:
            return False
        dp = camera_transform.translation.distance(prev.translation)
        dr = prev.rotation.normalize().angle_to(camera_transform.rotation.normalize())
        moved = (
            dp > _CAMERA_TRANSLATION_MOTION_THRESHOLD_M
            or dr > _CAMERA_ROTATION_MOTION_THRESHOLD_RAD
        )
        if moved:
            self._camera_motion_grace_frames_remaining = _CAMERA_MOTION_SETTLE_FRAMES
            return True
        if self._camera_motion_grace_frames_remaining > 0:
            self._camera_motion_grace_frames_remaining -= 1
            return True
        return False

    def _frustum_evict_exempt(
        self,
        color_image: Image,
        camera_transform: Transform | None,
    ) -> set[str]:
        if self._camera_info is None or not isinstance(self._object_db, WorldBelief):
            return set()
        return _frustum_exempt_object_ids(
            self._object_db.get_all_objects(),
            camera_info=self._camera_info,
            color_image=color_image,
            camera_transform=camera_transform,
            config=self._frustum_config,
        )

    def _advance_belief_time(self, frame_ts: float, evict_exempt: set[str]) -> None:
        with self._processing_lock:
            if isinstance(self._object_db, WorldBelief):
                self._object_db.advance_time(frame_ts, evict_exempt=evict_exempt)
            else:
                self._object_db.add_objects([])

    def _add_objects_to_memory(
        self,
        objects: list[Object],
        evict_exempt: set[str],
        frame_ts: float,
    ) -> list[Object]:
        self._strip_detector_tracking(objects)
        with self._processing_lock:
            if isinstance(self._object_db, WorldBelief):
                return self._object_db.add_objects(
                    objects,
                    evict_exempt=evict_exempt,
                    frame_ts=frame_ts,
                )
            return self._object_db.add_objects(objects)

    def _publish_worldbelief_audit(
        self,
        color_image: Image,
        present_objects: list[Object],
        frame_objects: list[Object] | None = None,
    ) -> None:
        if not isinstance(self._object_db, WorldBelief):
            return
        frame_objects = frame_objects if frame_objects is not None else present_objects
        try:
            stats = self._object_db.get_last_add_stats()
            event = {
                "type": "worldbelief_frame_audit",
                "ts": float(color_image.ts),
                "frame_id": color_image.frame_id or "",
                "target_frame": self._target_frame,
                "detector_tracking_enabled": self._detector_tracking_enabled,
                "present_count": len(present_objects),
                "maintained_count": len(self._object_db.get_all_objects()),
                "present_ids": [obj.object_id for obj in present_objects],
                "frame_object_ids": [obj.object_id for obj in frame_objects],
                "stats": stats,
                "association_evidence": self._object_db.get_last_association_evidence(),
                "gallery_stats": self._object_db.get_identity_gallery_stats(),
                "table": self._object_db.get_table_snapshot(),
            }
            self.worldbelief_audit.publish(event)
        except Exception as exc:
            logger.debug("Failed to publish WorldBelief audit event: %s", exc)

    def _publish_present_set(
        self,
        color_image: Image,
        present_objects: list[Object],
        frame_objects: list[Object] | None = None,
        pointcloud_objects: list[Object] | None = None,
    ) -> None:
        detections_3d = to_detection3d_array(
            present_objects, frame_id=self._target_frame, ts=color_image.ts
        )
        self.detections_3d.publish(detections_3d)
        self.objects.publish(present_objects)

        overlay = _render_tracking_overlay(
            color_image,
            frame_objects=frame_objects if frame_objects is not None else present_objects,
            present_objects=present_objects,
        )
        if overlay is not None:
            self.annotated_image.publish(overlay)

        pointcloud_objects = pointcloud_objects if pointcloud_objects is not None else []
        if pointcloud_objects:
            aggregated_pc = aggregate_pointclouds(pointcloud_objects)
        else:
            aggregated_pc = PointCloud2(
                pointcloud=o3d.geometry.PointCloud(),
                frame_id=self._target_frame,
                ts=color_image.ts,
            )
        self.pointcloud.publish(aggregated_pc)
        self._publish_worldbelief_audit(color_image, present_objects, frame_objects)

    def _attach_embeddings(self, objects: list[Object], color_image: Image) -> None:
        def attach(model: Any, target: str) -> None:
            crops: list[Image] = []
            idxs: list[int] = []
            for i, obj in enumerate(objects):
                if getattr(obj, "bbox", None) is None:
                    continue
                crop = obj.cropped_image(padding=0).to_rgb()
                if crop.width < 2 or crop.height < 2:
                    continue
                crop.frame_id = color_image.frame_id
                crop.ts = color_image.ts
                crops.append(crop)
                idxs.append(i)
            if not crops:
                return

            try:
                embeddings = model.embed(*crops)
                if not isinstance(embeddings, list):
                    embeddings = [embeddings]
                model_name = getattr(model, "model_name", None) or getattr(
                    getattr(model, "config", None), "model_name", None
                )
                device = getattr(model, "device", None)
                for i, embedding in zip(idxs, embeddings, strict=False):
                    emb = embedding.to_numpy().reshape(-1).astype(np.float32)
                    if target == "visual":
                        objects[i].visual_embedding = emb
                        objects[i].visual_embedding_model = model_name
                        objects[i].visual_embedding_device = device
                        objects[i].visual_embedding_dim = int(emb.size)
                    else:
                        objects[i].embedding = emb
                        objects[i].embedding_model = model_name
                        objects[i].embedding_device = device
                        objects[i].embedding_dim = int(emb.size)
            except Exception as exc:
                logger.warning("%s embedding step failed (skipping this frame): %s", target, exc)

        if self._embedder is not None:
            attach(self._embedder, "semantic")
        if self._visual_embedder is not None:
            attach(self._visual_embedder, "visual")
